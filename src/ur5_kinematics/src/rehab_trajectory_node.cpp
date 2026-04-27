#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/service_client.h>
#include <locale.h>
#include <deque>
#include "ur5_kinematics/IkService.h"

#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

#define PI 3.1415926535
//和time_from_start 完全一致（硬延时）
const double CONTROL_DELAY = 0.1;
const double MAX_CACHE = 0.3;

struct TargetPose
{
    ros::Time stamp;
    double x, y, z;
};
std::deque<TargetPose> target_history;

nav_msgs::Path ee_path;
ros::ServiceClient ik_client;
int dir = 1;

std::vector<double> actual_joints(6, 0.0);
double actual_xyz[3] = {0,0,0};
double total_error=0, avg_error=0, max_error=0;
int error_count=0;
bool first_error_clear=false;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
ros::Publisher pub, path_pub, pose_data_pub, error_data_pub;

void quinticInterp(double t, double T, double p0, double p1, double &pos)
{
    double tau = std::min(t/T, 1.0);
    pos = p0 + (p1-p0)*(10*pow(tau,3)-15*pow(tau,4)+6*pow(tau,5));
}

double trajectory1(double t, double xyz[3], double &roll, double &pitch, double &yaw)
{
    const double T = 5.0;
    t = std::min(t, T);
    if(dir == 1){
        quinticInterp(t, T, -0.4, -0.6, xyz[0]);
        quinticInterp(t, T,  0.0, -0.2, xyz[1]);
        quinticInterp(t, T,  0.6,  0.5, xyz[2]);
    }else{
        quinticInterp(t, T, -0.6, -0.4, xyz[0]);
        quinticInterp(t, T, -0.2,  0.0, xyz[1]);
        quinticInterp(t, T,  0.5,  0.6, xyz[2]);
    }
    roll = PI; pitch=0; yaw=0; return T;
}
double trajectory2(double t, double xyz[3], double &roll, double &pitch, double &yaw)
{
    const double T = 6.0;
    const double cx = -0.5;
    const double cy = 0.0;
    const double cz = 0.5;
    const double r = 0.15;
    t = std::min(t, T);
    double angle;
    if(dir == 1){
        quinticInterp(t, T, 0.0, PI, angle);
    }else{
        quinticInterp(t, T, PI, 0.0, angle);
    }
    xyz[0] = cx + r * cos(angle);
    xyz[1] = cy;
    xyz[2] = cz + r * sin(angle);
    roll = PI;
    pitch = 0;
    yaw = 0.0;
    return T;
}

double trajectory3(double t, double xyz[3], double &roll, double &pitch, double &yaw)
{
    const double T = 8.0;
    const double cx = -0.5;
    const double cy = 0.0;
    const double cz = 0.5;
    const double r = 0.1;
    t = std::min(t, T);
    double angle;
    if(dir == 1){
        quinticInterp(t, T, 0.0, 2*PI, angle);
    }else{
        quinticInterp(t, T, 2*PI, 0.0, angle);
    }
    xyz[0] = cx;
    xyz[1] = cy + r * cos(angle);
    xyz[2] = cz + r * sin(angle);
    roll = 0;
    pitch = -PI;
    yaw = 0.0;
    return T;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::vector<std::string> names = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    for(int i=0;i<6;i++) for(int j=0;j<(int)msg->name.size();j++) if(msg->name[j]==names[i]) { actual_joints[i]=msg->position[j]; break; }
}

//高精度插值
bool getInterpolatedTarget(ros::Time target_time, double &x, double &y, double &z)
{
    if(target_history.size() < 2) return false;
    TargetPose p0, p1;
    bool found = false;
    for(int i=1; i<(int)target_history.size(); i++){
        if(target_history[i-1].stamp <= target_time && target_history[i].stamp >= target_time){
            p0 = target_history[i-1];
            p1 = target_history[i];
            found = true;
            break;
        }
    }
    if(!found) return false;
    double dt = (p1.stamp - p0.stamp).toSec();
    double ratio = (target_time - p0.stamp).toSec() / dt;
    x = p0.x + (p1.x - p0.x)*ratio;
    y = p0.y + (p1.y - p0.y)*ratio;
    z = p0.z + (p1.z - p0.z)*ratio;
    return true;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "rehab_trajectory_node");
    ros::NodeHandle nh;

    pub = nh.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/end_effector_path", 10);
    pose_data_pub = nh.advertise<std_msgs::Float64MultiArray>("/trajectory_verify/pose_data", 10);
    error_data_pub = nh.advertise<std_msgs::Float64MultiArray>("/trajectory_verify/error_data", 10);

    ik_client = nh.serviceClient<ur5_kinematics::IkService>("/ur5_compute_ik");
    ee_path.header.frame_id = "base_link";
    std::vector<double> q_prev = {0.0, -0.4, 0.4, -0.8, -1.57, 0.0};
    ros::Rate rate(50);
    double t=0, cur_T=5;

    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    nh.subscribe("/joint_states", 10, jointStateCallback);
    ros::Duration(1.0).sleep();

    while (ros::ok())
    {
        double tar_xyz[3];
        double roll, pitch, yaw;
        cur_T = trajectory3(t, tar_xyz, roll, pitch, yaw);
        ros::Time now = ros::Time::now();

        // 缓存目标
        TargetPose tmp{now, tar_xyz[0], tar_xyz[1], tar_xyz[2]};
        target_history.push_back(tmp);
        // 清理缓存
        while(!target_history.empty() && (now - target_history.front().stamp).toSec() > MAX_CACHE)
            target_history.pop_front();

        // 逆解 + 轨迹发布
        ur5_kinematics::IkService srv;
        srv.request.target_x = -tar_xyz[0];
        srv.request.target_y = -tar_xyz[1];
        srv.request.target_z = tar_xyz[2];
        srv.request.roll = roll; srv.request.pitch = pitch; srv.request.yaw = yaw;
        srv.request.joint_prev = q_prev;
        if(ik_client.call(srv) && srv.response.success) q_prev = srv.response.joint_positions;

        // 发布期望轨迹
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id="base_link"; pose.header.stamp=now;
        pose.pose.position.x=tar_xyz[0]; pose.pose.position.y=tar_xyz[1]; pose.pose.position.z=tar_xyz[2];
        ee_path.poses.push_back(pose); path_pub.publish(ee_path);

        // 发布控制指令
        trajectory_msgs::JointTrajectory traj;
        traj.header.stamp=now; traj.header.frame_id="base_link";
        traj.joint_names = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = q_prev;
        pt.time_from_start = ros::Duration(CONTROL_DELAY); // 硬延时0.1s
        traj.points.push_back(pt);
        pub.publish(traj);

        //终极时间戳对齐
        double aligned_x=0, aligned_y=0, aligned_z=0;
        double cur_error = 0;
        try
        {
            // TF
            auto trans = tf_buffer->lookupTransform("base_link", "wrist_3_link", ros::Time(0));
            actual_xyz[0] = trans.transform.translation.x;
            actual_xyz[1] = trans.transform.translation.y;
            actual_xyz[2] = trans.transform.translation.z;

            //实际时间 - 控制延时 = 精准目标时间
            ros::Time ideal_target_time = trans.header.stamp - ros::Duration(CONTROL_DELAY);
            // 高精度插值获取目标
            if(getInterpolatedTarget(ideal_target_time, aligned_x, aligned_y, aligned_z))
            {
                // 误差计算 时间对齐）
                double dx = aligned_x - actual_xyz[0];
                double dy = aligned_y - actual_xyz[1];
                double dz = aligned_z - actual_xyz[2];
                cur_error = sqrt(dx*dx + dy*dy + dz*dz) * 1000;

                // 误差统计
                if(!first_error_clear && cur_error < 1.0) {total_error=0; error_count=0; max_error=0; first_error_clear=true;}
                total_error += cur_error;
                error_count++;
                avg_error = total_error / error_count;
                if(cur_error>max_error) max_error=cur_error;
            }
        }
        catch(...){ cur_error=0; }

        // 发布数据
        std_msgs::Float64MultiArray pose_msg;
        pose_msg.data = {aligned_x, aligned_y, aligned_z, actual_xyz[0], actual_xyz[1], actual_xyz[2]};
        pose_data_pub.publish(pose_msg);

        std_msgs::Float64MultiArray error_msg;
        error_msg.data = {cur_error, avg_error, max_error};
        error_data_pub.publish(error_msg);

        ROS_INFO("精准误差: %.2fmm | 平均: %.2fmm | 最大: %.2fmm", cur_error, avg_error, max_error);

        t += 0.02;
        if(t >= cur_T) {dir*=-1; t=0; ee_path.poses.clear(); total_error=0; error_count=0; max_error=0; first_error_clear=false;}

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}