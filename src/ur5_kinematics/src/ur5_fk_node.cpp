#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iomanip>
#include <unordered_map>
// 新增：用于发布误差数据的头文件
#include <std_msgs/Float64MultiArray.h>

#define PI 3.1415926535

// 纯数据全局变量
KDL::JntArray current_q(6);
KDL::Chain ur5_chain;
KDL::ChainFkSolverPos_recursive* fk_solver = nullptr;
ros::Time latest_joint_time;

// 关节名称
const std::vector<std::string> joint_names = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

// 初始化DH参数
void initUR5Chain() {
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, PI/2, 0.089159, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.425, 0, 0, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.39225, 0, 0, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, PI/2, 0.10915, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, -PI/2, 0.09465, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, 0, 0.0823, 0)));
    fk_solver = new KDL::ChainFkSolverPos_recursive(ur5_chain);
}

// 关节回调（哈希表匹配，杜绝读错顺序）
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    std::unordered_map<std::string, int> name_to_idx;
    for (int i = 0; i < msg->name.size(); ++i) {
        name_to_idx[msg->name[i]] = i;
    }

    for (int i = 0; i < 6; ++i) {
        if (name_to_idx.count(joint_names[i])) {
            current_q(i) = msg->position[name_to_idx[joint_names[i]]];
        }
    }
    latest_joint_time = msg->header.stamp;
}

// 姿态误差计算
double calculateOrientationError(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2) {
    tf2::Quaternion tq1, tq2;
    tf2::convert(q1, tq1);
    tf2::convert(q2, tq2);
    tq1.normalize();
    tq2.normalize();
    double dot = tq1.dot(tq2);
    
    dot = std::max(std::min(dot, 1.0), -1.0);
    return 2 * acos(fabs(dot)) * 180.0 / PI;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "ur5_fk_verify");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    initUR5Chain();
    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 50, jointStateCallback);
    
    // ====================== 新增：误差数据发布者（给rqt用） ======================
    ros::Publisher error_pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/pose_error", 10);
    
    ros::Rate rate(10);
    ROS_INFO("UR5正运动学验证节点启动，误差话题：/ur5/pose_error");

    while (ros::ok()) {
        ros::spinOnce();

        KDL::Frame ee_dh;
        fk_solver->JntToCart(current_q, ee_dh);
        KDL::Frame align_transform = KDL::Frame(KDL::Rotation::RotZ(PI));
        KDL::Frame ee_result = align_transform * ee_dh;

        geometry_msgs::TransformStamped tf_true;
        try {
            // 修复：使用ros::Time(0)最新TF，彻底杜绝时间越界崩溃
            tf_true = tf_buffer.lookupTransform("base_link", "wrist_3_link", ros::Time(0), ros::Duration(0.05));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF获取失败: %s", ex.what());
            continue;
        }

        // 数据转换
        geometry_msgs::Pose dh_pose;
        dh_pose.position.x = ee_result.p.x();
        dh_pose.position.y = ee_result.p.y();
        dh_pose.position.z = ee_result.p.z();
        ee_result.M.GetQuaternion(dh_pose.orientation.x, dh_pose.orientation.y,
                                  dh_pose.orientation.z, dh_pose.orientation.w);

        // 计算误差
        double dx = dh_pose.position.x - tf_true.transform.translation.x;
        double dy = dh_pose.position.y - tf_true.transform.translation.y;
        double dz = dh_pose.position.z - tf_true.transform.translation.z;
        double pos_error = sqrt(dx*dx + dy*dy + dz*dz) * 1000;
        
        double ori_error = calculateOrientationError(dh_pose.orientation, tf_true.transform.rotation);

        // ====================== 新增：发布误差数据到ROS话题 ======================
        std_msgs::Float64MultiArray error_msg;
        error_msg.data.clear();
        error_msg.data.push_back(pos_error);    // 索引0：位置误差(mm)
        error_msg.data.push_back(ori_error);    // 索引1：姿态误差(°)
        error_pub.publish(error_msg);

        // 打印结果
        ROS_INFO("\n===== 精度对比 =====");
        ROS_INFO("位置误差: %.3f mm | 姿态误差: %.3f °", pos_error, ori_error);

        rate.sleep();
    }

    delete fk_solver;
    return 0;
}