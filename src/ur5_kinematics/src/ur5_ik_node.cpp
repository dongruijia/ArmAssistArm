#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <cmath>
#include <locale.h>
#include "ur5_kinematics/IkService.h"

#define PI 3.1415926535
#define RAD_TO_DEG (180.0/PI)

KDL::Chain ur5_chain;
KDL::ChainFkSolverPos_recursive* fk_solver;
KDL::ChainIkSolverVel_pinv* ik_vel_solver;
KDL::ChainIkSolverPos_NR* ik_solver;

//关节限位
const double JOINT_LIMITS[6][2] = {
    {-6.28319,  6.28319},
    {-6.28319,  6.28319},
    {-6.28319,  6.28319},
    {-6.28319,  6.28319},
    {-6.28319,  6.28319},
    {-6.28319,  6.28319}
};

void initUR5Chain()
{
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, PI/2, 0.089159, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.425, 0, 0, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.39225, 0, 0, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, PI/2, 0.10915, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, -PI/2, 0.09465, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, 0, 0.0823, 0)));

    fk_solver = new KDL::ChainFkSolverPos_recursive(ur5_chain);
    ik_vel_solver = new KDL::ChainIkSolverVel_pinv(ur5_chain);
    ik_solver = new KDL::ChainIkSolverPos_NR(ur5_chain, *fk_solver, *ik_vel_solver, 5000, 1e-9);
}

void enforceJointLimits(KDL::JntArray& q_prev, KDL::JntArray& q_out)
{
    for(int i=0; i<6; i++){
        q_out(i) = std::max(JOINT_LIMITS[i][0], std::min(JOINT_LIMITS[i][1], q_out(i)));
        double max_step = 0.1;
        q_out(i) = std::max(q_prev(i)-max_step, std::min(q_prev(i)+max_step, q_out(i)));
    }
}


bool computeIKService(ur5_kinematics::IkService::Request &req, ur5_kinematics::IkService::Response &res)
{
    double x = req.target_x;
    double y = req.target_y;
    double z = req.target_z;
    double roll = req.roll;
    double pitch = req.pitch;
    double yaw = req.yaw;

    KDL::JntArray q_prev(6);
    for(int i=0; i<6; i++) q_prev(i) = req.joint_prev[i];

    KDL::Frame target_frame;
    target_frame.p = KDL::Vector(x, y, z);
    target_frame.M = KDL::Rotation::RPY(roll, pitch, yaw);

    KDL::JntArray q_out(6);
    int ik_result = ik_solver->CartToJnt(q_prev, target_frame, q_out);

    res.success = false;
    res.error_code = ik_result;

    if(ik_result == 0)
    {
        enforceJointLimits(q_prev, q_out);

        // 精度计算：正解
        KDL::Frame test_frame;
        fk_solver->JntToCart(q_out, test_frame);
        // 计算姿态真实误差
        KDL::Rotation diff_R = target_frame.M.Inverse() * test_frame.M;
        double r_err, p_err, y_err;
        diff_R.GetRPY(r_err, p_err, y_err);
        double true_angle_err = fabs(r_err*RAD_TO_DEG) + fabs(p_err*RAD_TO_DEG) + fabs(y_err*RAD_TO_DEG);

        // 输出结果
        res.success = true;
        res.joint_positions.clear();
        for(int i=0; i<6; i++) res.joint_positions.push_back(q_out(i));
        res.max_angle_error_deg = true_angle_err;

        ROS_INFO("逆解成功 | 真实姿态误差: %.9f°", true_angle_err);
    }
    else
    {
        ROS_WARN("逆解失败，错误码:%d", ik_result);
    }
    return true;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "ur5_ik_node");
    ros::NodeHandle nh;
    initUR5Chain();

    // 注册服务
    ros::ServiceServer ik_service = nh.advertiseService("/ur5_compute_ik", computeIKService);
    ROS_INFO("UR5 逆解服务节点启动（支持中文日志）");
    ros::spin();

    delete fk_solver;
    delete ik_vel_solver;
    delete ik_solver;
    return 0;
}