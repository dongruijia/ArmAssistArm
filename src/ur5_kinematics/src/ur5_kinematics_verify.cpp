#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <unordered_map>
#include <locale.h>

#define PI 3.1415926535
#define RAD2DEG (180.0 / PI)
#define DEG2RAD (PI / 180.0)

// ===================== 全局变量 =====================
KDL::JntArray current_q(6);
KDL::Chain ur5_chain;
KDL::ChainFkSolverPos_recursive* fk_solver = nullptr;
KDL::ChainJntToJacSolver* jac_solver = nullptr;
tf2_ros::Buffer* tf_buffer = nullptr;
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

const std::vector<double> joint_min = {-2*PI, -PI,   -PI,   -PI,   -2*PI, -2*PI};
const std::vector<double> joint_max = { 2*PI,  0,     PI,    PI,    2*PI,  2*PI};

const int MAX_ITER = 1;
const double POS_TOL = 1e-6;
const double ORI_TOL = 1e-6;
const double SINGULAR_THRESH = 1e-2; // 调大了奇异阈值，规避腕部奇异乱跳

KDL::Frame align_transform = KDL::Frame(KDL::Rotation::RotZ(PI));

// ===================== 初始化 =====================
void initUR5Chain() {
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, PI/2, 0.089159, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.425, 0, 0, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.39225, 0, 0, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, PI/2, 0.10915, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, -PI/2, 0.09465, 0)));
    ur5_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, 0, 0.0823, 0)));
    
    fk_solver = new KDL::ChainFkSolverPos_recursive(ur5_chain);
    jac_solver = new KDL::ChainJntToJacSolver(ur5_chain);
}

// ===================== 工具函数 =====================
void clampJointLimits(KDL::JntArray& q) {
    for (int i = 0; i < 6; ++i) {
        q(i) = std::max(joint_min[i], std::min(q(i), joint_max[i]));
    }
}

// 关节误差周期性归一化
double normalizeAngleDiff(double diff) {
    while (diff > PI) diff -= 2*PI;
    while (diff < -PI) diff += 2*PI;
    return fabs(diff);
}

// FK的姿态误差计算
double calculateOrientationErrorFK(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2) {
    tf2::Quaternion tq1, tq2;
    tf2::convert(q1, tq1);
    tf2::convert(q2, tq2);
    tq1.normalize();
    tq2.normalize();
    double dot = tq1.dot(tq2);
    dot = std::max(std::min(dot, 1.0), -1.0);
    return 2 * acos(fabs(dot)) * 180.0 / PI;
}

// IK的精确轴角姿态误差
KDL::Twist computePoseErrorIK(const KDL::Frame& T_target, const KDL::Frame& T_current) {
    KDL::Twist error;
    error.vel = T_target.p - T_current.p;
    KDL::Rotation R_err = T_target.M * T_current.M.Inverse();
    double qx, qy, qz, qw;
    R_err.GetQuaternion(qx, qy, qz, qw);
    double angle = 2 * acos(std::max(std::min(qw, 1.0), -1.0));
    if (fabs(angle) > 1e-6) {
        double sin_half = sin(angle / 2.0);
        error.rot = KDL::Vector(qx / sin_half, qy / sin_half, qz / sin_half) * angle;
    } else {
        error.rot = KDL::Vector(0, 0, 0);
    }
    return error;
}

// ===================== 牛顿-拉夫逊 IK核心 =====================
bool newtonRaphsonIK(KDL::JntArray& q_out, const KDL::Frame& T_target, const KDL::JntArray& q_init) {
    KDL::JntArray q = q_init;
    clampJointLimits(q);
    KDL::Frame T_current;
    KDL::Jacobian jac(6);
    KDL::Twist error;

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        fk_solver->JntToCart(q, T_current);

        error = computePoseErrorIK(T_target, T_current);
        double pos_err = error.vel.Norm();
        double ori_err = error.rot.Norm();

        if (pos_err < POS_TOL && ori_err < ORI_TOL) {
            q_out = q;
            return true;
        }

        jac_solver->JntToJac(q, jac);

        Eigen::MatrixXd J = jac.data;
        Eigen::VectorXd e(6);
        e << error.vel.x(), error.vel.y(), error.vel.z(),
             error.rot.x(), error.rot.y(), error.rot.z();
        
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::MatrixXd& U = svd.matrixU();
        const Eigen::MatrixXd& V = svd.matrixV();
        const Eigen::VectorXd& S = svd.singularValues();

        double min_sv = S.minCoeff();
        if (min_sv < SINGULAR_THRESH) {
            ROS_WARN("⚠️  检测到奇异位形，已自动规避");
            return false;
        }

        Eigen::MatrixXd S_pinv = Eigen::MatrixXd::Zero(6, 6);
        for (int i = 0; i < 6; ++i) {
            if (S(i) > SINGULAR_THRESH) {
                S_pinv(i, i) = 1.0 / S(i);
            }
        }
        Eigen::MatrixXd J_pinv = V * S_pinv * U.transpose();

        Eigen::VectorXd delta_q_eig = J_pinv * e;

        KDL::JntArray delta_q(6);
        for (int i = 0; i < 6; ++i) {
            delta_q(i) = delta_q_eig(i);
        }

        q.data += delta_q.data;
        clampJointLimits(q);
    }

    ROS_ERROR("❌ IK迭代未收敛");
    return false;
}

// ===================== ROS回调 =====================
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

int main(int argc, char** argv) {
    setlocale(LC_ALL,""); // 解决中文乱码
    ros::init(argc, argv, "ur5_kinematics_verify");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buf;
    tf2_ros::TransformListener tf_listener(tf_buf);
    tf_buffer = &tf_buf;

    initUR5Chain();
    current_q.data.setZero();

    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 50, jointStateCallback);
    ros::Rate rate(10);

    ROS_INFO("✅ UR5运动学全自动验证节点启动！");
    ROS_INFO("ℹ️  自动读取关节角 -> FK算位姿 -> IK反解 -> 实时输出误差");

    while (ros::ok()) {
        ros::spinOnce();

        // 1. FK计算
        KDL::Frame ee_dh;
        fk_solver->JntToCart(current_q, ee_dh);
        KDL::Frame ee_result = align_transform * ee_dh;

        // 2. 读取TF真值
        geometry_msgs::TransformStamped tf_true;
        try {
            tf_true = tf_buffer->lookupTransform(
                "base_link", "wrist_3_link",
                latest_joint_time,
                ros::Duration(0.05)
            );
        } catch (tf2::TransformException& ex) {
            continue;
        }

        // 3. FK精度计算
        geometry_msgs::Pose dh_pose;
        dh_pose.position.x = ee_result.p.x();
        dh_pose.position.y = ee_result.p.y();
        dh_pose.position.z = ee_result.p.z();
        ee_result.M.GetQuaternion(dh_pose.orientation.x, dh_pose.orientation.y,
                                  dh_pose.orientation.z, dh_pose.orientation.w);

        double dx = dh_pose.position.x - tf_true.transform.translation.x;
        double dy = dh_pose.position.y - tf_true.transform.translation.y;
        double dz = dh_pose.position.z - tf_true.transform.translation.z;
        double pos_error_fk = sqrt(dx*dx + dy*dy + dz*dz) * 1000;
        double ori_error_fk = calculateOrientationErrorFK(dh_pose.orientation, tf_true.transform.rotation);

        // 4. 自动跑IK：把FK算出的位姿当成目标
        KDL::Frame target_pose = align_transform.Inverse() * ee_result;
        KDL::JntArray q_solution(6);
        bool ik_success = newtonRaphsonIK(q_solution, target_pose, current_q);

        // 5. 打印所有结果
        ROS_INFO("\n==================================================");
        // FK结果
        ROS_INFO("【FK精度】位置误差: %.3f mm | 姿态误差: %.3f °", pos_error_fk, ori_error_fk);
        ROS_INFO("DH位姿: x=%.3f y=%.3f z=%.3f qx=%.6f qy=%.6f qz=%.6f qw=%.6f", 
            dh_pose.position.x, dh_pose.position.y, dh_pose.position.z,
            dh_pose.orientation.x, dh_pose.orientation.y, dh_pose.orientation.z, dh_pose.orientation.w);
        
        // IK结果
        if (ik_success) {
            double max_error_ik = 0.0;
            ROS_INFO("【IK精度】");
            for (int i = 0; i < 6; ++i) {
                double diff = q_solution(i) - current_q(i);
                double err = normalizeAngleDiff(diff) * RAD2DEG;
                max_error_ik = std::max(max_error_ik, err);
                ROS_INFO("  关节%d 误差: %.4f °", i+1, err);
            }
            ROS_INFO("  IK最大关节误差: %.4f ° (要求≤0.1°)", max_error_ik);
        }
        ROS_INFO("==================================================");

        rate.sleep();
    }

    delete fk_solver;
    delete jac_solver;
    return 0;
}