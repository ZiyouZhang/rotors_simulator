#include "visual_ekf.hpp"
#include "Transformation.hpp"
#include "operators.hpp"
#include <iostream>

#include <ros/console.h>

Eigen::Quaterniond operator*(const double a, const Eigen::Quaterniond b)
{
    Eigen::Quaterniond c;
    c.w() = a * b.w();
    c.vec() = a * b.vec();
    c.normalize();
    return c;
}

Eigen::Quaterniond operator+(const Eigen::Quaterniond q1, const Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q;
    q.w() = q1.w() + q2.w();
    q.vec() = q1.vec() + q2.vec();
    return q;
}

Eigen::Quaterniond operator*(const Eigen::Quaterniond q1, const Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q;
    q.w() = q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z();
    q.x() = q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y();
    q.y() = q1.w() * q2.y() - q1.x() * q2.z() + q1.y() * q2.w() + q1.z() * q2.x();
    q.z() = q1.w() * q2.z() + q1.x() * q2.y() - q1.y() * q2.x() + q1.z() * q2.w();
    q.normalize();
    return q;
}

VisualEKF::VisualEKF()
{
    x_.r_W.setZero();
    x_.q_WO.setIdentity();
    x_.v_O.setZero();
    x_.omega_O.setZero();
    P_.setZero();

    last_update_time_ = ros::Time::now();
}

VisualEKF::~VisualEKF()
{
}

bool VisualEKF::initialise(ros::Time rosTimeStamp,
                           const ObjectState &x0,
                           const Eigen::Matrix<double, 13, 13> &P0)
{
    x_ = x0;
    P_ = P0;

    x_propagated_ = x_;

    return true;
}

ObjectStateDerivative VisualEKF::calcStateDerivative(const ObjectState &state,
                                                     const ApriltagMeasurement &measurement)
{
    ObjectStateDerivative stateDerivative;
    stateDerivative.r_W_dot = measurement.q_WO.toRotationMatrix() * state.v_O;
    // stateDerivative.q_dot = 0.5 * oplus(Eigen::Quaterniond(0.0, omega_S[0], omega_S[1], omega_S[2])) * state.q_WS.coeffs();
    Eigen::Quaterniond temp;
    temp.w() = 0.0;
    temp.vec() = state.omega_O;
    stateDerivative.q_WO_dot = 0.5 * measurement.q_WO * temp;

    stateDerivative.v_O_dot = state.q_WO.toRotationMatrix().inverse() * Eigen::Vector3d(0.0, 0.0, -9.81) - state.omega_O.cross(state.v_O);
    stateDerivative.omega_O_dot = state.inertia.inverse() * state.omega_O.cross(state.inertia * state.omega_O);
    return stateDerivative;
}

bool VisualEKF::calcJacobian(const ObjectState &state,
                             const ApriltagMeasurement &measurement,
                             Eigen::Matrix<double, 13, 13> &jacobian)
{
    // Eigen::Matrix3d C_WS = state.q_WO.toRotationMatrix();
    // jacobian.setZero();
    // jacobian.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    // jacobian.block<3, 3>(3, 9) = -C_WS;
    // jacobian.block<3, 3>(6, 3) = -crossMx(
    //     C_WS * (z.acc_S - state.b_a));
    // jacobian.block<3, 3>(6, 12) = -C_WS;
    return true;
}

bool VisualEKF::stateTransition(const ObjectState &object_state_k_minus_1,
                                ObjectState &object_state_k,
                                const ApriltagMeasurement &measurement_k_minus_1,
                                const ApriltagMeasurement &measurement_k,
                                Eigen::Matrix<double, 13, 13> &jacobian)
{
    ObjectStateDerivative objectStateDerivative_k_minus_1 = calcStateDerivative(object_state_k_minus_1, measurement_k_minus_1);

    const double dt = double(measurement_k.timestamp - measurement_k_minus_1.timestamp);

    ObjectState delta_x_1;
    delta_x_1.r_W = dt * objectStateDerivative_k_minus_1.r_W_dot;
    delta_x_1.q_WO = dt * objectStateDerivative_k_minus_1.q_WO_dot;
    delta_x_1.v_O = dt * objectStateDerivative_k_minus_1.v_O_dot;
    delta_x_1.omega_O = dt * objectStateDerivative_k_minus_1.omega_O_dot;

    ObjectState state_k_tmp;
    state_k_tmp.r_W = object_state_k_minus_1.r_W + delta_x_1.r_W;
    state_k_tmp.q_WO = (object_state_k_minus_1.q_WO + delta_x_1.q_WO).normalized();
    state_k_tmp.v_O = object_state_k_minus_1.v_O + delta_x_1.v_O;
    state_k_tmp.omega_O = object_state_k_minus_1.omega_O + delta_x_1.omega_O;

    ObjectStateDerivative objectStateDerivative_k = calcStateDerivative(state_k_tmp, measurement_k);

    ObjectState delta_x_2;
    delta_x_2.r_W = dt * objectStateDerivative_k.r_W_dot;
    delta_x_2.q_WO = dt * objectStateDerivative_k.q_WO_dot;
    delta_x_2.v_O = dt * objectStateDerivative_k.v_O_dot;
    delta_x_2.omega_O = dt * objectStateDerivative_k.omega_O_dot;

    object_state_k.r_W = object_state_k_minus_1.r_W + 0.5 * (delta_x_1.r_W + delta_x_2.r_W);
    object_state_k.q_WO = (object_state_k_minus_1.q_WO + 0.5 * (delta_x_1.q_WO + delta_x_2.q_WO)).normalized();
    object_state_k.v_O = object_state_k_minus_1.v_O + 0.5 * (delta_x_1.v_O + delta_x_2.v_O);
    object_state_k.omega_O = object_state_k_minus_1.omega_O + 0.5 * (delta_x_1.omega_O + delta_x_2.omega_O);

    return true;
}

bool VisualEKF::predict()
{
    Eigen::Matrix<double, 13, 13> F;   // Jacobian
    Eigen::Matrix<double, 13, 13> LQL; // Linearised error

    stateTransition(x_, x_predicted_, lastApriltagMeasurement, currentApriltagMeasurement, F);

    // LQL.setZero();
    // LQL.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * delta_t_ * sigma_c_r_W * sigma_c_r_W;
    // LQL.block<4, 4>(6, 6) = Eigen::Matrix3d::Identity() * delta_t_ * sigma_c_q_WO * sigma_c_q_WO;
    // LQL.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity() * delta_t_ * sigma_c_v_O * sigma_c_v_O;
    // LQL.block<3, 3>(13, 13) = Eigen::Matrix3d::Identity() * delta_t_ * sigma_c_omega_O * sigma_c_omega_O;

    // P_ = F * P_ * F.transpose() + LQL;

    return true;
}

bool VisualEKF::update()
{
    return true;
}