#ifndef VISUAL_EKF_HPP_
#define VISUAL_EKF_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>

struct ObjectState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double mass;
    double timestamp;                  // Time stamp in seconds.
    Eigen::Vector3d r_W;                 // The position relative to the W frame.
    Eigen::Quaterniond q_WO;             // The quaternion of rotation W-O.
    Eigen::Vector3d v_O;                 // The velocity expressed in object frame.
    Eigen::Vector3d omega_O;             // The angular velocity expressed in object frame.
    Eigen::Matrix<double, 3, 3> inertia; //The moment of inertia.
};

struct ObjectStateDerivative
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;          // Time stamp in seconds.
    Eigen::Vector3d r_W_dot;     // The position relative to the W frame.
    Eigen::Quaterniond q_WO_dot; // The quaternion of rotation W-O.
    Eigen::Vector3d v_O_dot;     // The velocity expressed in object frame.
    Eigen::Vector3d omega_O_dot; // The angular velocity expressed in object frame.
};

struct ApriltagMeasurement
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    double timestamp;      // Time stamp in seconds.
    Eigen::Vector3d r_W;     // The position relative to the W frame.
    Eigen::Quaterniond q_WO; // The quaternion of rotation W-O.
};

class VisualEKF
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VisualEKF();
    ~VisualEKF();

    bool isInitialsed();

    bool initialise(const ObjectState initialState);

    bool predict(const double dt);

    bool update(const ApriltagMeasurement apriltagMeasurement);

    bool stateTransition(const ObjectState &object_state_k_minues_1,
                         ObjectState &object_state_1,
                         const double dt,
                         Eigen::Matrix<double, 13, 13> &jacobian);

    ObjectStateDerivative calcStateDerivative(const ObjectState &state);

    bool calcJacobian(const ObjectState &state,
                      const double &dt,
                      Eigen::Matrix<double, 13, 13> &jacobian);

private:
    ObjectState x_;
    ObjectState x_predicted_;
    ObjectState x_propagated_;

    bool initialised = false;

    ros::Time last_update_time_;
    uint64_t delta_t_;

    Eigen::Matrix<double, 13, 13> P_;
    Eigen::Matrix<double, 13, 13> jacobian;

    // noise params
    double sigma_c_r_W = 1.0e-2;     // 0.01 meter for tag detection error
    double sigma_c_q_WO = 1.0e-3;    // 0.001 for quoternion error
    double sigma_c_v_O = 5.0e-2;     // 0.05 velocity error
    double sigma_c_omega_O = 5.0e-4; // 5e-4, amgular velocity error
    double sigma_z_r_W = 0.05;       // 0.05 meter for pose measurement error
    double sigma_z_q_WO = 0.01;      // 0.001 for quaternion measurement error

    friend class PoseDetector;
};

#endif // VISUAL_EKF_HPP_