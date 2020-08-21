#include <Eigen/Dense>

class kf
{

public:
    kf(double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P);

    /**
* Create a blank estimator.
*/
    kf();

    /**
    * Initialize the filter with initial states as zero.
    */
    void init();

    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(double t0, const Eigen::VectorXd& x0);

    /**
    * Update the estimated state based on measured values. The
    * time step is assumed to remain constant.
    */
    void update(const Eigen::VectorXd& y);

    /**
    * Update the estimated state based on measured values,
    * using the given time step and dynamics matrix.
    */
    void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

    /**
    * Return the current state and time.
    */
    Eigen::VectorXd state() {
        return x_hat;
    };
    double time() {
        return t;
    };

    ~kf();

private:
    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, P, K, P0;

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;

};

