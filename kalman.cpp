#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

class KalmanFilter {

public:
    KalmanFilter(
        double t,
        const Eigen::MatrixXd& sysDyn,
        const Eigen::MatrixXd& out,
        const Eigen::MatrixXd& processNoise,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& measureNoise,
        const Eigen::MatrixXd& estimNoise)
        : sysDyn(sysDyn), out(out), processNoise(processNoise), B(B), measureNoise(measureNoise), P0(estimNoise),
        m(out.rows()), n(sysDyn.rows()), t(t), initialized(false),
        I(n, n), X(n), X2(n)
    {
        I.setIdentity();
    }
    Eigen::VectorXd state() { return X; };
    double time() { return t; };
    void init() {
        X.setZero();
        estimNoise = P0;
        t = 0;
        t = t;
        initialized = true;
    }
    void init(double t, const Eigen::VectorXd& x) {
        X = x;
        estimNoise = P0;
        this->t = t;
        t = t;
        initialized = true;
    }
    void update(const Eigen::VectorXd& y, const Eigen::VectorXd& z) {
        if (!initialized)
            throw std::runtime_error("Error");
        X2 = (sysDyn * X) + (B * y);
        estimNoise = sysDyn * estimNoise * sysDyn.transpose() + processNoise;
        K = estimNoise * out.transpose() * (out * estimNoise * out.transpose() + measureNoise).inverse();
        X2 += K * (z - out * X2);
        estimNoise = (I - K * out) * estimNoise;
        X = X2;
        t += t;
    }
    void update(const Eigen::VectorXd& y, const Eigen::VectorXd& z, double t, const Eigen::MatrixXd sysDyn) {
        this->sysDyn = sysDyn;
        this->t = t;
        update(y, z);
    }
private:
    Eigen::MatrixXd sysDyn, out, processNoise, measureNoise, B, estimNoise, K, P0;
    int m, n;
    double initt, currt;
    double time;
    bool initialized;
    Eigen::MatrixXd I;
    Eigen::VectorXd X, X2;


};


int main() {

    int n = 2;
    int m = 1;
    double t = 0.5;
    Eigen::MatrixXd sysDyn(n, n);
    Eigen::MatrixXd out(m, n);
    Eigen::MatrixXd B(n, m);
    Eigen::MatrixXd processNoise(n, n);
    Eigen::MatrixXd measureNoise(m, m);
    Eigen::MatrixXd estimNoise(n, n);
    out << 1.0, 0.0;
    sysDyn << 1.0, t, 0.0, 1.0;
    processNoise << .1, 0, 0, .1;
    measureNoise << 0.05;
    estimNoise << 0.01, 0, 0, 1;
    B << 0, t;
    std::cout << "sysDyn: \n" << sysDyn << std::endl;
    std::cout << "out: \n" << out << std::endl;
    std::cout << "B: \n" << B << std::endl;
    std::cout << "processNoise: \n" << processNoise << std::endl;
    std::cout << "measureNoise: \n" << measureNoise << std::endl;
    std::cout << "estimNoise: \n" << estimNoise << std::endl;
    KalmanFilter kf(t, sysDyn, out, processNoise, B, measureNoise, estimNoise);
    Eigen::MatrixXd x(n, m);
    x << 0, 5;
    double t = 0;
    kf.init(t, x);
    Eigen::VectorXd z(m);
    Eigen::VectorXd y(m);
    std::cout << "t = " << t << ", " << "X[0]: " << kf.state().transpose() << std::endl;
    z << 2.2;
    y << -2;
    kf.update(y, z);
    std::cout << "t = " << t << ", " << ", X[" << 1 << "] = " << kf.state().transpose() << std::endl;
    return 0;
}