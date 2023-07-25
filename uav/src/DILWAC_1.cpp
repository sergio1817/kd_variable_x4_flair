#include <Eigen/Dense>
#include "NMethods.h"
#include <cmath>

// Constructor
DILWAC::DILWAC(float gamma_): gamma(gamma_), 
{
    levant = Levant_diff("tanh", 8, 6, 3000);
    
    this -> gamma = gamma;

}

// Destructor
DILWAC::~DILWAC(void) {}

Eigen::Matrix3f DILWAC::compute_damping()
{
    wc_update(sadlksandlkasnda);
    CNN(nksadjnaskdjnasjkdsa);



    return kd_matrix;
}


// --> Critic functions
// Wc update
void DILWAC::wc_update(float gamma, const Eigen::MatrixXf& Gamma_c, const Eigen::VectorXf& ec,
                          const Eigen::VectorXf& r, const Eigen::MatrixXf& xc, const Eigen::VectorXf& wc,
                          const Eigen::MatrixXf& xcp, Eigen::VectorXf& wcp) {
    Eigen::VectorXf n = xc * ec + Gamma_c * wc;
    float norm_n = n.transpose() * n;
    //Eigen::VectorXf dWc(4);
    //dWc.setZero();

    if (norm_n != 0) {
        wcp = -(n / norm_n) * (wc.transpose() * xcp * ec - (1 / gamma) * wc.transpose() * xc * ec + r.transpose() * ec);
    }
}

// compute C-NN output
void DILWAC::CNN(const Eigen::VectorXf& wc, const Eigen::MatrixXf& xc, Eigen::VectorXf& J) 
    {
        J = xc.transpose() * wc;
    }

// reward computation
Eigen::VectorXf DILWAC::reward(const Eigen::VectorXf& Sq, float P, float eps_a, Eigen::VectorXf& r)
    {
        int n = Sq.size();
        Eigen::VectorXf r(n);
        r.setZero();

        for (int i = 0; i < n; ++i) {
            if (Sq(i).norm() > eps_a) {
                r(i) = -P;
            } else {
                r(i) = 0;
            }
        }
        return r;
    }

// virtual control
void DILWAC::virtual_control(float kc, const Eigen::VectorXf& ec, const Eigen::VectorXf& int_ec,
                     Eigen::VectorXf& u_c, Eigen::VectorXf& s_e) {
    int n = ec.size();
    float gamma_e = 9.0;
    kc = 0.1;

    s_e = ec + gamma_e * int_ec;

    // Option 1: u_c = -kc * ec
    // u_c = -kc * ec;

    // Option 2: u_c = -kc * tanh(200 * ec)
    // u_c = -kc * (ec.array().tanh() * 200.0);

    // Option 3: u_c = -gamma_e * ec - kc * tanh(100 * s_e)
    u_c = -gamma_e * ec - kc * (s_e.array().tanh() * 100.0);
}

// compute temporal difference (TD)
void DILWAC::TD(const Eigen::VectorXf& Jhat, float gamma, const Eigen::VectorXf& r, const Eigen::VectorXf& u_c, Eigen::VectorXf& ecp) {

    Eigen::VectorXf dot_J = levant.Compute(Jhat,delta_t);
    
    ecp = dot_J - Jhat / gamma + r + u_c;
}

// compute CNN input
void DILWAC::xc_builder(const Eigen::Quaternionf& qd, const Eigen::Quaternionf& qdp, const Eigen::Quaternionf& q, const Eigen::Quaternionf& qp, 
                Eigen::MatrixXf& xc, Eigen::MatrixXf& xcp) {
    int n = qd.size();

    xc.resize(n, 4);
    xc.col(0) = q.coeffs().tail(3);    // Vector part of q
    xc.col(1) = qd.coeffs().tail(3);   // Vector part of qd
    xc.col(2) = (q.coeffs().tail(3)).array().square();    // Vector part of q^2
    xc.col(3) = (q.coeffs().tail(3)).array() * (qd.coeffs().tail(3)).array();    // Vector part of q * qd

    xcp.resize(n, 4);
    xcp.col(0) = qp.coeffs().tail(3);    // Vector part of qp
    xcp.col(1) = qdp.coeffs().tail(3);   // Vector part of qdp
    xcp.col(2) = 2 * (q.coeffs().tail(3)).array() * (qp.coeffs().tail(3)).array();    // Vector part of 2 * (q * qp)
    xcp.col(3) = (q.coeffs().tail(3)).array() * (qdp.coeffs().tail(3)).array() + (qp.coeffs().tail(3)).array() * (qd.coeffs().tail(3)).array();    // Vector part of (q * qdp + qp * qd)
}

/* ----------------------------------------------------------------------------------------- */
// --> ANN functions
// compute kd
Eigen::VectorXf DILWAC::ANN(const Eigen::MatrixXf& xa, const Eigen::MatrixXf& wa) {
    Eigen::VectorXf kd = xa.transpose() * wa;
    Eigen::VectorXf kd_abs = kd.array().square();
    return kd_abs;
}

// compute learning monitor => Psi
Eigen::MatrixXf DILWAC::Psi(const Eigen::VectorXf& Sq, const Eigen::VectorXf& Jp) {
    int n = Sq.size();
    Eigen::MatrixXf aF(n, n);
    aF.setZero();

    for (int i = 0; i < n; ++i) {
        aF(i, i) = -std::tanh(Sq(i) * Jp(i));
    }

    return aF;
}

// update wa
Eigen::MatrixXf DILWAC::wa_update(const Eigen::MatrixXf& lr, const Eigen::MatrixXf& Psi) {
    Eigen::MatrixXf dWa = lr * Psi;
    return dWa;
}

// compute ANN input
Eigen::MatrixXf DILWAC::xa_builder(const Eigen::Quaternionf& q, const Eigen::Quaternionf& qd) {
    Eigen::MatrixXf xa(2, 2);

    xa(0, 0) = q.coeffs().dot(q.coeffs()) + 2;    // q'*q + 2
    xa(0, 1) = 0;
    xa(1, 0) = 0;
    xa(1, 1) = qd.coeffs().dot(qd.coeffs()) + 2;  // qd'*qd + 2

    return xa;
}