#include "DILWAC.h"
#include "NMethods.h"

DILWAC::DILWAC(uint8_t DoF)
{
    Eigen::MatrixXf wa(DoF);
    Eigen::VectorXf wc(DoF);

    Eigen::MatrixXf xa(DoF);
    Eigen::MatrixXf xc(DoF,4);

        
    // Initial weights
    wa.setZero();
    wc.setZero();
    xa.setZero();
    xc.setZero();
    
    levant = Levant_diff("tanh", 8, 6, 3000);


}

DILWAC::~DILWAC(void){}

void DILWAC::setLearningParameters(const uint16_t& gamma, const uint8_t& P, const uint16_t& kc, const uint8_t& sq_eps)
{
    gamma = gamma;
    P = P;
    kc = kc;
    sq_eps = sq_eps;
}

void DILWAC::setLevant(float alpha_l, float lamb_l)
{
    levant.setParam(alpha_l, lamb_l);
}

void DILWAC::setVirtualControl(float& kc, float& gamma_e)
{
    float kc = kc;
    float gamma_e = gamma_e;
}

Eigen::MatrixXf DILWAC::learn(const Eigen::VectorXf we, const Eigen::Quaternion qe, const Eigen::Quaternion qep, const Eigen::Quaternion q, const Eigen::Quaternion qp, const Eigen::Quaternion qd, const Eigen::Quaternion qdp, float delta_t)
{
    // Update CNN
    updateNN_inputs(we, qe, qep, q, qp, qd,qdp);

    // Compute reward
    reward(Sq);

    Jp = levant.Compute(J,delta_t);

    int_ec = rk4_vec(int_ec, ec, delta_t);

    virtual_control();

    ecp = TD(Jp);
    ec = rk4_vec(ec, ecp, delta_t);

    wcp = wc_update();
    wc = rk4_vec(wc, wcp, delta_t);
    CNN();
    // Jp = levant.Compute(J,delta_t);

    // Update ANN
    Psi(Sq, Jp);

    Eigen::MatrixXf wap = wa_update();

    wa = rk4_vec(wa, wap, delta_t);

    Eigen::MatrixXf KD = ANN();
    Eigen::MatrixXf sKD = squareKD(KD)

    // sending the learned damping
    return sKD;
}

Eigen::MatrixXf DILWAC::ANN(const Eigen::MatrixXf& xa, const Eigen::MatrixXf& wa) 
{
    return xa.transpose() * wa;
}

Eigen::MatrixXf DILWAC::squareKD(const Eigen::MatrixXf ANN)
{
    return ANN.array().square();
}

Eigen::MatrixXf DILWAC::wa_update(const Eigen::MatrixXf& Lambda, const Eigen::MatrixXf& Psi) 
{
    return Lambda * Psi;
}

void DILWAC::Psi(const Eigen::VectorXf Sq, const Eigen::VectorXf Jp, Eigen::MatrixXf& Psi)
{
    for (int i = 0; i < n; ++i) {
        Psi(i, i) = -signth(Sq(i),Jp(i));
    }
}

void DILWAC::CNN(const Eigen::VectorXf& wc, const Eigen::MatrixXf& xc, Eigen::VectorXf& J) 
{
    J = xc.transpose() * wc;
}

Eigen::VectorXf DILWAC::wc_update(const Eigen::MatrixXf& GammaC, const Eigen::VectorXf& ec, const Eigen::VectorXf& r, const Eigen::MatrixXf& xc, const Eigen::VectorXf& wc,const Eigen::MatrixXf& xcp) 
{
    //int n = ec.size();
    Eigen::VectorXf wcp(4);
    wcp.setZero();

    Eigen::VectorXf omega = xc * ec + Gamma_c * wc;
    float norm_omega = omega.transpose() * omega;

    if (norm_omega != 0) 
    {
        wcp = -(omega / norm_omega) * (wc.transpose() * xcp * ec - (1 / gamma) * wc.transpose() * xc * ec + r.transpose() * ec);
    }
    return wcp;
}

void DILWAC::updateNN_inputs(const Eigen::VectorXf we, const Eigen::Quaternion qe, const Eigen::Quaternion qep, const Eigen::Quaternion q, const Eigen::Quaternion qp, const Eigen::Quaternion qd, const Eigen::Quaternion qdp, const Eigen::VectorXf& int_r, const Eigen::VectorXf& r, Eigen::MatrixXf& xc, Eigen::MatrixXf& xcp, Eigen::MatrixXf& xa)
{
    xc.col(0) = qe.vec();
    xc.col(1) = q.vec();
    xc.col(2) = qd.vec();
    xc.col(3) = int_r;

    xcp.col(0) = qep.vec();
    xcp.col(1) = qp.vec();
    xcp.col(2) = qdp.vec();
    xcp.col(3) = r;

    xa(0,0) = qe.vec().norm();
    xa(1,1) = qep.vec().norm();
    xa(2,2) = we.norm();

    xa = xa + Eigen::MatrixXd::Identity(3,3);
}

Eigen::VectorXf DILWAC::TD(const Eigen::VectorXf& J, const Eigen::VectorXf& r, const Eigen::VectorXf& uc, const Eigen::VectorXf Jp, uint16_t& gamma)
{
    return Jp - J / gamma + r + uc;
}

void DILWAC::virtual_control(const Eigen::VectorXf& ec, const Eigen::VectorXf& int_ec, Eigen::VectorXf& uc, float& kc, float& gamma_e) 
{
    Eigen::VectorXf s_e = ec + gamma_e * int_ec;

    // Option 1: u_c = -kc * ec
    // u_c = -kc * ec;

    // Option 2: u_c = -kc * tanh(200 * ec)
    // u_c = -kc * (ec.array().tanh() * 200.0);

    // Option 3: u_c = -gamma_e * ec - kc * tanh(100 * s_e)
    uc = -gamma_e * ec - kc * signth(100, s_e);
}

void DILWAC::reward(const Eigen::VectorXf Sq, Eigen::VectorXf& r, uint8_t& P, float& sq_eps)
{
    for (int i = 0; i < n; ++i) 
    {
        if (Sq(i).norm() > sq_eps) 
        {
            r(i) = -P;
        } 
        else 
        {
            r(i) = 0;
        }
    }
}