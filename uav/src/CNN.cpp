#include "CNN.h"
#include "NMethods.h"

CNN::CNN(const uint16_t DoF, const uint16_t criticDoF)
{
    wc    = Eigen::VectorXf(criticDoF);
    xc    = Eigen::MatrixXf(DoF, DoF);
    xcp   = Eigen::MatrixXf(DoF, DoF);
    J     = Eigen::VectorXf(DoF);
    ec    = Eigen::VectorXf(DoF);
    int_r = Eigen::VectorXf(DoF);

    levant = Levant_diff("tanh", 8, 6, 3000);
}

CNN::~CNN()
{

}

void CNN::setLearningParameters(const int gamma_, const int penality_, const Eigen::MatrixXf& GammaC_)
{
    gamma = gamma_;
    penality = penality_;
    GammaC = GammaC_;
}

void CNN::setVirtualControlParameters(float kc_, float miuc_)
{
    kc = kc_;
    miuc = miuc_;
}

void CNN::setLevant(float alpha_l, float lamb_l)
{
    levant.setParam(alpha_l, lamb_l);
}

void CNN::approximateValueFunction()
{
    J = xc.transpose() * (wc);
}

Eigen::VectorXf CNN::updateWeights(const Eigen::VectorXf& r) 
{
    Eigen::VectorXf wcp = 0*(wc);

    Eigen::VectorXf omega = (xc) * (ec) + (GammaC) * (wc);
    float norm_omega = omega.transpose() * omega;

    if (norm_omega != 0) 
    {
        wcp = -(omega / norm_omega) * ((wc).transpose() * (xcp) * (ec) - (1 / (gamma)) * (wc).transpose() * (xc) * (ec) + r.transpose() * (ec));
    }
    return wcp;
}

Eigen::VectorXf CNN::computeBellmanError(const Eigen::VectorXf& r, const Eigen::VectorXf& uc, const Eigen::VectorXf Jp)
{
    return Jp - J / (gamma) + r + uc;
}

Eigen::VectorXf CNN::computeBellmanControl(const Eigen::VectorXf& int_ec) 
{
    return - (kc) * (ec) - (miuc) * int_ec;
}


void CNN::getInputs(const Eigen::Quaternionf qe, const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qep, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, const Eigen::VectorXf& r)
{
    (xc).col(0) = qe.vec();
    (xc).col(1) = q.vec();
    (xc).col(2) = qd.vec();
    (xc).col(3) = int_r;

    (xcp).col(0) = qep.vec();
    (xcp).col(1) = qp.vec();
    (xcp).col(2) = qdp.vec();
    (xcp).col(3) = r;
}

Eigen::VectorXf CNN::learnFromInteraction(const Eigen::Quaternionf qe, const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qep, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, const Eigen::VectorXf& r, float delta_t)
{
    getInputs(qe, qd, q, qep, qp, qdp, r);

    int_ec = rk4_vec(int_ec, ec, delta_t);

    int_r = rk4_vec(int_r, r, delta_t);

    Eigen::VectorXf uc  = computeBellmanControl(int_ec);

    Eigen::VectorXf Jp  = levant.Compute(J,delta_t);

    Eigen::VectorXf ecp = computeBellmanError(r, uc, Jp);

    ec = rk4_vec(ec, ecp, delta_t);

    Eigen::VectorXf wcp  = updateWeights(r);

    approximateValueFunction();

    return Jp;
}
