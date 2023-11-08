#include "CNN.h"
#include "NMethods.h"

CNN::CNN(const int& DoF, const int& criticDoF)
{   
    printf("CNN constructor in\n");
    wc    = Eigen::Vector4f::Zero();
    xc    = Eigen::Matrix<float, 4, 3>::Zero();
    xcp   = Eigen::Matrix<float, 4, 3>::Zero();
    J     = Eigen::Vector3f::Zero();
    ec    = Eigen::Vector3f::Zero();
    int_r = Eigen::Vector3f::Zero();
    intS_ec = Eigen::Vector3f::Zero();

    levant = Levant_diff("tanh", 8, 6, 3000);

    printf("CNN constructor out\n");
}

CNN::~CNN() { }

void CNN::setLearningParameters(const int gamma_, const int penality_, const Eigen::Matrix4f& GammaC_)
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

Eigen::Vector4f CNN::updateWeights(const Eigen::Vector3f& r) 
{
    Eigen::Vector4f wcp = 0*(wc);

    Eigen::Vector4f omega = (xc) * (ec) + (GammaC) * (wc);
    float norm_omega = omega.transpose() * omega;

    if (norm_omega != 0) 
    {
        wcp = -(omega / norm_omega) * ((wc).transpose() * (xcp) * (ec) - (1 / (gamma)) * (wc).transpose() * (xc) * (ec) + r.transpose() * (ec));
    }
    return wcp;
}

Eigen::Vector3f CNN::computeBellmanError(const Eigen::Vector3f& r, const Eigen::Vector3f& uc, const Eigen::Vector3f& Jp)
{
    return Jp - J / (gamma) + r + uc;
}

Eigen::Vector3f CNN::computeBellmanControl(const Eigen::Vector3f& int_ec) 
{
    return - (kc) * (ec) - (miuc) * int_ec;
}

Eigen::Vector3f CNN::computeISMBellmanControl(const Eigen::Vector3f& ec,const Eigen::Vector3f& sb) 
{
    return - (kc) * (sb) - (miuc) * signth(ec, 100);
}

Eigen::Vector3f CNN::computeBellmanSurface(const Eigen::Vector3f& ec, const Eigen::Vector3f& intS_ec) 
{
    return ec + (miuc) * intS_ec;
}

void CNN::getInputs(const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qd, const Eigen::Quaternionf& q, const Eigen::Quaternionf& qep, const Eigen::Quaternionf& qp, const Eigen::Quaternionf& qdp, const Eigen::Vector3f& r)
{
    (xc).row(0) = qe.vec().transpose();
    (xc).row(1) = q.vec().transpose();
    (xc).row(2) = qd.vec().transpose();
    (xc).row(3) = int_r.transpose();

    (xcp).row(0) = qep.vec().transpose();
    (xcp).row(1) = qp.vec().transpose();
    (xcp).row(2) = qdp.vec().transpose();
    (xcp).row(3) = r.transpose();
}

Eigen::Vector3f CNN::learnFromInteraction(const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qd, const Eigen::Quaternionf& q, const Eigen::Quaternionf& qep, const Eigen::Quaternionf& qp, const Eigen::Quaternionf& qdp, const Eigen::Vector3f& r, float delta_t)
{
    getInputs(qe, qd, q, qep, qp, qdp, r);

    int_ec = rk4_vec(int_ec, ec, delta_t);

    int_r = rk4_vec(int_r, r, delta_t);

    intS_ec = rk4_vec(intS_ec, signth(ec, 100), delta_t);

    Eigen::Vector3f sb  = computeBellmanSurface(ec, intS_ec);

    //Eigen::Vector3f uc  = computeBellmanControl(int_ec);
    
    Eigen::Vector3f uc  = computeISMBellmanControl(ec, sb);

    Eigen::Vector3f Jp  = levant.Compute(J,delta_t);

    Eigen::Vector3f ecp = computeBellmanError(r, uc, Jp);

    ec = rk4_vec(ec, ecp, delta_t);

    Eigen::Vector4f wcp  = updateWeights(r);

    wc = rk4_mat(wc, wcp, delta_t);

    approximateValueFunction();

    return Jp;
}
