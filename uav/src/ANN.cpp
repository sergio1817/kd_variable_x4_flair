#include "ANN.h"
#include "NMethods.h"

ANN::ANN(const int& DoF)
{   
    printf("ANN constructor in\n");
    Lambda = Eigen::Matrix3f::Zero();
    xa     = Eigen::Matrix3f::Zero();
    K      = Eigen::Matrix3f::Zero();
    wa     = Eigen::Matrix3f::Zero();
    Psi    = Eigen::Matrix3f::Zero();

    printf("ANN constructor out\n");
}

ANN::~ANN() { }

void ANN::setLearningParameters(const Eigen::Matrix3f& Lambda_)
{
    Lambda = Lambda_;
}

void ANN::getInputs(const Eigen::Vector3f& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep)
{
    (xa)(0, 0) = qe.vec().norm();
    (xa)(1, 1) = qep.vec().norm();
    (xa)(2, 2) = we.norm();

    xa = xa + Eigen::Matrix3f::Identity(3, 3);
}

void ANN::computeFeedbackGain()
{
    K = xa.transpose() * (wa);
}

Eigen::Matrix3f ANN::updateWeights(const Eigen::Matrix3f& Psi, const Eigen::Matrix3f& Lambda)
{
    Eigen::Matrix3f wap = Lambda * Psi;
    return wap;
}

void ANN::learningMonitor(const Eigen::Vector3f& sq, const Eigen::Vector3f& r, const Eigen::Vector3f& Jp)
{
    for (int i = 0; i < sq.size(); i++)
    {
        (Psi)(i, i) = -signth(-r(i) * sq(i), Jp(i));
    }
}

Eigen::Matrix3f ANN::learnDamping(const Eigen::Vector3f& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep,const Eigen::Vector3f& sq, const Eigen::Vector3f& r, const Eigen::Vector3f& Jp, float delta_t)
{
    getInputs(we, qe, qep);

    learningMonitor(sq, r, Jp);

    Eigen::Matrix3f wap = updateWeights(Psi, Lambda);

    wa = rk4_mat(wa, wap, delta_t);

    computeFeedbackGain();

    K = ((K).transpose())*(K);
    return K;
}