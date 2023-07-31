#include "ANN.h"
#include "NMethods.h"

ANN::ANN(const uint16_t DoF)
{
    Lambda = new Eigen::MatrixXf(DoF, DoF);
    xa     = new Eigen::MatrixXf(DoF, DoF);
    K      = new Eigen::MatrixXf(DoF, DoF);
    wa     = new Eigen::MatrixXf(DoF, DoF);
    Psi    = new Eigen::MatrixXf(DoF, DoF);
}

ANN::~ANN()
{
    delete Lambda;
    delete xa;
    delete K;
    delete wa;
    delete Psi;
}

void ANN::setLearningParameters(const Eigen::MatrixXf& Lambda_)
{
    *Lambda = Lambda_;
}

void ANN::getInputs(const Eigen::VectorXf& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep)
{
    (*xa)(0, 0) = qe.vec().norm();
    (*xa)(1, 1) = qep.vec().norm();
    (*xa)(2, 2) = we.norm();

    *xa = *xa + Eigen::MatrixXf::Identity(3, 3);
}

void ANN::computeFeedbackGain()
{
    *K = xa->transpose() * (*wa);
}

Eigen::MatrixXf ANN::updateWeights(const Eigen::MatrixXf& Psi, const Eigen::MatrixXf& Lambda)
{
    Eigen::MatrixXf wap = Lambda * Psi;
    return wap;
}

void ANN::learningMonitor(const Eigen::VectorXf& sq, const Eigen::VectorXf& r, const Eigen::MatrixXf& Jp)
{
    for (int i = 0; i < sq.size(); i++)
    {
        (*Psi)(i, i) = -signth(100 * sq(i), Jp(i));
    }
}

Eigen::MatrixXf ANN::learnDamping(const Eigen::VectorXf& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep,const Eigen::VectorXf& sq, const Eigen::VectorXf& r, const Eigen::MatrixXf& Jp, float delta_t)
{
    getInputs(we, qe, qep);

    learningMonitor(sq, r, Jp);

    Eigen::MatrixXf wap = updateWeights(*Psi, *Lambda);

    *wa = rk4_vec(*wa, wap, delta_t);

    computeFeedbackGain();

    *K = ((*K).transpose())*(*K);
    return *K;
}