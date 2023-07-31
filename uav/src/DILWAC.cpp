#include "DILWAC.h"
#include "ANN.h"
#include "CNN.h"
#include "NMethods.h"
#include <Eigen/Dense>

DILWAC::DILWAC(const uint16_t DoF, const uint16_t criticDoF): actor(DoF), critic(DoF, criticDoF)
{
    r = new Eigen::VectorXf(DoF);
}

DILWAC::~DILWAC()
{
    delete goal;
    delete penality;
}

void DILWAC::setANN(const Eigen::MatrixXf& Lambda_)
{
    actor.setLearningParameters(Lambda_);
}

void DILWAC::setCNN(const int gamma_, const int penality_, const Eigen::MatrixXf& GammaC_, float goal_, float alpha_l, float lamb_l)
{
    *goal = goal_;
    *penality = penality_;
    critic.setLearningParameters(gamma_, penality_, GammaC_);
    critic.setLevant(alpha_l, lamb_l);
}

Eigen::MatrixXf DILWAC::learnDampingInjection(const Eigen::VectorXf& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep,const Eigen::VectorXf& sq,const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, float delta_t)
{
    Eigen::VectorXf r = rewardPolicy(sq);

    critic.getInputs(qe, qd, q, qep, qp, qdp, r);
    Eigen::VectorXf Jp = critic.learnFromInteraction(qe, qd, q, qep, qp, qdp, r, delta_t);
    
    actor.getInputs(we, qe, qep);
    return actor.learnDamping(we, qe, qep, sq, r, Jp, delta_t);
}

Eigen::VectorXf DILWAC::rewardPolicy(const Eigen::VectorXf& sq)
{
    Eigen::VectorXf r_current = 0*sq;

    for (int i = 0; i < sq.size(); ++i) 
    {
        if (sqrt(sq(i)*sq(i)) > *goal) 
        {
            r_current(i) = -(*penality);
        } 
        else 
        {
            r_current(i) = 0;
        }
    }
    *r = r_current;
    return r_current;
}