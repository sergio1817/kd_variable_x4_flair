#include "DILWAC.h"
#include "ANN.h"
#include "CNN.h"
#include "NMethods.h"
#include <Eigen/Dense>
#include <type_traits>

DILWAC::DILWAC(const uint16_t DoF, const uint16_t criticDoF): actor(DoF), critic(DoF, criticDoF)
{
    r = Eigen::VectorXf(DoF);
}

DILWAC::~DILWAC()
{
    //delete goal;
    //delete penalty;
}

void DILWAC::setANN(const Eigen::MatrixXf& Lambda_)
{
    actor.setLearningParameters(Lambda_);
}

void DILWAC::setCNN(const int gamma_, const int penalty_, const Eigen::MatrixXf& GammaC_, float goal_, float alpha_l, float lamb_l)
{
    goal = goal_;
    penalty = penalty_;
    critic.setLearningParameters(gamma_, penalty_, GammaC_);
    critic.setLevant(alpha_l, lamb_l);
    critic.setVirtualControlParameters(10, 0.01);
}

//template<typename Derived>
Eigen::Matrix3f DILWAC::learnDampingInjection(const Eigen::VectorXf& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep,const Eigen::Vector3f& sq,const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, float delta_t)
{
    Eigen::Vector3f r = rewardPolicy(sq);

    //critic.getInputs(qe, qd, q, qep, qp, qdp, r);
    //Eigen::VectorXf Jp = critic.learnFromInteraction(qe, qd, q, qep, qp, qdp, r, delta_t);
    
    //actor.getInputs(we, qe, qep);
    //return actor.learnDamping(we, qe, qep, sq, r, Jp, delta_t);
}

Eigen::Vector3f DILWAC::rewardPolicy(const Eigen::Vector3f& sq)
{
    Eigen::Vector3f r_current = 0*sq;

    for (int i = 0; i < sq.size(); ++i) 
    {
        if (sqrt(sq(i)*sq(i)) > goal) 
        {
            r_current(i) = -(penalty);
        } 
        else 
        {
            r_current(i) = 0;
        }
    }
    r = r_current;
    return r_current;
}