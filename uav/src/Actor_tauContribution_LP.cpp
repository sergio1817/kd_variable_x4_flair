
#include "Actor_tauContribution_LP.h"
#include "ANN_LP.h"
#include "CNN_LP.h"
#include "NMethods.h"
#include <Eigen/Dense>
#include <type_traits>


Actor_tauContribution_LP::Actor_tauContribution_LP()
{
critic= new CNN_LP();
actor= new ANN_LP();

}

Actor_tauContribution_LP::~Actor_tauContribution_LP(){}

void Actor_tauContribution_LP::setANN(const Eigen::Vector3f& nur2,Eigen::Matrix<float,4,10>& Va2, const Eigen::Matrix<float,10,10>&GAMMA_a2, float delta_t)
{
actor->getInputs(nur2, Va2, GAMMA_a2, delta_t);

}

void Actor_tauContribution_LP::reset(Eigen::Matrix<float,10,3> Wa_0, Eigen::Matrix<float,10,1> Wc_0){
    critic->reset(Wc_0);
    actor->reset(Wa_0);

}

void Actor_tauContribution_LP::setCNN(Eigen::Quaternionf& qe_,Eigen::Quaternionf qep_, float Q_, float P_,const Eigen::Matrix<float,4,10>& Vc2,const float psi_,const float K_,const float Kw_)
{
    critic->getInputs(qe_, qep_, Q_, P_, Vc2, psi_, K_, Kw_);
}


void Actor_tauContribution_LP::ActorCritic_Compute(float delta_t)
{
    r = critic->compute_reward() ;
    critic->computeVect_Zc();
    gamma_hat=critic->computeIntTDerror(delta_t);
    critic -> updateWeights(delta_t) ;


    actor->computeVect_Za();
    actor->updateWeights(gamma_hat, r, delta_t);

}

Eigen::Matrix<float,10,3> Actor_tauContribution_LP::getActorWeights(){
    return actor->getWeights();
}
Eigen::Matrix<float,10,1> Actor_tauContribution_LP::getCriticWeights(){
    return critic->getWeights();
}

Eigen::Vector3f Actor_tauContribution_LP::YrOutput()
{
return actor->computeActorOutput();

}