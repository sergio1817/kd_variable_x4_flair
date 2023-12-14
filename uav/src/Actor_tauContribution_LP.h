#ifndef ACTOR_TAUCONTRIBUTION_LP //CAMBIAR
#define ACTOR_TAUCONTRIBUTION_LP //CAMBIAR

#include <Eigen/Dense>
#include "ANN_LP.h"
#include "CNN_LP.h"
#include <type_traits>



class Actor_tauContribution_LP
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Actor_tauContribution_LP();
        ~Actor_tauContribution_LP();

        void reset(Eigen::Matrix<float,10,3> Wa_0, Eigen::Matrix<float,10,1> Wc_0);
        void setANN(const Eigen::Vector3f& nur2,Eigen::Matrix<float,4,10>& Va2, const Eigen::Matrix<float,10,10>&GAMMA_a2, float delta_t);
        void setCNN(Eigen::Quaternionf& qe_,Eigen::Quaternionf qep_, float Q_, float P_,const Eigen::Matrix<float,4,10>& Vc2,const float psi_,const float K_,const float Kw_);
        void ActorCritic_Compute(float delta_t);
        Eigen::Vector3f  YrOutput();
        //template<typename Derived>
        // Eigen::Matrix3f learnDampingInjection(const Eigen::Vector3f& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep, const Eigen::Vector3f& sq,const Eigen::Quaternionf& qd, const Eigen::Quaternionf& q, const Eigen::Quaternionf& qp, const Eigen::Quaternionf& qdp, float delta_t);

        // Eigen::Vector3f getR() const { return r; }
        // Eigen::Vector3f getEc() const { return critic->getEc(); }
        // Eigen::Vector3f getJ() const { return critic->getJ(); }
        // Eigen::Matrix3f getK() const { return actor->getK(); }       
        // Eigen::Matrix3f getPsi() const { return actor->getPsi(); }


    private:

        ANN_LP *actor;
        CNN_LP *critic;

};

#endif // ACTOR_TAUCONTRIBUTION_LP