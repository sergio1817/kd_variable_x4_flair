#ifndef DILWAC_H
#define DILWAC_H

#include <Eigen/Dense>
#include "ANN.h"
#include "CNN.h"
#include <type_traits>



class DILWAC
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DILWAC(const int& DoF, const int& criticDoF);
        ~DILWAC();

        void setANN(const Eigen::Matrix3f& Lambda_);
        void setCNN(const int gamma_, const int penalty_, const Eigen::Matrix4f& GammaC_, float goal, float alpha_l, float lamb_l);

        //template<typename Derived>
        Eigen::Matrix3f learnDampingInjection(const Eigen::Vector3f& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep, const Eigen::Vector3f& sq,const Eigen::Quaternionf& qd, const Eigen::Quaternionf& q, const Eigen::Quaternionf& qp, const Eigen::Quaternionf& qdp, float delta_t);

        Eigen::Vector3f getR() const { return r; }
        Eigen::Vector3f getEc() const { return critic->getEc(); }
        Eigen::Vector3f getJ() const { return critic->getJ(); }
        Eigen::Matrix3f getK() const { return actor->getK(); }       
        Eigen::Matrix3f getPsi() const { return actor->getPsi(); }

    private:
        float goal;
        int penalty;
        Eigen::Vector3f r;

        ANN *actor;
        CNN *critic;

        Eigen::Vector3f rewardPolicy(const Eigen::Vector3f& sq);
};

#endif // DILWAC_H