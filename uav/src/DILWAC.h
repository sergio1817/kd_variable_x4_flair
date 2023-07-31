#ifndef DILWAC_H
#define DILWAC_H

#include <Eigen/Dense>
#include "ANN.h"
#include "CNN.h"
#include <type_traits>



class DILWAC
{
    public:
        DILWAC(const uint16_t DoF, const uint16_t criticDoF);
        ~DILWAC();

        void setANN(const Eigen::MatrixXf& Lambda_);
        void setCNN(const int gamma_, const int penalty_, const Eigen::MatrixXf& GammaC_, float goal, float alpha_l, float lamb_l);
        //void setRewardPolicy(float goal_, uint16_t penalty_);

        //template<typename Derived>
        Eigen::Matrix3f learnDampingInjection(const Eigen::VectorXf& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep, const Eigen::Vector3f& sq,const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, float delta_t);

        Eigen::VectorXf getR() const { return r; }
        Eigen::VectorXf getEc() const { return critic.getEc(); }
        Eigen::VectorXf getJ() const { return critic.getJ(); }
        Eigen::MatrixXf getK() const { return actor.getK(); }       
        Eigen::MatrixXf getPsi() const { return actor.getPsi(); }

    private:
        float goal;
        int penalty;
        Eigen::VectorXf r;
        

        ANN actor;
        CNN critic;

        Eigen::Vector3f rewardPolicy(const Eigen::Vector3f& sq);
};

#endif // DILWAC_H