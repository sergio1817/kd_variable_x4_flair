#ifndef CNN_H
#define CNN_H

#include <Eigen/Dense>
#include "NMethods.h"

class CNN
{
    public:
        CNN(const uint16_t DoF, const uint16_t criticDoF);
        ~CNN();

        void setLearningParameters(const int gamma_, const int penality_, const Eigen::Matrix4f& GammaC_);

        void setVirtualControlParameters(float kc_, float miuc_);
        void setLevant(float alpha_l, float lamb_l);

        Eigen::Vector3f learnFromInteraction(const Eigen::Quaternionf qe, const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qep, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, const Eigen::Vector3f& r, float delta_t);

        void getInputs(const Eigen::Quaternionf qe, const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qep, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, const Eigen::Vector3f& r);

        Eigen::Vector3f getEc() const { return ec; }
        Eigen::Vector3f getJ() const { return J; }

    private:
        float kc;
        float gamma;
        float miuc;
        float penality;

        Eigen::Matrix<float, 4, 3> xc;
        Eigen::Matrix<float, 4, 3> xcp;
        Eigen::Matrix4f GammaC;

        Eigen::Vector4f wc;
        Eigen::Vector3f J;
        Eigen::Vector3f ec;
        Eigen::Vector3f int_r;
        Eigen::Vector3f int_ec;

        Levant_diff levant;

        
        void approximateValueFunction();
        Eigen::Vector4f updateWeights(const Eigen::Vector3f& r);
        Eigen::Vector3f computeBellmanError(const Eigen::Vector3f& r, const Eigen::Vector3f& uc, const Eigen::Vector3f& Jp);
        Eigen::Vector3f computeBellmanControl(const Eigen::Vector3f& int_ec);

};

#endif // CNN_H