#ifndef CNN_H
#define CNN_H

#include <Eigen/Dense>
#include "NMethods.h"

class CNN
{
    public:
        CNN(const uint16_t DoF, const uint16_t criticDoF);
        ~CNN();

        void setLearningParameters(const int gamma_, const int penality_, Eigen::MatrixXf& GammaC_);

        void setVirtualControlParameters(float kc_, float miuc_);
        void setLevant(float alpha_l, float lamb_l);

        Eigen::VectorXf learnFromInteraction(const Eigen::Quaternionf qe, const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qep, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, const Eigen::VectorXf& r, float delta_t);

        void getInputs(const Eigen::Quaternionf qe, const Eigen::Quaternionf qd, const Eigen::Quaternionf q, const Eigen::Quaternionf qep, const Eigen::Quaternionf qp, const Eigen::Quaternionf qdp, const Eigen::VectorXf& r);

        Eigen::VectorXf getEc() const { return *ec; }
        Eigen::VectorXf getJ() const { return *J; }

    private:
        float* kc;
        float* gamma;
        float* miuc;
        float* penality;

        Eigen::MatrixXf* xc;
        Eigen::MatrixXf* xcp;
        Eigen::MatrixXf* GammaC;

        Eigen::VectorXf* wc;
        Eigen::VectorXf* J;
        Eigen::VectorXf* ec;
        Eigen::VectorXf* int_r;
        Eigen::VectorXf* int_ec;

        Levant_diff levant;

        
        void approximateValueFunction();
        Eigen::VectorXf updateWeights(const Eigen::VectorXf& r);
        Eigen::VectorXf computeBellmanError(const Eigen::VectorXf& r, const Eigen::VectorXf& uc, const Eigen::VectorXf Jp);
        Eigen::VectorXf computeBellmanControl(const Eigen::VectorXf& int_ec);

};

#endif // CNN_H