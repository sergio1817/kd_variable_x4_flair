#ifndef CNN_LP_H
#define CNN_LP_H

#include <Eigen/Dense>
#include "NMethods.h"

class CNN_LP
{
    private:

        float gamma_hat;
        float r=0;
         float psi=0;
         float K=0;
         float Kw=0;
        Eigen::Vector4f xc;
        Eigen::Matrix<float,10,1> wc;
        float J;
        float int_r;
        float Q;
        float P;
        float int_wc_Zc;
        float sgn_gamma_hat;
        float R;
        float J_hat;

        Eigen::Matrix<float,10,1> Zc;
        Eigen::Matrix<float,10,1> wcp;
        Eigen::Matrix<float,4,10> Vc;
        Eigen::Matrix<float,10,1> Xic;
        Eigen::Quaternionf  qe;
        Eigen::Quaternionf  qep;
        Eigen::Matrix<float,4,10> Vc2;

        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CNN_LP();
        ~CNN_LP();
         double Sigmoid(const double z);
         void getInputs( Eigen::Quaternionf& qe_,Eigen::Quaternionf qep_, float Q_, float P_,const Eigen::Matrix<float,4,10>& Vc2,const float psi_,const float K_,const float Kw_);


        float compute_reward();
        void computeVect_Zc();

        void reset(Eigen::Matrix<float,10,1> Wc_0);
        float approximateValueFunction();
        float ValueFunction(float t, float delta_t);

        void updateWeights(float delta_t);

        float computeIntTDerror(float delta_t);

        Eigen::Matrix<float,10,1> getWeights();

        
  
};

#endif // CNN_H