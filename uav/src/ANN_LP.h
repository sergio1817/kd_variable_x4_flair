#ifndef ANN_LP_H
#define ANN_LP_H

#include <Eigen/Dense>

class ANN_LP
{
    private:
    Eigen::Matrix<float,4,10> Va;
    Eigen::Matrix<float,10,3> Wa;
    Eigen::Matrix<float,10,3> Wadot;
    Eigen::Matrix<float,10,1> Za;
    Eigen::Matrix<float,10,1> Xi;
    Eigen::Matrix<float,10,10> GAMMA_a;
    Eigen::Vector4f xa;
    Eigen::Vector3f int_nur;  
    Eigen::Vector3f nur;    

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ANN_LP();
        ~ANN_LP();



        // METODOS LUIS PANTOJA
        float Sigmoid(float z);

        void reset(Eigen::Matrix<float,10,3> Wa_0);
        void getInputs(const Eigen::Vector3f& nur2,Eigen::Matrix<float,4,10>& Va2, const Eigen::Matrix<float,10,10>&GAMMA_a2,float delta_t);
        void computeVect_Za();
        void updateWeights(float gamma_hat, float r, float delta_t);
        Eigen::Vector3f computeActorOutput();
        Eigen::Matrix<float,10,3> getWeights();
};

#endif // ANN_H