#ifndef ANN_H
#define ANN_H

#include <Eigen/Dense>

class ANN 
{
    public:
        ANN(const int& DoF);
        ~ANN();

        void setLearningParameters(const Eigen::Matrix3f& Lambda_);
        
        Eigen::Matrix3f learnDamping(const Eigen::Vector3f& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep, const Eigen::Vector3f& sq, const Eigen::Vector3f& r, const Eigen::Vector3f& Jp, float delta_t);

        void getInputs(const Eigen::Vector3f& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep);

        Eigen::Matrix3f getK() const { return K; }       
        Eigen::Matrix3f getPsi() const { return Psi; }

    private:
        Eigen::Matrix3f Lambda;
        Eigen::Matrix3f xa;
        Eigen::Matrix3f K;
        Eigen::Matrix3f wa;
        Eigen::Matrix3f Psi;

        void computeFeedbackGain();
        Eigen::Matrix3f updateWeights(const Eigen::Matrix3f& Psi, const Eigen::Matrix3f& Lambda);
        void learningMonitor(const Eigen::Vector3f& sq, const Eigen::Vector3f& r, const Eigen::Vector3f& Jp);
};

#endif // ANN_H