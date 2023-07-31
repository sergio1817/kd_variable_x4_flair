#ifndef ANN_H
#define ANN_H

#include <Eigen/Dense>

class ANN 
{
    public:
        ANN(const uint16_t DoF);
        ~ANN();

        void setLearningParameters(const Eigen::MatrixXf& Lambda_);
        
        Eigen::MatrixXf learnDamping(const Eigen::VectorXf& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep, const Eigen::VectorXf& sq, const Eigen::VectorXf& r, const Eigen::MatrixXf& Jp, float delta_t);

        void getInputs(const Eigen::VectorXf& we, const Eigen::Quaternionf& qe, const Eigen::Quaternionf& qep);

        Eigen::MatrixXf getK() const { return K; }       
        Eigen::MatrixXf getPsi() const { return Psi; }

    private:
        Eigen::MatrixXf Lambda;
        Eigen::MatrixXf xa;
        Eigen::MatrixXf K;
        Eigen::MatrixXf wa;
        Eigen::MatrixXf Psi;

        void computeFeedbackGain();
        Eigen::MatrixXf updateWeights(const Eigen::MatrixXf& Psi, const Eigen::MatrixXf& Lambda);
        void learningMonitor(const Eigen::VectorXf& sq, const Eigen::VectorXf& r, const Eigen::MatrixXf& Jp);
};

#endif // ANN_H