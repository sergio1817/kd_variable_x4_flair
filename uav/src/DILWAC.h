#ifndef DILWAC_H
#define DILWAC_H

class DILWAC
{
    public:
        DILWAC();
        ~DILWAC();

        Eigen::MatrixXf learn(float delta_t);
        
        void setLearningParameters(const uint16_t gamma, const uint8_t P, const uint16_t kc, const uint8_t sq_eps);
        
        void setLevant(float alpha_l, float lamb_l);

    private:
        uint16_t gamma, kc;
        uint8_t P; 
        float kc, gamma_e, sq_eps;
        
        Eigen::MatrixXf Lambda, GammaC, wa, xa, Psi, xc, xcp;
        Eigen::VectorXf r, ec, wc, J, uc, Jp, int_ec, we;
        Eigen::Quaternion q, qd, qp, qpd, qe, qep;

        Eigen::MatrixXf DILWAC::ANN(const Eigen::MatrixXf& xa, const Eigen::MatrixXf& wa);
        Eigen::MatrixXf DILWAC::squareKD(const Eigen::MatrixXf ANN);
        Eigen::MatrixXf DILWAC::wa_update(const Eigen::MatrixXf& Lambda, const Eigen::MatrixXf& Psi);
        void Psi(const Eigen::VectorXf Sq, const Eigen::VectorXf Jp, Eigen::MatrixXf& Psi);
        void CNN(const Eigen::VectorXf& wc, const Eigen::MatrixXf& xc, Eigen::VectorXf& J) 
        Eigen::VectorXf wc_update(const Eigen::MatrixXf& GammaC, const Eigen::VectorXf& ec, const Eigen::VectorXf& r, const Eigen::MatrixXf& xc, const Eigen::VectorXf& wc,const Eigen::MatrixXf& xcp);

};

#endif // DILWAC_H