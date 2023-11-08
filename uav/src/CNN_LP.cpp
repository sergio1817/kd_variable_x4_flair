#include "CNN_LP.h"
#include "NMethods.h"


CNN_LP::CNN_LP()
{   
    printf("CNN constructor in\n");
    Vc=Eigen::Matrix<float,4,10>::Zero();
    wc  = Eigen::Matrix<float,10,1>::Zero();
    wcp =Eigen::Matrix<float,10,1>::Zero();
    xc    = Eigen::Vector4f::Zero();
    J     = 0;
    gamma_hat=0; 
    r=0;
    Zc=Eigen::Matrix<float,10,1>::Zero();
    Xic=Eigen::Matrix<float,10,1>::Zero();
    int_wc_Zc=0;
    int_r = 0;


    printf("CNN constructor out\n");
}

CNN_LP::~CNN_LP() { }

float CNN_LP::compute_reward() 
{
    

        r=(float) (0.5*qe.vec().transpose()*qe.vec()*Q+0.5*qep.vec().transpose()*qep.vec()*P).value();
   
    return r;
}


void CNN_LP::getInputs( Eigen::Quaternionf& qe_,Eigen::Quaternionf qep_, float Q_, float P_,const Eigen::Matrix<float,4,10>& Vc2,const float psi_,const float K_,const float Kw_)
{ 
    P=P_;
    Q=Q_;
    qe=qe_;
    qep=qep_;
     Vc=Vc2;
    psi=psi_;
    K=K_;
    Kw=Kw_;
    xc(0) = -1;
    xc.segment(1,3) = qe.vec().transpose();
    sgn_gamma_hat=0;

}



double CNN_LP::Sigmoid(const double z) {
    return (1-exp(-z))/(1+exp(-z));
}


void CNN_LP::computeVect_Zc()
{  
    Xic=Vc.transpose()*xc;
    Zc(0)=Sigmoid(Xic(0));
    Zc(1)=Sigmoid(Xic(1));
    Zc(2)=Sigmoid(Xic(2));
    Zc(3)=Sigmoid(Xic(3));
    Zc(4)=Sigmoid(Xic(4));
    Zc(5)=Sigmoid(Xic(5));
    Zc(6)=Sigmoid(Xic(6));
    Zc(7)=Sigmoid(Xic(7));
    Zc(8)=Sigmoid(Xic(8));
    Zc(9)=Sigmoid(Xic(9));

}

void CNN_LP::approximateValueFunction()
{
    J = wc.transpose()*Zc;
}

float CNN_LP::computeIntTDerror(float delta_t)
{

    int_r=rk4(function1d, int_r, r, delta_t);

    float wc_Zc=wc.transpose()*Zc;


    int_wc_Zc=rk4(function1d, int_wc_Zc, wc_Zc, delta_t);
  
    gamma_hat= int_r+wc.transpose()*Zc-(1/psi)*int_wc_Zc; 
    return gamma_hat; 
    
    }

void CNN_LP::updateWeights(float delta_t) 
{
   
  sgn_gamma_hat=sign(gamma_hat);
  Eigen::Matrix<float,10,1> SignodeWc;

  SignodeWc << sign(wc(0)),sign(wc(1)),sign(wc(2)),sign(wc(3)),sign(wc(4)),sign(wc(5)),sign(wc(6)),sign(wc(7)),sign(wc(8)),sign(wc(9));

   wcp=-K*(Zc/(Zc.transpose()*Zc))*sgn_gamma_hat-Kw*SignodeWc;

    wc(0,0)=rk4(function1d, wc(0,0), wcp(0,0), delta_t);
    wc(1,0)=rk4(function1d, wc(1,0), wcp(1,0), delta_t);
    wc(2,0)=rk4(function1d, wc(2,0), wcp(2,0), delta_t);
    wc(3,0)=rk4(function1d, wc(3,0), wcp(3,0), delta_t);
    wc(4,0)=rk4(function1d, wc(4,0), wcp(4,0), delta_t);
    wc(5,0)=rk4(function1d, wc(5,0), wcp(5,0), delta_t);
    wc(6,0)=rk4(function1d, wc(6,0), wcp(6,0), delta_t);
    wc(7,0)=rk4(function1d, wc(7,0), wcp(7,0), delta_t);
    wc(8,0)=rk4(function1d, wc(8,0), wcp(8,0), delta_t);
    wc(9,0)=rk4(function1d, wc(9,0), wcp(9,0), delta_t);
    
}




