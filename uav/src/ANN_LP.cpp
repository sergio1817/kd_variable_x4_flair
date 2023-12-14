#include "ANN_LP.h"
#include "NMethods.h"



ANN_LP::ANN_LP()
{   
    printf("ANN constructor in\n");
    Va=Eigen::Matrix<float,4,10>::Zero();
    Wa=Eigen::Matrix<float,10,3>::Zero();
    Za=Eigen::Matrix<float,10,1>::Zero();
    Xi=Eigen::Matrix<float,10,1>::Zero();
    GAMMA_a=Eigen::Matrix<float,10,10>::Zero();
    xa=Eigen::Vector4f::Zero();
    printf("ANN constructor out\n");
}


ANN_LP::~ANN_LP() { }

float ANN_LP::Sigmoid(float z) {
    return (1-exp(-z))/(1+exp(-z));
}


void ANN_LP::reset(Eigen::Matrix<float,10,3> Wa_0){
    Wa = Wa_0;
    Za=Eigen::Matrix<float,10,1>::Zero();
    Xi=Eigen::Matrix<float,10,1>::Zero();
    xa=Eigen::Vector4f::Zero();
    Wadot = Eigen::Matrix<float,10,3>::Zero();
}

void ANN_LP::getInputs(const Eigen::Vector3f& nur2,Eigen::Matrix<float,4,10>& Va2, const Eigen::Matrix<float,10,10>&GAMMA_a2, float delta_t)
{   
    Va=Va2;
    nur=nur2;
    GAMMA_a=GAMMA_a2;
    int_nur=rk4_vec(int_nur, nur, delta_t);
    xa(0) = 1;
    xa.segment(1,3) = int_nur; 

}

void ANN_LP::computeVect_Za()

{  
   
    Xi=Va.transpose()*xa;

    Za(0)=Sigmoid(Xi(0));
    Za(1)=Sigmoid(Xi(1));
    Za(2)=Sigmoid(Xi(2));
    Za(3)=Sigmoid(Xi(3));
    Za(4)=Sigmoid(Xi(4));
    Za(5)=Sigmoid(Xi(5));
    Za(6)=Sigmoid(Xi(6));
    Za(7)=Sigmoid(Xi(7));
    Za(8)=Sigmoid(Xi(8));
    Za(9)=Sigmoid(Xi(9));
}

void  ANN_LP::updateWeights(float gamma_hat, float r, float delta_t) //falta calcular reward y gammahat ADEMAS DEFINIR BIEN LAS DIMENSIONES
{
    
    Wadot = -GAMMA_a*Za*(nur.transpose())-GAMMA_a*Wa*(gamma_hat*r)*(gamma_hat*r); //DEFINIR BIEN LAS DIMENSIONES
    
    Wa(0,0)=rk4(function1d, Wa(0,0), Wadot(0,0), delta_t);
    Wa(1,0)=rk4(function1d, Wa(1,0), Wadot(1,0), delta_t);
    Wa(2,0)=rk4(function1d, Wa(2,0), Wadot(2,0), delta_t);
    Wa(3,0)=rk4(function1d, Wa(3,0), Wadot(3,0), delta_t);
    Wa(4,0)=rk4(function1d, Wa(4,0), Wadot(4,0), delta_t);
    Wa(5,0)=rk4(function1d, Wa(5,0), Wadot(5,0), delta_t);
    Wa(6,0)=rk4(function1d, Wa(6,0), Wadot(6,0), delta_t);
    Wa(7,0)=rk4(function1d, Wa(7,0), Wadot(7,0), delta_t);
    Wa(8,0)=rk4(function1d, Wa(8,0), Wadot(8,0), delta_t);
    Wa(9,0)=rk4(function1d, Wa(9,0), Wadot(9,0), delta_t);

    Wa(0,1)=rk4(function1d, Wa(0,1), Wadot(0,1), delta_t);
    Wa(1,1)=rk4(function1d, Wa(1,1), Wadot(1,1), delta_t);
    Wa(2,1)=rk4(function1d, Wa(2,1), Wadot(2,1), delta_t);
    Wa(3,1)=rk4(function1d, Wa(3,1), Wadot(3,1), delta_t);
    Wa(4,1)=rk4(function1d, Wa(4,1), Wadot(4,1), delta_t);
    Wa(5,1)=rk4(function1d, Wa(5,1), Wadot(5,1), delta_t);
    Wa(6,1)=rk4(function1d, Wa(6,1), Wadot(6,1), delta_t);
    Wa(7,1)=rk4(function1d, Wa(7,1), Wadot(7,1), delta_t);
    Wa(8,1)=rk4(function1d, Wa(8,1), Wadot(8,1), delta_t);
    Wa(9,1)=rk4(function1d, Wa(9,1), Wadot(9,1), delta_t);
    
    Wa(0,2)=rk4(function1d, Wa(0,2), Wadot(0,2), delta_t);
    Wa(1,2)=rk4(function1d, Wa(1,2), Wadot(1,2), delta_t);
    Wa(2,2)=rk4(function1d, Wa(2,2), Wadot(2,2), delta_t);
    Wa(3,2)=rk4(function1d, Wa(3,2), Wadot(3,2), delta_t);
    Wa(4,2)=rk4(function1d, Wa(4,2), Wadot(4,2), delta_t);
    Wa(5,2)=rk4(function1d, Wa(5,2), Wadot(5,2), delta_t);
    Wa(6,2)=rk4(function1d, Wa(6,2), Wadot(6,2), delta_t);
    Wa(7,2)=rk4(function1d, Wa(7,2), Wadot(7,2), delta_t);
    Wa(8,2)=rk4(function1d, Wa(8,2), Wadot(8,2), delta_t);
    Wa(9,2)=rk4(function1d, Wa(9,2), Wadot(9,2), delta_t);

    // actualizacion de mi esquema wadot=-GAMMA_a*Z_2*(Sr.')-GAMMA_a*W2_hat*(gamma_hat*r)^(2)
}

Eigen::Vector3f ANN_LP::computeActorOutput()
{    
    Eigen::Vector3f Yr_hat= Wa.transpose()*Za;
    return Yr_hat;
}