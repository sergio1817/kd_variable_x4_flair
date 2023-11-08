// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/01/01
//  filename:   Sliding_kdvar_h.cpp
//
//  author:     Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a position sliding mode controller
//
//
/*********************************************************************/
#include "Sliding_kdvar_h.h"
#include "NMethods.h"
#include "DILWAC.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <Eigen/Dense>
#include <TabWidget.h>
#include <CheckBox.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>
#include <iostream>
#include <Label.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

namespace flair {
namespace filter {

Sliding_kdvar_h::Sliding_kdvar_h(const LayoutPosition *position, string name): ControlLaw(position->getLayout(), name, 4){ // Salidas 4
    first_update = true;
    // init matrix
    input = new Matrix(this, 4, 11, floatType, name);

    MatrixDescriptor *desc = new MatrixDescriptor(26, 1);
    desc->SetElementName(0, 0, "u_roll");
    desc->SetElementName(1, 0, "u_pitch");
    desc->SetElementName(2, 0, "u_yaw");
    desc->SetElementName(3, 0, "u_z");
    desc->SetElementName(4, 0, "roll_d");
    desc->SetElementName(5, 0, "pitch_d");
    desc->SetElementName(6, 0, "yaw_d");
    desc->SetElementName(7, 0, "nurp_x");
    desc->SetElementName(8, 0, "nurp_y");
    desc->SetElementName(9, 0, "nurp_z");
    desc->SetElementName(10, 0, "nur_roll");
    desc->SetElementName(11, 0, "nur_pitch");
    desc->SetElementName(12, 0, "nur_yaw");
    desc->SetElementName(13, 0, "kd_phi");
    desc->SetElementName(14, 0, "kd_theta");
    desc->SetElementName(15, 0, "kd_psi");
    desc->SetElementName(16, 0, "r_phi");
    desc->SetElementName(17, 0, "r_theta");
    desc->SetElementName(18, 0, "r_psi");
    desc->SetElementName(19, 0, "ec_phi");
    desc->SetElementName(20, 0, "ec_theta");
    desc->SetElementName(21, 0, "ec_psi");
    desc->SetElementName(22, 0, "J_phi");
    desc->SetElementName(23, 0, "J_theta");
    desc->SetElementName(24, 0, "J_psi");
    desc->SetElementName(25, 0, "battery");
    state = new Matrix(this, desc, floatType, name);
    delete desc;


    GroupBox *reglages_groupbox = new GroupBox(position, name);
    GroupBox *num = new GroupBox(reglages_groupbox->NewRow(), "Integral y derivada");
    GroupBox *ori = new GroupBox(reglages_groupbox->NewRow(), "Orientacion");
    GroupBox *pos = new GroupBox(reglages_groupbox->NewRow(), "Posicion");
    GroupBox *mot = new GroupBox(reglages_groupbox->NewRow(), "Motores");

    T = new DoubleSpinBox(num->NewRow(), "period, 0 for auto", " s", 0, 1, 0.01,3);
    alpha_l = new DoubleSpinBox(num->NewRow(), "alpha Levant:", 0, 500, 0.001, 3);
    lamb_l = new DoubleSpinBox(num->LastRowLastCol(), "lambda Levant:", 0, 500, 0.001, 3);
    levantd = new CheckBox(num->LastRowLastCol(), "Levant");

    gamma_roll = new DoubleSpinBox(ori->NewRow(), "gamma_roll:", 0, 500, 0.001, 3);
    gamma_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "gamma_pitch:", 0, 500, 0.001, 3);
    gamma_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "gamma_yaw:", 0, 500, 0.001, 3);
    alpha_roll = new DoubleSpinBox(ori->NewRow(), "alpha_roll:", 0, 50000, 0.5, 3);
    alpha_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "alpha_pitch:", 0, 50000, 0.5, 3);
    alpha_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "alpha_yaw:", 0, 50000, 0.5, 3);
    k = new DoubleSpinBox(ori->NewRow(), "k:", 0, 50000, 0.5, 3);
    p = new DoubleSpinBox(ori->LastRowLastCol(), "p:", 0, 50000, 1, 3);
    lo = new Label(ori->LastRowLastCol(), "Latencia ori");
    Kd_roll = new DoubleSpinBox(ori->NewRow(), "Kd_rol:", 0, 50000, 0.5, 3);
    Kd_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "Kd_pitch:", 0, 50000, 0.5, 3);
    Kd_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "Kd_yaw:", 0, 50000, 0.5, 3);

    gamma_x = new DoubleSpinBox(pos->NewRow(), "gamma_x:", 0, 500, 0.001, 3);
    gamma_y = new DoubleSpinBox(pos->LastRowLastCol(), "gamma_y:", 0, 500, 0.001, 3);
    gamma_z = new DoubleSpinBox(pos->LastRowLastCol(), "gamma_z:", 0, 500, 0.001, 3);
    alpha_x = new DoubleSpinBox(pos->NewRow(), "alpha_x:", 0, 50000, 0.5, 3);
    alpha_y = new DoubleSpinBox(pos->LastRowLastCol(), "alpha_y:", 0, 50000, 0.5, 3);
    alpha_z = new DoubleSpinBox(pos->LastRowLastCol(), "alpha_z:", 0, 50000, 0.5, 3);
    Kp_x = new DoubleSpinBox(pos->NewRow(), "Kp_x:", 0, 50000, 0.5, 3);
    Kp_y = new DoubleSpinBox(pos->LastRowLastCol(), "Kp_y:", 0, 50000, 0.5, 3);
    Kp_z = new DoubleSpinBox(pos->LastRowLastCol(), "Kp_z:", 0, 50000, 0.5, 3);
    

    sat_r = new DoubleSpinBox(mot->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(mot->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(mot->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(mot->LastRowLastCol(), "sat thrust:", 0, 1, 0.1);
    
    km = new DoubleSpinBox(mot->NewRow(), "km:", -100, 100, 0.01, 3);
    km_z = new DoubleSpinBox(mot->LastRowLastCol(), "km_z:", -100, 100, 0.01, 3);
    
    m = new DoubleSpinBox(pos->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(pos->LastRowLastCol(),"g",-10,10,0.01,3);
    lp = new Label(pos->LastRowLastCol(), "Latencia pos");
    
    t0 = double(GetTime())/1000000000;

    levant = Levant_diff("tanh", 8, 6, 3000);

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;

    sgnori_p << 0,0,0;
    sgnori << 0,0,0;

    kd_var = new DILWAC(3,4);

    pert = new CheckBox(num->LastRowLastCol(), "Perturbacion");
    pert_g = new DoubleSpinBox(num->LastRowLastCol(), "Gain Pert:", 0, 1, 0.1);

    AddDataToLog(state);
    
    
}

Sliding_kdvar_h::~Sliding_kdvar_h(void) {}

void Sliding_kdvar_h::Reset(void) {
    first_update = true;
    t0 = 0;
    t0 = double(GetTime())/1000000000;
    sgnori_p << 0,0,0;
    sgnori << 0,0,0;

    levant.Reset();
    kd_var->forgetDamping();

    // sgnpos2 = Vector3ff(0,0,0);
    // sgn2 = Vector3ff(0,0,0);

    // sgnpos << 0,0,0;
    // sgn << 0,0,0;

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;


//    pimpl_->i = 0;
//    pimpl_->first_update = true;
}

void Sliding_kdvar_h::SetValues(Vector3Df xie, Vector3Df xiep, Vector3Df xid, Vector3Df xidpp, Vector3Df xidppp, Vector3Df w, Quaternion q, 
                                Vector3Df Lambda, Vector3Df GammaC, int gamma, int p, float goal, float alph_l, float lamb_l, Quaternion q_p, 
                                float battery){

    // float xe = xie.x;
    // float ye = xie.y;
    // float ze = xie.z;

    // float xep = xiep.x;
    // float yep = xiep.y;
    // float zep = xiep.z;

    // float xd = xid.x;
    // float yd = xid.y;
    // float zd = xid.z;

    // float xdp = xidp.x;
    // float ydp = xidp.y;
    // float zdp = xidp.z;

    // float xdpp = xidpp.x;
    // float ydpp = xidpp.y;
    // float zdpp = xidpp.z;

    // float xdppp = xidppp.x;
    // float ydppp = xidppp.y;
    // float zdppp = xidppp.z;

    // float wex = we.x;
    // float wey = we.y;
    // float wez = we.z;

    // float q0 = q.q0;
    // float q1 = q.q1;
    // float q2 = q.q2;
    // float q3 = q.q3;

    input->SetValue(0, 0, xie.x);
    input->SetValue(1, 0, xie.y);
    input->SetValue(2, 0, xie.z);

    input->SetValue(3, 0, battery);

    input->SetValue(0, 1, xiep.x);
    input->SetValue(1, 1, xiep.y);
    input->SetValue(2, 1, xiep.z);

    input->SetValue(0, 2, xid.x);
    input->SetValue(1, 2, xid.y);
    input->SetValue(2, 2, xid.z);

    input->SetValue(0, 4, xidpp.x);
    input->SetValue(1, 4, xidpp.y);
    input->SetValue(2, 4, xidpp.z);

    input->SetValue(0, 5, xidppp.x);
    input->SetValue(1, 5, xidppp.y);
    input->SetValue(2, 5, xidppp.z);

    input->SetValue(0, 6, w.x);
    input->SetValue(1, 6, w.y);
    input->SetValue(2, 6, w.z);

    input->SetValue(0, 7, q.q0);
    input->SetValue(1, 7, q.q1);
    input->SetValue(2, 7, q.q2);
    input->SetValue(3, 7, q.q3);

    input->SetValue(0, 3, Lambda.x);
    input->SetValue(1, 3, Lambda.y);
    input->SetValue(2, 3, Lambda.z);

    input->SetValue(0, 8, GammaC.x);
    input->SetValue(1, 8, GammaC.y);
    input->SetValue(2, 8, GammaC.z);

    input->SetValue(0, 9, gamma);
    input->SetValue(1, 9, p);
    input->SetValue(2, 9, alph_l);
    input->SetValue(3, 9, lamb_l);

    input->SetValue(3, 8, goal);

    input->SetValue(0, 10, q_p.q0);
    input->SetValue(1, 10, q_p.q1);
    input->SetValue(2, 10, q_p.q2);
    input->SetValue(3, 10, q_p.q3);


//   input->SetValue(0, 0, ze);
//   input->SetValue(1, 0, wex);
//   input->SetValue(2, 0, wey);
//   input->SetValue(3, 0, wez);
//   input->SetValue(4, 0, zp);

//   input->SetValue(0, 1, q0);
//   input->SetValue(1, 1, q1);
//   input->SetValue(2, 1, q2);
//   input->SetValue(3, 1, q3);

//   input->SetValue(0, 2, qd0);
//   input->SetValue(1, 2, qd1);
//   input->SetValue(2, 2, qd2);
//   input->SetValue(3, 2, qd3);
}

void Sliding_kdvar_h::UseDefaultPlot(const LayoutPosition *position) {
    DataPlot1D *rollg = new DataPlot1D(position, "u_roll", -1, 1);
    rollg->AddCurve(state->Element(0));
    
}

void Sliding_kdvar_h::UseDefaultPlot2(const LayoutPosition *position) {
    DataPlot1D *pitchg = new DataPlot1D(position, "u_pitch", -1, 1);
    pitchg->AddCurve(state->Element(1));
    
}

void Sliding_kdvar_h::UseDefaultPlot3(const LayoutPosition *position) {
    DataPlot1D *yawg = new DataPlot1D(position, "u_yaw", -1, 1);
    yawg->AddCurve(state->Element(2));
    
}

void Sliding_kdvar_h::UseDefaultPlot4(const LayoutPosition *position) {    
    DataPlot1D *uz = new DataPlot1D(position, "u_z", -1, 1);
    uz->AddCurve(state->Element(3));
    
}

void Sliding_kdvar_h::UseDefaultPlot5(const LayoutPosition *position) {    
    DataPlot1D *r = new DataPlot1D(position, "r", -3.14, 3.14);
    r->AddCurve(state->Element(4));
    
}

void Sliding_kdvar_h::UseDefaultPlot6(const LayoutPosition *position) {    
    DataPlot1D *p = new DataPlot1D(position, "p", -3.14, 3.14);
    p->AddCurve(state->Element(5));
    
}

void Sliding_kdvar_h::UseDefaultPlot7(const LayoutPosition *position) {    
    DataPlot1D *y = new DataPlot1D(position, "y", -3.14, 3.14);
    y->AddCurve(state->Element(6));
    
}

void Sliding_kdvar_h::UseDefaultPlot8(const LayoutPosition *position) {    
    DataPlot1D *Sp = new DataPlot1D(position, "nu_rp", -5, 5);
    Sp->AddCurve(state->Element(7), DataPlot::Red);
    Sp->AddCurve(state->Element(8), DataPlot::Green);
    Sp->AddCurve(state->Element(9), DataPlot::Blue);
    
}

void Sliding_kdvar_h::UseDefaultPlot9(const LayoutPosition *position) {    
    DataPlot1D *Sq = new DataPlot1D(position, "nu_r", -5, 5);
    Sq->AddCurve(state->Element(10), DataPlot::Green);
    Sq->AddCurve(state->Element(11), DataPlot::Red);
    Sq->AddCurve(state->Element(12), DataPlot::Black);
    
}

void Sliding_kdvar_h::UseDefaultPlot10(const LayoutPosition *position) {    
    DataPlot1D *Kd = new DataPlot1D(position, "Kd", -1, 10);
    Kd->AddCurve(state->Element(13), DataPlot::Green);
    Kd->AddCurve(state->Element(14), DataPlot::Red);
    Kd->AddCurve(state->Element(15), DataPlot::Black);
    
}

void Sliding_kdvar_h::UseDefaultPlot11(const LayoutPosition *position) {    
    DataPlot1D *rew = new DataPlot1D(position, "reward", -110, 10);
    rew->AddCurve(state->Element(16), DataPlot::Green);
    rew->AddCurve(state->Element(17), DataPlot::Red);
    rew->AddCurve(state->Element(18), DataPlot::Black);
    
}

void Sliding_kdvar_h::UseDefaultPlot12(const LayoutPosition *position) {    
    DataPlot1D *ec = new DataPlot1D(position, "ec", -5, 5);
    ec->AddCurve(state->Element(19), DataPlot::Green);
    ec->AddCurve(state->Element(20), DataPlot::Red);
    ec->AddCurve(state->Element(21), DataPlot::Black);
    
}

void Sliding_kdvar_h::UseDefaultPlot13(const LayoutPosition *position) {    
    DataPlot1D *J = new DataPlot1D(position, "J", 0, 3000);
    J->AddCurve(state->Element(22), DataPlot::Green);
    J->AddCurve(state->Element(23), DataPlot::Red);
    J->AddCurve(state->Element(24), DataPlot::Black);
    
}


void Sliding_kdvar_h::UpdateFrom(const io_data *data) {
    float tactual=double(GetTime())/1000000000-t0;
    //Printf("tactual: %f\n",tactual);
    float Trs=0, tau_roll=0, tau_pitch=0, tau_yaw=0, Tr=0;
    Eigen::Vector3f ez(0,0,1);
    
    Eigen::Vector3f alphap_v(alpha_x->Value(), alpha_y->Value(), alpha_z->Value());
    Eigen::Matrix3f alphap = alphap_v.asDiagonal();

    Eigen::Vector3f gammap_v(gamma_x->Value(), gamma_y->Value(), gamma_z->Value());
    Eigen::Matrix3f gammap = gammap_v.asDiagonal();

    Eigen::Vector3f Kpv(Kp_x->Value(), Kp_y->Value(), Kp_z->Value());
    Eigen::Matrix3f Kpm = Kpv.asDiagonal();

    Eigen::Vector3f alphao_v(alpha_roll->Value(), alpha_pitch->Value(), alpha_yaw->Value());
    Eigen::Matrix3f alphao = alphao_v.asDiagonal();

    Eigen::Vector3f gammao_v(gamma_roll->Value(), gamma_pitch->Value(), gamma_yaw->Value());
    Eigen::Matrix3f gammao = gammao_v.asDiagonal();

    Eigen::Vector3f Kdv(Kd_roll->Value(), Kd_pitch->Value(), Kd_yaw->Value());
    //Eigen::Matrix3f Kdm = Kdv.asDiagonal();

    if (T->Value() == 0) {
        delta_t = (float)(data->DataDeltaTime()) / 1000000000.;
    } else {
        delta_t = T->Value();
    }
    
    if (first_update == true) {
        delta_t = 0;
        first_update = false;
    }

    const Matrix* input = dynamic_cast<const Matrix*>(data);
  
    if (!input) {
        Warn("casting %s to Matrix failed\n",data->ObjectName().c_str(),TIME_INFINITE);
        return;
    }


    input->GetMutex();

    Eigen::Vector3f xie(input->ValueNoMutex(0, 0),input->ValueNoMutex(1, 0),input->ValueNoMutex(2, 0));
    Eigen::Vector3f xiep(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1));

    Eigen::Vector3f xid(input->ValueNoMutex(0, 2),input->ValueNoMutex(1, 2),input->ValueNoMutex(2, 2));
    Eigen::Vector3f xidpp(input->ValueNoMutex(0, 4),input->ValueNoMutex(1, 4),input->ValueNoMutex(2, 4));
    Eigen::Vector3f xidppp(input->ValueNoMutex(0, 5),input->ValueNoMutex(1, 5),input->ValueNoMutex(2, 5));

    Eigen::Vector3f w(input->ValueNoMutex(0, 6),input->ValueNoMutex(1, 6),input->ValueNoMutex(2, 6));

    Eigen::Quaternionf q(input->ValueNoMutex(0, 7),input->ValueNoMutex(1, 7),input->ValueNoMutex(2, 7),input->ValueNoMutex(3, 7));

    Eigen::Matrix3f Lambda = Eigen::Vector3f(input->ValueNoMutex(0, 3),input->ValueNoMutex(1, 3),input->ValueNoMutex(2, 3)).asDiagonal();
    Eigen::Matrix4f GammaC = Eigen::Vector4f(input->ValueNoMutex(0, 8),input->ValueNoMutex(1, 8),input->ValueNoMutex(2, 8),input->ValueNoMutex(2, 8)).asDiagonal();

    float gamma = input->ValueNoMutex(0, 9);
    float penalty = input->ValueNoMutex(1, 9);
    float alph_l2 = input->ValueNoMutex(2, 9);
    float lamb_l2 = input->ValueNoMutex(3, 9);
    float goal = input->ValueNoMutex(3, 8);

    Eigen::Quaternionf q_p(input->ValueNoMutex(0, 10),input->ValueNoMutex(1, 10),input->ValueNoMutex(2, 10),input->ValueNoMutex(3, 10));

    float battery = input->ValueNoMutex(3, 0);
    
    input->ReleaseMutex();

    flair::core::Time t0_p = GetTime();

    Eigen::Vector3f nup = xiep + alphap*xie;

    sgnpos_p = signth(nup,1);
    sgnpos = rk4_vec(sgnpos, sgnpos_p, delta_t);

    Eigen::Vector3f nurp = nup + gammap*sgnpos;

    Eigen::Vector3f xirpp = xidpp - alphap*xiep - gammap*sgnpos_p;


    Eigen::Vector3f u = -Kpm*nurp - m->Value()*g->Value()*ez + m->Value()*xirpp; //- m->Value()*g->Value()*ez + m->Value()*xirpp

    //printf("u: %f, %f, %f\n", u(0), u(1), u(2));

    Trs = u.norm();

    Eigen::Vector3f Qe3 = q.toRotationMatrix()*ez;
    
    Eigen::Vector3f Lambpv(powf(sech(nup(0)*1),2), powf(sech(nup(1)*1),2), powf(sech(nup(2)*1),2) );
    Eigen::Matrix3f Lambp = Lambpv.asDiagonal();

    //Eigen::Vector3f vec(sin(tactual), sin(tactual), sin(tactual));

    // float f = gammap->Value()*sin(alphap->Value()*tactual);
    // float alpha2 = Kp->Value();
    // float lamb = Kd->Value();

    

    //ud = levant.Compute(f,delta_t);

    Eigen::Vector3f up;
    // Eigen::Vector3f ud;
    if(levantd->IsChecked()){
        levant.setParam(alpha_l->Value(), lamb_l->Value());
        up = levant.Compute(u,delta_t);
        //ud = levant.Compute(vec,delta_t);
    }else{
        up = -(Kpm + m->Value()*alphap + m->Value()*gammap*Lambp) * (g->Value()*ez - (Trs/m->Value())*Qe3 - xidpp) 
                        -alphap*(Kpm + m->Value()*gammap*Lambp)*xiep - Kpm*gammap*sgnpos_p + m->Value()*xidppp;
    }

    

    Eigen::Vector3f uh = u.normalized();
    Eigen::Vector3f uph = ((u.transpose()*u)*up - (u.transpose()*up)*u)/(powf(u.norm(),3));

    //std::cout << "uph: " << uph << std::endl;


    Eigen::Quaternionf qd( (0.5)*sqrtf(-2*uh(2)+2) , uh(1)/sqrtf(-2*uh(2)+2), -uh(0)/sqrtf(-2*uh(2)+2), 0);
    

    Eigen::Quaternionf qdp(-(0.5)*(uph(2)/sqrtf(-2*uh(2)+2)),
                            (uph(1)/sqrtf(-2*uh(2)+2)) + ((uh(1)*uph(2))/powf(-2*uh(2)+2,1.5)),
                            -(uph(0)/sqrtf(-2*uh(2)+2)) - ((uh(0)*uph(2))/powf(-2*uh(2)+2,1.5)),
                            0);

    q_p = q_p.coeffs()*pert_g->Value();

    //Eigen::Quaternionf qd;

    if(pert->IsChecked()){
        q = q*(q_p);
    }else{
        q = q;
    }

    Quaternion qd2 = Quaternion(qd.w(),qd.x(),qd.y(),qd.z());
    Euler eta = qd2.ToEuler();
    // Eigen::Vector3f eta = qd.toRotationMatrix().eulerAngles(0, 1, 2);

    Eigen::Quaternionf qe = q*qd.conjugate();

    //std::cout<<"qe: " << qe.coeffs() << std::endl;

    Eigen::Vector3f wd(uph(1) - ( (uh(1)*uph(2))/(1-uh(2)) ), 
                        -uph(0) + ( (uh(0)*uph(2))/(1-uh(2)) ), 
                        (uh(1)*uph(0) - uh(0)*uph(1))/(1-uh(2)));

    //Eigen::Vector3f wd = 2*(qd.conjugate()*qdp).vec();

    //std::cout<<"w: " << w << std::endl;
    //std::cout<<"wd: " << wd << std::endl;

    kd_var->setANN(Lambda);
    kd_var->setCNN(gamma, penalty, GammaC, goal, alph_l2, lamb_l2);

    flair::core::Time dt_pos = GetTime() - t0_p;

    //lp->SetText("Latecia pos: %.3f ms",(float)dt_pos/1000000);

    

    flair::core::Time t0_o = GetTime();

    Eigen::Vector3f we = w - wd;

    Eigen::Quaternionf wet(0, we(0), we(1), we(2));
    Eigen::Quaternionf wt(0, w(0), w(1), w(2));

    Eigen::Quaternionf qep1 = q*wet*qd.conjugate();

    Eigen::Quaternionf qep(qep1.coeffs()*0.5);

    Eigen::Quaternionf qp1 = q*wt;

    Eigen::Quaternionf qp(qp1.coeffs()*0.5);

    //std::cout<<"we: " << we << std::endl;

    Eigen::Quaternionf QdTqe3 = qd.conjugate()*qe*qd;

    //std::cout<<"QdTqe3: " << QdTqe3.coeffs() << std::endl;

    Eigen::Vector3f nu = we + alphao*QdTqe3.vec();

    //std::cout<<"nu: " << nu << std::endl;
    
    Eigen::Vector3f nu_t0 = 0.01*Eigen::Vector3f(1,1,1);
    
    Eigen::Vector3f nud = nu_t0*exp(-k->Value()*(tactual));
    
    Eigen::Vector3f nuq = nu-nud;

    sgnori_p = signth(nuq,p->Value());
    sgnori = rk4_vec(sgnori, sgnori_p, delta_t);

    Eigen::Vector3f nur = nuq + gammao*sgnori;

    Eigen::Matrix3f Kdm = kd_var->learnDampingInjection(we, qe, qep, nuq, qd, q, qp, qdp, delta_t);

    printf("Kdm: %f %f %f\n",Kdm(0,0),Kdm(1,1),Kdm(2,2));

    Eigen::Vector3f reward = kd_var->getR();

    Eigen::Vector3f ec = kd_var->getEc();

    Eigen::Vector3f J = kd_var->getJ();

    Eigen::Matrix3f psi = kd_var->getPsi();

    Eigen::Vector3f tau = -Kdm*nur;

    flair::core::Time dt_ori = GetTime() - t0_o;

    //lo->SetText("Latecia ori: %.3f ms",(float)dt_ori/1000000);

    
    tau_roll = (float)tau(0)/km->Value();
    
    tau_pitch = (float)tau(1)/km->Value();
    
    tau_yaw = (float)tau(2)/km->Value();
    
    Tr = Trs/km_z->Value();
    
    tau_roll = -Sat(tau_roll,sat_r->Value());
    tau_pitch = -Sat(tau_pitch,sat_p->Value());
    tau_yaw = -Sat(tau_yaw,sat_y->Value());
    Tr = -Sat(Tr,sat_t->Value());
    
    state->GetMutex();
    state->SetValueNoMutex(0, 0, tau_roll);
    state->SetValueNoMutex(1, 0, tau_pitch);
    state->SetValueNoMutex(2, 0, tau_yaw);
    state->SetValueNoMutex(3, 0, Tr);
    state->SetValueNoMutex(4, 0, eta.roll);
    state->SetValueNoMutex(5, 0, eta.pitch);
    state->SetValueNoMutex(6, 0, eta.yaw);
    state->SetValueNoMutex(7, 0, nup.x());
    state->SetValueNoMutex(8, 0, nup.y());
    state->SetValueNoMutex(9, 0, nup.z());
    state->SetValueNoMutex(10, 0, nuq.x());
    state->SetValueNoMutex(11, 0, nuq.y());
    state->SetValueNoMutex(12, 0, nuq.z());
    state->SetValueNoMutex(13, 0, Kdm(0,0));
    state->SetValueNoMutex(14, 0, Kdm(1,1));
    state->SetValueNoMutex(15, 0, Kdm(2,2));
    state->SetValueNoMutex(16, 0, reward(0));
    state->SetValueNoMutex(17, 0, reward(1));
    state->SetValueNoMutex(18, 0, reward(2));
    state->SetValueNoMutex(19, 0, ec(0));
    state->SetValueNoMutex(20, 0, ec(1));
    state->SetValueNoMutex(21, 0, ec(2));
    state->SetValueNoMutex(22, 0, J(0));
    state->SetValueNoMutex(23, 0, J(1));
    state->SetValueNoMutex(24, 0, J(2));
    state->SetValueNoMutex(25, 0, battery);
    //state->SetDataTime(data->DataTime());
    state->ReleaseMutex();


    output->SetValue(0, 0, tau_roll);
    output->SetValue(1, 0, tau_pitch);
    output->SetValue(2, 0, tau_yaw);
    output->SetValue(3, 0, Tr);
    output->SetDataTime(data->DataTime());
    
    ProcessUpdate(output);
    //ProcessUpdate(state);
    
}

float Sliding_kdvar_h::Sat(float value, float borne) {
    if (value < -borne)
        return -borne;
    if (value > borne)
        return borne;
    return value;
}

float Sliding_kdvar_h::sech(float value) {
    return 1 / coshf(value);
}

} // end namespace filter
} // end namespace flair
