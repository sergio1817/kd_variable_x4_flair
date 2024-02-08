// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/01/01
//  filename:   Sliding_LP.cpp
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
#include "Sliding_LP.h"
#include "NMethods.h"
#include "Actor_tauContribution_LP.h"
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

Sliding_LP::Sliding_LP(const LayoutPosition *position, const LayoutPosition *position_AC, string name): ControlLaw(position->getLayout(), name, 4){ // Salidas 4
    first_update = true;
    // init matrix
    input = new Matrix(this, 4, 11, floatType, name);

    MatrixDescriptor *desc = new MatrixDescriptor(30, 1);
    desc->SetElementName(0, 0, "u_roll");
    desc->SetElementName(1, 0, "u_pitch");
    desc->SetElementName(2, 0, "u_yaw");
    desc->SetElementName(3, 0, "u_z");
    desc->SetElementName(4, 0, "roll_d");
    desc->SetElementName(5, 0, "pitch_d");
    desc->SetElementName(6, 0, "yaw_d");
    desc->SetElementName(7, 0, "Srp_x");
    desc->SetElementName(8, 0, "Srp_y");
    desc->SetElementName(9, 0, "Srp_z");
    desc->SetElementName(10, 0, "Sq_roll");
    desc->SetElementName(11, 0, "Sq_pitch");
    desc->SetElementName(12, 0, "Sq_yaw");
    desc->SetElementName(13, 0, "Yr_phi");
    desc->SetElementName(14, 0, "Yr_theta");
    desc->SetElementName(15, 0, "Yr_psi");
    desc->SetElementName(16, 0, "tau_phi");
    desc->SetElementName(17, 0, "tau_theta");
    desc->SetElementName(18, 0, "tau_psi");
    desc->SetElementName(19, 0, "tauf_phi");
    desc->SetElementName(20, 0, "tauf_theta");
    desc->SetElementName(21, 0, "tauf_psi");
    desc->SetElementName(22, 0, "gamma_hat");
    desc->SetElementName(23, 0, "reward");
    desc->SetElementName(24, 0, "J");
    desc->SetElementName(25, 0, "batery");
    desc->SetElementName(26, 0, "Sr_roll");
    desc->SetElementName(27, 0, "Sr_ptich");
    desc->SetElementName(28, 0, "Sr_yaw");
    desc->SetElementName(29, 0, "J_hat");
    state = new Matrix(this, desc, floatType, name);
    delete desc;

    MatrixDescriptor *desc2 = new MatrixDescriptor(10, 3);
    desc2->SetElementName(0, 0, "wa(0,0)");
    desc2->SetElementName(1, 0, "wa(1,0)");
    desc2->SetElementName(2, 0, "wa(2,0)");
    desc2->SetElementName(3, 0, "wa(3,0)");
    desc2->SetElementName(4, 0, "wa(4,0)");
    desc2->SetElementName(5, 0, "wa(5,0)");
    desc2->SetElementName(6, 0, "wa(6,0)");
    desc2->SetElementName(7, 0, "wa(7,0)");
    desc2->SetElementName(8, 0, "wa(8,0)");
    desc2->SetElementName(9, 0, "wa(9,0)");

    desc2->SetElementName(0, 1, "wa(0,1)");
    desc2->SetElementName(1, 1, "wa(1,1)");
    desc2->SetElementName(2, 1, "wa(2,1)");
    desc2->SetElementName(3, 1, "wa(3,1)");
    desc2->SetElementName(4, 1, "wa(4,1)");
    desc2->SetElementName(5, 1, "wa(5,1)");
    desc2->SetElementName(6, 1, "wa(6,1)");
    desc2->SetElementName(7, 1, "wa(7,1)");
    desc2->SetElementName(8, 1, "wa(8,1)");
    desc2->SetElementName(9, 1, "wa(9,1)");

    desc2->SetElementName(0, 2, "wa(0,2)");
    desc2->SetElementName(1, 2, "wa(1,2)");
    desc2->SetElementName(2, 2, "wa(2,2)");
    desc2->SetElementName(3, 2, "wa(3,2)");
    desc2->SetElementName(4, 2, "wa(4,2)");
    desc2->SetElementName(5, 2, "wa(5,2)");
    desc2->SetElementName(6, 2, "wa(6,2)");
    desc2->SetElementName(7, 2, "wa(7,2)");
    desc2->SetElementName(8, 2, "wa(8,2)");
    desc2->SetElementName(9, 2, "wa(9,2)");


    wa = new Matrix(this, desc2, floatType, name);
    delete desc2;

    MatrixDescriptor *desc3 = new MatrixDescriptor(10, 1);
    desc3->SetElementName(0, 0, "wc(0,0)");
    desc3->SetElementName(1, 0, "wc(1,0)");
    desc3->SetElementName(2, 0, "wc(2,0)");
    desc3->SetElementName(3, 0, "wc(3,0)");
    desc3->SetElementName(4, 0, "wc(4,0)");
    desc3->SetElementName(5, 0, "wc(5,0)");
    desc3->SetElementName(6, 0, "wc(6,0)");
    desc3->SetElementName(7, 0, "wc(7,0)");
    desc3->SetElementName(8, 0, "wc(8,0)");
    desc3->SetElementName(9, 0, "wc(9,0)");
    wc = new Matrix(this, desc3, floatType, name);
    delete desc3;



    GroupBox *reglages_groupbox = new GroupBox(position, name);
    GroupBox *num = new GroupBox(reglages_groupbox->NewRow(), "Integral y derivada");
    GroupBox *ori = new GroupBox(reglages_groupbox->NewRow(), "Orientacion");
    GroupBox *pos = new GroupBox(reglages_groupbox->NewRow(), "Posicion");
    GroupBox *mot = new GroupBox(reglages_groupbox->NewRow(), "Motores");

    GroupBox *reglages_groupbox_AC = new GroupBox(position_AC, name);
    GroupBox *actorBox_LP = new GroupBox(reglages_groupbox_AC->NewRow(), "Actor LP");
    GroupBox *criticBox_LP = new GroupBox(reglages_groupbox_AC->NewRow(), "Critic LP");

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

    Va = new DoubleSpinBox *[40];

    for (unsigned int i = 0; i < 40; i++){
	    unsigned int columns = 10;
        LayoutPosition *position;
		if (i % columns == 0) {
            position = actorBox_LP->NewRow();
		}else{
            position = actorBox_LP->LastRowLastCol();
		}
		Va[i] = new DoubleSpinBox(position, "Va " + std::to_string(i+1), -100000, 100000,1,2);
	}

    Wa = new DoubleSpinBox *[30];
    for (unsigned int i = 0; i < 30; i++){
	    unsigned int columns = 10;
        LayoutPosition *position;
		if (i % columns == 0) {
            position = actorBox_LP->NewRow();
		}else{
            position = actorBox_LP->LastRowLastCol();
		}
		Wa[i] = new DoubleSpinBox(position, "Wa " + std::to_string(i+1), -100000, 100000,1,2);
	}

    G = new DoubleSpinBox *[100];
    G[0] = new DoubleSpinBox(actorBox_LP->NewRow(), "Gamma ", -100000, 100000,1,2);

    // for (unsigned int i = 0; i < 100; i++){
    //     unsigned int columns = 10;
    //     LayoutPosition *position;
	// 	if (i % columns == 0) {
    //         position = actorBox_LP->NewRow();
	// 	}else{
    //         position = actorBox_LP->LastRowLastCol();
	// 	}
	// 	G[i] = new DoubleSpinBox(position, "G " + std::to_string(i+1), -100000, 100000,1,2);
	// }

    Vc = new DoubleSpinBox *[40];
    for (unsigned int i = 0; i < 40; i++){
        unsigned int columns = 10;
        LayoutPosition *position;
		if (i % columns == 0) {
            position = criticBox_LP->NewRow();
		}else{
            position = criticBox_LP->LastRowLastCol();
		}
		Vc[i] = new DoubleSpinBox(position, "Vc " + std::to_string(i+1), -100000, 100000,1,2);
	}

    Wc = new DoubleSpinBox *[10];
    for (unsigned int i = 0; i < 10; i++){
	    unsigned int columns = 10;
        LayoutPosition *position;
		if (i % columns == 0) {
            position = criticBox_LP->NewRow();
		}else{
            position = criticBox_LP->LastRowLastCol();
		}
		Wc[i] = new DoubleSpinBox(position, "Wc " + std::to_string(i+1), -100000, 100000,1,2);
	}

    Q = new DoubleSpinBox(criticBox_LP->NewRow(),"Q",-200000,200000,1,3);
    P = new DoubleSpinBox(criticBox_LP->LastRowLastCol(),"P",-200000,200000,1,3);
    Psi = new DoubleSpinBox(criticBox_LP->LastRowLastCol(),"Psi",-200000,200000,1,3);
    K = new DoubleSpinBox(criticBox_LP->LastRowLastCol(),"K",-200000,200000,1,3);
    Kw = new DoubleSpinBox(criticBox_LP->LastRowLastCol(),"Kw",-200000,200000,1,3);

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;

    sgnori_p << 0,0,0;
    sgnori << 0,0,0;

    Yr = new Actor_tauContribution_LP();

    pert = new CheckBox(num->LastRowLastCol(), "Perturbacion");
    pert_g = new DoubleSpinBox(num->LastRowLastCol(), "Gain Pert:", 0, 1, 0.1);

    AddDataToLog(state);
    AddDataToLog(wa);
    AddDataToLog(wc);
    
    
}

Sliding_LP::~Sliding_LP(void) {}

void Sliding_LP::Reset(void) {
    first_update = true;
    t0 = 0;
    t0 = double(GetTime())/1000000000;
    sgnori_p << 0,0,0;
    sgnori << 0,0,0;

    Eigen::Matrix<float,10,3> Wa_0;
    Eigen::Matrix<float,10,1> Wc_0;

    Wa_0 << Wa[0]->Value(),Wa[1]->Value(),Wa[2]->Value(),Wa[3]->Value(),Wa[4]->Value(),Wa[5]->Value(),Wa[6]->Value(),Wa[7]->Value(),Wa[8]->Value(),
            Wa[9]->Value(),Wa[10]->Value(),Wa[11]->Value(),Wa[12]->Value(),Wa[13]->Value(),Wa[14]->Value(),Wa[15]->Value(),Wa[16]->Value(),Wa[17]->Value(),
            Wa[18]->Value(),Wa[19]->Value(),Wa[20]->Value(),Wa[21]->Value(),Wa[22]->Value(),Wa[23]->Value(),Wa[24]->Value(),Wa[25]->Value(),Wa[26]->Value(),
            Wa[27]->Value(),Wa[28]->Value(),Wa[29]->Value();

    Wc_0 << Wc[0]->Value(),Wc[1]->Value(),Wc[2]->Value(),Wc[3]->Value(),Wc[4]->Value(),Wc[5]->Value(),Wc[6]->Value(),Wc[7]->Value(),Wc[8]->Value(),
            Wc[9]->Value();

    Yr->reset(Wa_0, Wc_0);

    levant.Reset();
    //kd_var->forgetDamping();

    // sgnpos2 = Vector3ff(0,0,0);
    // sgn2 = Vector3ff(0,0,0);

    // sgnpos << 0,0,0;
    // sgn << 0,0,0;

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;


//    pimpl_->i = 0;
//    pimpl_->first_update = true;
}

void Sliding_LP::SetValues(Vector3Df xie, Vector3Df xiep, Vector3Df xid, Vector3Df xidpp, Vector3Df xidppp, Vector3Df w, Quaternion q, 
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

void Sliding_LP::UseDefaultPlot(const LayoutPosition *position) {
    DataPlot1D *rollg = new DataPlot1D(position, "u_roll", -1, 1);
    rollg->AddCurve(state->Element(0));
    
}

void Sliding_LP::UseDefaultPlot2(const LayoutPosition *position) {
    DataPlot1D *pitchg = new DataPlot1D(position, "u_pitch", -1, 1);
    pitchg->AddCurve(state->Element(1));
    
}

void Sliding_LP::UseDefaultPlot3(const LayoutPosition *position) {
    DataPlot1D *yawg = new DataPlot1D(position, "u_yaw", -1, 1);
    yawg->AddCurve(state->Element(2));
    
}

void Sliding_LP::UseDefaultPlot4(const LayoutPosition *position) {    
    DataPlot1D *uz = new DataPlot1D(position, "u_z", -1, 1);
    uz->AddCurve(state->Element(3));
    
}

void Sliding_LP::UseDefaultPlot5(const LayoutPosition *position) {    
    DataPlot1D *r = new DataPlot1D(position, "r", -3.14, 3.14);
    r->AddCurve(state->Element(4));
    
}

void Sliding_LP::UseDefaultPlot6(const LayoutPosition *position) {    
    DataPlot1D *p = new DataPlot1D(position, "p", -3.14, 3.14);
    p->AddCurve(state->Element(5));
    
}

void Sliding_LP::UseDefaultPlot7(const LayoutPosition *position) {    
    DataPlot1D *y = new DataPlot1D(position, "y", -3.14, 3.14);
    y->AddCurve(state->Element(6));
    
}

void Sliding_LP::UseDefaultPlot8(const LayoutPosition *position) {    
    DataPlot1D *Sp = new DataPlot1D(position, "nu_rp", -5, 5);
    Sp->AddCurve(state->Element(7), DataPlot::Red);
    Sp->AddCurve(state->Element(8), DataPlot::Green);
    Sp->AddCurve(state->Element(9), DataPlot::Blue);
    
}

void Sliding_LP::UseDefaultPlot9(const LayoutPosition *position) {    
    DataPlot1D *Sq = new DataPlot1D(position, "nu_r", -5, 5);
    Sq->AddCurve(state->Element(10), DataPlot::Green);
    Sq->AddCurve(state->Element(11), DataPlot::Red);
    Sq->AddCurve(state->Element(12), DataPlot::Black);
    
}

void Sliding_LP::UseDefaultPlot10(const LayoutPosition *position) {    
    DataPlot1D *Kd = new DataPlot1D(position, "Yr", -3, 3);
    Kd->AddCurve(state->Element(13), DataPlot::Green);
    Kd->AddCurve(state->Element(14), DataPlot::Red);
    Kd->AddCurve(state->Element(15), DataPlot::Black);
    
}

void Sliding_LP::UseDefaultPlot11(const LayoutPosition *position) {    
    DataPlot1D *tau = new DataPlot1D(position, "tau", -3, 3);
    tau->AddCurve(state->Element(16), DataPlot::Green);
    tau->AddCurve(state->Element(17), DataPlot::Red);
    tau->AddCurve(state->Element(18), DataPlot::Black);
    
}

void Sliding_LP::UseDefaultPlot12(const LayoutPosition *position) {    
    DataPlot1D *tau = new DataPlot1D(position, "tau f", -3, 3);
    tau->AddCurve(state->Element(19), DataPlot::Green);
    tau->AddCurve(state->Element(20), DataPlot::Red);
    tau->AddCurve(state->Element(21), DataPlot::Black);
    
}

void Sliding_LP::UseDefaultPlot13(const LayoutPosition *position) {    
    DataPlot1D *J = new DataPlot1D(position, "IntTD", -5, 5);
    J->AddCurve(state->Element(22), DataPlot::Black);
    //J->AddCurve(state->Element(23), DataPlot::Red);
    //J->AddCurve(state->Element(24), DataPlot::Black);
    
}


void Sliding_LP::UpdateFrom(const io_data *data) {
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
    Eigen::Matrix3f Kdm = Kdv.asDiagonal();

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

    // if(pert->IsChecked()){
    //     q = q*(q_p);
    // }else{
    //     q = q;
    // }

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


    flair::core::Time dt_pos = GetTime() - t0_p;

    //lp->SetText("Latecia pos: %.3f ms",(float)dt_pos/1000000);

    //Yr->setCNN();

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

    //Eigen::Matrix3f Kdm = kd_var->learnDampingInjection(we, qe, qep, nuq, qd, q, qp, qdp, delta_t);

    //printf("Kdm: %f %f %f\n",Kdm(0,0),Kdm(1,1),Kdm(2,2));

    //Eigen::Vector3f reward = kd_var->getR();

    //Eigen::Vector3f ec = kd_var->getEc();

    //Eigen::Vector3f J = kd_var->getJ();

    //Eigen::Matrix3f psi = kd_var->getPsi();

    Eigen::Matrix<float,4,10> Vam;
    
    Vam << Va[0]->Value(),Va[1]->Value(),Va[2]->Value(),Va[3]->Value(),Va[4]->Value(),Va[5]->Value(),Va[6]->Value(),Va[7]->Value(),Va[8]->Value(),
            Va[9]->Value(),Va[10]->Value(),Va[11]->Value(),Va[12]->Value(),Va[13]->Value(),Va[14]->Value(),Va[15]->Value(),Va[16]->Value(),Va[17]->Value(),
            Va[18]->Value(),Va[19]->Value(),Va[20]->Value(),Va[21]->Value(),Va[22]->Value(),Va[23]->Value(),Va[24]->Value(),Va[25]->Value(),Va[26]->Value(),
            Va[27]->Value(),Va[28]->Value(),Va[29]->Value(),Va[30]->Value(),Va[31]->Value(),Va[32]->Value(),Va[33]->Value(),Va[34]->Value(),Va[35]->Value(),
            Va[36]->Value(),Va[37]->Value(),Va[38]->Value(),Va[39]->Value();

    // int j=0, i2=0;
    // for(unsigned int i; i<40; i++){
    //     unsigned int columns = 10;
    //     Vam(j,i2-1) = (float)Va[i]->Value();
    //     if (i % columns == 0) {
    //         j++;
    //         i2 = 0;
	// 	}else{
    //         i2++;
    //     }
	// }

    Eigen::Matrix<float,4,10> Vcm;

    Vcm << Vc[0]->Value(),Vc[1]->Value(),Vc[2]->Value(),Vc[3]->Value(),Vc[4]->Value(),Vc[5]->Value(),Vc[6]->Value(),Vc[7]->Value(),Vc[8]->Value(),
            Vc[9]->Value(),Vc[10]->Value(),Vc[11]->Value(),Vc[12]->Value(),Vc[13]->Value(),Vc[14]->Value(),Vc[15]->Value(),Vc[16]->Value(),Vc[17]->Value(),
            Vc[18]->Value(),Vc[19]->Value(),Vc[20]->Value(),Vc[21]->Value(),Vc[22]->Value(),Vc[23]->Value(),Vc[24]->Value(),Vc[25]->Value(),Vc[26]->Value(),
            Vc[27]->Value(),Vc[28]->Value(),Vc[29]->Value(),Vc[30]->Value(),Vc[31]->Value(),Vc[32]->Value(),Vc[33]->Value(),Vc[34]->Value(),Vc[35]->Value(),
            Vc[36]->Value(),Vc[37]->Value(),Vc[38]->Value(),Vc[39]->Value();

    // j=0, i2=0;
    // for(unsigned int i; i<40; i++){
    //     unsigned int columns = 10;
    //     Vcm(j,i2-1) = (float)Vc[i]->Value();
    //     if (i % columns == 0) {
    //         j++;
    //         i2 = 0;
	// 	}else{
    //         i2++;
    //     }
	// }

    Eigen::Matrix<float,10,10> GAMMA_a2 = Eigen::Matrix<float,10,10>::Identity();
    // j=0, i2=0;
    // for(unsigned int i; i<100; i++){
    //     unsigned int columns = 10;
    //     GAMMA_a2(j,i2-1) = (float)G[i]->Value();
    //     if (i % columns == 0) {
    //         j++;
    //         i2 = 0;
	// 	}else{
    //         i2++;
    //     }
	// }


    Yr->setANN(nur,Vam,GAMMA_a2*G[0]->Value(),delta_t);
    Yr->setCNN(qe,qep,Q->Value(),P->Value(),Vcm,Psi->Value(),K->Value(),Kw->Value());
    
    Yr->ActorCritic_Compute(delta_t);

    Eigen::Vector3f Yrv = Yr->YrOutput();

    float gamma_hat = Yr->gamma_hat;
    float r = Yr->getReward();

    Eigen::Matrix<float,10,3> wa_m = Yr->getActorWeights();
    Eigen::Matrix<float,10,1> wc_m = Yr->getCriticWeights();

    Eigen::Vector3f tau2 = -Kdm*nur;

    Eigen::Vector3f tau = tau2 + Yrv;

    flair::core::Time dt_ori = GetTime() - t0_o;

    //lo->SetText("Latecia ori: %.3f ms",(float)dt_ori/1000000);

    float J_hat = Yr->getApproximateValueFnc();
    float J = Yr->getValueFnc(tactual,delta_t);

    
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
    state->SetValueNoMutex(13, 0, Yrv(0));
    state->SetValueNoMutex(14, 0, Yrv(1));
    state->SetValueNoMutex(15, 0, Yrv(2));
    state->SetValueNoMutex(16, 0, tau2(0));
    state->SetValueNoMutex(17, 0, tau2(1));
    state->SetValueNoMutex(18, 0, tau2(2));
    state->SetValueNoMutex(19, 0, tau(0));
    state->SetValueNoMutex(20, 0, tau(1));
    state->SetValueNoMutex(21, 0, tau(2));
    state->SetValueNoMutex(22, 0, gamma_hat);
    state->SetValueNoMutex(23, 0, r);
    state->SetValueNoMutex(24, 0, J);
    state->SetValueNoMutex(25, 0, battery);
    state->SetValueNoMutex(26, 0, nur(0));
    state->SetValueNoMutex(27, 0, nur(1));
    state->SetValueNoMutex(28, 0, nur(2));
    state->SetValueNoMutex(29, 0, J_hat);
    //state->SetDataTime(data->DataTime());
    state->ReleaseMutex();

    wa->GetMutex();
    wa->SetValueNoMutex(0,0,wa_m(0,0));
    wa->SetValueNoMutex(1,0,wa_m(1,0));
    wa->SetValueNoMutex(2,0,wa_m(2,0));
    wa->SetValueNoMutex(3,0,wa_m(3,0));
    wa->SetValueNoMutex(4,0,wa_m(4,0));
    wa->SetValueNoMutex(5,0,wa_m(5,0));
    wa->SetValueNoMutex(6,0,wa_m(6,0));
    wa->SetValueNoMutex(7,0,wa_m(7,0));
    wa->SetValueNoMutex(8,0,wa_m(8,0));
    wa->SetValueNoMutex(9,0,wa_m(9,0));

    wa->SetValueNoMutex(0,1,wa_m(0,1));
    wa->SetValueNoMutex(1,1,wa_m(1,1));
    wa->SetValueNoMutex(2,1,wa_m(2,1));
    wa->SetValueNoMutex(3,1,wa_m(3,1));
    wa->SetValueNoMutex(4,1,wa_m(4,1));
    wa->SetValueNoMutex(5,1,wa_m(5,1));
    wa->SetValueNoMutex(6,1,wa_m(6,1));
    wa->SetValueNoMutex(7,1,wa_m(7,1));
    wa->SetValueNoMutex(8,1,wa_m(8,1));
    wa->SetValueNoMutex(9,1,wa_m(9,1));

    wa->SetValueNoMutex(0,2,wa_m(0,2));
    wa->SetValueNoMutex(1,2,wa_m(1,2));
    wa->SetValueNoMutex(2,2,wa_m(2,2));
    wa->SetValueNoMutex(3,2,wa_m(3,2));
    wa->SetValueNoMutex(4,2,wa_m(4,2));
    wa->SetValueNoMutex(5,2,wa_m(5,2));
    wa->SetValueNoMutex(6,2,wa_m(6,2));
    wa->SetValueNoMutex(7,2,wa_m(7,2));
    wa->SetValueNoMutex(8,2,wa_m(8,2));
    wa->SetValueNoMutex(9,2,wa_m(9,2));
    wa->ReleaseMutex();

    wc->GetMutex();
    wc->SetValueNoMutex(0,0,wc_m(0,0));
    wc->SetValueNoMutex(1,0,wc_m(1,0));
    wc->SetValueNoMutex(2,0,wc_m(2,0));
    wc->SetValueNoMutex(3,0,wc_m(3,0));
    wc->SetValueNoMutex(4,0,wc_m(4,0));
    wc->SetValueNoMutex(5,0,wc_m(5,0));
    wc->SetValueNoMutex(6,0,wc_m(6,0));
    wc->SetValueNoMutex(7,0,wc_m(7,0));
    wc->SetValueNoMutex(8,0,wc_m(8,0));
    wc->SetValueNoMutex(9,0,wc_m(9,0));
    wc->ReleaseMutex();



    output->SetValue(0, 0, tau_roll);
    output->SetValue(1, 0, tau_pitch);
    output->SetValue(2, 0, tau_yaw);
    output->SetValue(3, 0, Tr);
    output->SetDataTime(data->DataTime());
    
    ProcessUpdate(output);
    //ProcessUpdate(state);
    
}

float Sliding_LP::Sat(float value, float borne) {
    if (value < -borne)
        return -borne;
    if (value > borne)
        return borne;
    return value;
}

float Sliding_LP::sech(float value) {
    return 1 / coshf(value);
}

} // end namespace filter
} // end namespace flair
