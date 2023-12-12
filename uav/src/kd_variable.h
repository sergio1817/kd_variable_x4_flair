//  created:    2011/05/01
//  filename:   kd_variable.h
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#ifndef PROYECTO22_H
#define PROYECTO22_H

#include <UavStateMachine.h>
#include "Sliding.h"
#include "Sliding_kdvar.h"
#include "Sliding_pos.h"
#include "Sliding_force.h"
#include "Sliding_kdvar_h.h"
#include "Sliding_LP.h"

namespace flair {
    namespace gui {
        class PushButton;
        class ComboBox;
        class Tab;
        class TabWidget;
        class DoubleSpinBox;
        class GroupBox;
        class Label;
    }
    namespace filter {
        class ControlLaw;
        class Sliding;
        class Sliding_pos;
        class Sliding_force;
        class Sliding_kdvar;
        class Sliding_kdvar_h;
        class Sliding_LP;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
        class TargetJR3;
        class BatteryMonitor;
    }
}

class kd_variable : public flair::meta::UavStateMachine {
    public:
        kd_variable(flair::sensor::TargetController *controller);
        ~kd_variable();

    private:

	enum class BehaviourMode_t {
            Default,
            control
        }clTabCtrl;

        BehaviourMode_t behaviourMode;
        bool vrpnLost;
        flair::sensor::TargetJR3 *jr3;

        void ExtraCheckPushButton(void);
        void ExtraCheckJoystick(void);
        void SignalEvent(Event_t event);
        void Startkd_variable(void);
        void Stopkd_variable(void);
        void ExtraSecurityCheck(void);
        void ComputeCustomTorques(flair::core::Euler &torques);
        float ComputeCustomThrust(void);
        void sliding_ctrl(flair::core::Euler &torques);
        void sliding_ctrl_pos(flair::core::Euler &torques);
        void sliding_kdvar_ctrl(flair::core::Euler &torques);
        void sliding_ctrl_force(flair::core::Euler &torques);
        void sliding_ctrl_kdvar_h(flair::core::Euler &torques);
        void sliding_ctrl_LP_h(flair::core::Euler &torques);
        //const flair::core::AhrsData *GetOrientation(void) const;
        void pos_reference(flair::core::Vector3Df &xid, flair::core::Vector3Df &xidp, flair::core::Vector3Df &xidpp, flair::core::Vector3Df &xidppp, float tactual);

        flair::filter::Sliding *u_sliding;
        flair::filter::Sliding_kdvar *u_sliding_kdvar;
        flair::filter::Sliding_pos *u_sliding_pos;
        flair::filter::Sliding_force *u_sliding_force;
        flair::filter::Sliding_kdvar_h* u_sliding_kdvar_h;
        flair::filter::Sliding_LP* u_sliding_LP;

        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        
        //bool first_update;

        flair::sensor::BatteryMonitor *battery;

        float thrust;


        flair::gui::DoubleSpinBox *xd, *yd, *zd, *ax, *wx, *bx, *ay, *wy, *by, *az, *wz, *bz;
        flair::gui::DoubleSpinBox *Lambda_phi, *Lambda_th, *Lambda_psi, *Gamma_c, *gamma, *p, *goal, *alph_l, *lamb_l;

        flair::gui::PushButton *start_prueba1,*stop_prueba1;
        flair::gui::ComboBox *control_select, *position_behavior, *xd_behavior, *yd_behavior, *zd_behavior;    
        flair::gui::Tab *setupLawTab2, *graphLawTab2, *lawTab2, *setupLawTab3, *graphLawTab3, *positionTab, *positiongTab, *setupLawTabAC, *setupLawTabACp, *setupLawKDvar, *graphLawKDvar, *setuoLawKDvarh, *graphLawKDvarh, *setupLawTabACph;
        flair::gui::TabWidget *tabWidget2, *Pos_tabWidget;
        flair::gui::GroupBox *seg;
        flair::gui::Label *l, *l2, *lx, *ly, *lz;

        flair::core::Vector3Df b;

        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
        
};

#endif // PROYECTO22_H
