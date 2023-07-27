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
#include "Sliding_pos.h"
#include "Sliding_force.h"

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
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
        class TargetJR3;
    }
}

class kd_variable : public flair::meta::UavStateMachine {
    public:
        kd_variable(flair::sensor::TargetController *controller, flair::sensor::TargetJR3 *sensor);
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
        void sliding_ctrl_force(flair::core::Euler &torques);
        //const flair::core::AhrsData *GetOrientation(void) const;
        void pos_reference(flair::core::Vector3Df &xid, flair::core::Vector3Df &xidp, flair::core::Vector3Df &xidpp, flair::core::Vector3Df &xidppp, float tactual);

        flair::filter::Sliding *u_sliding;
        flair::filter::Sliding_pos *u_sliding_pos;
        flair::filter::Sliding_force *u_sliding_force;

        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        
        //bool first_update;

        float thrust;

        flair::gui::DoubleSpinBox *xd, *yd, *zd, *ax, *wx, *bx, *ay, *wy, *by, *az, *wz, *bz;

        flair::gui::PushButton *start_prueba1,*stop_prueba1;
        flair::gui::ComboBox *control_select, *position_behavior, *xd_behavior, *yd_behavior, *zd_behavior;    
        flair::gui::Tab *setupLawTab2, *graphLawTab2, *lawTab2, *setupLawTab3, *graphLawTab3, *positionTab, *positiongTab;
        flair::gui::TabWidget *tabWidget2, *Pos_tabWidget;
        flair::gui::GroupBox *seg;
        flair::gui::Label *l, *l2, *lx, *ly, *lz;

        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
        
};

#endif // PROYECTO22_H
