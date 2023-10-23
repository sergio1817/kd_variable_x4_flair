// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file Slifing_pos.h
 * \brief Class defining a Sliding mode controller for position
 * \author Sergio Urzua, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2023/05/12
 * \version 1.0
 */

#ifndef SLIDING_KDVAR_H_H
#define SLIDING_KDVAR_H_H

#include <Object.h>
#include "NMethods.h"
#include "DILWAC.h"
#include <ControlLaw.h>
#include <Eigen/Dense>
#include <Vector3D.h>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
    }
}

/*! \class Sliding_kdvar_h
* \brief Class defining a Sliding_kdvar_h
*/

    
    
namespace flair {
    namespace filter {
    /*! \class Sliding_kdvar_h
    *
    * \brief Class defining a Sliding_kdvar_h
    */
        class Sliding_kdvar_h : public ControlLaw {
    
    
public:
    Sliding_kdvar_h(const flair::gui::LayoutPosition *position, std::string name);
    ~Sliding_kdvar_h();
    void UpdateFrom(const flair::core::io_data *data);
    void Reset(void);
    
    /*!
  * \brief Set input values
  *
  * \param xie Error de posicion
  * \param xiep Error de velocidad
  * \param xid  Posicion deseada
  * \param xidpp Aceleracion deseada
  * \param xidppp Jerk deseado
  * \param w Velocidad angular
  * \param q Cuaternio de orientacion
  */
    void SetValues(flair::core::Vector3Df xie, flair::core::Vector3Df xiep, flair::core::Vector3Df xid, 
                    flair::core::Vector3Df xidpp, flair::core::Vector3Df xidppp, flair::core::Vector3Df w, flair::core::Quaternion q,
                    flair::core::Vector3Df Lambda, flair::core::Vector3Df GammaC, int gamma, int p, float goal, float alph_l, float lamb_l);
    
    void UseDefaultPlot(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot2(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot3(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot4(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot5(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot6(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot7(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot8(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot9(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot10(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot11(const flair::gui::LayoutPosition *position);

    
    
    flair::core::Time t0;
    flair::core::Matrix *state;

private:
    
    Levant_diff levant;

    float sech(float value);

    flair::gui::CheckBox *levantd;
    flair::gui::DoubleSpinBox *T, *gamma, *k, *sat_r, *sat_p, *sat_y, *sat_t, *m, *g, *km, *p, *km_z;
    flair::gui::DoubleSpinBox *gamma_roll, *gamma_pitch, *gamma_yaw, *gamma_x, *gamma_y, *gamma_z;
    flair::gui::DoubleSpinBox *alpha_roll, *alpha_pitch, *alpha_yaw, *alpha_x, *alpha_y, *alpha_z;
    flair::gui::DoubleSpinBox *Kd_roll, *Kd_pitch, *Kd_yaw, *Kd_x, *Kd_y, *Kd_z;
    flair::gui::DoubleSpinBox *Kp_roll, *Kp_pitch, *Kp_yaw, *Kp_x, *Kp_y, *Kp_z;
    flair::gui::DoubleSpinBox *alpha_l,*lamb_l;

    flair::gui::Label *lo, *lp;
    
    float Sat(float value, float borne);
    
    float delta_t;
    
    bool first_update;
    
    Eigen::Vector3f sgnpos_p, sgnpos, sgnori_p, sgnori;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3,3);

    DILWAC *kd_var;

    //flair::core::Vector3ff sgnori_p, sgnori;
    
    
};
} // end namespace filter
} // end namespace flair

#endif // LINEAR_H
