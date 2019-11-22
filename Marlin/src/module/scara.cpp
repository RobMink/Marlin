/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * scara.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_SCARA

#include "scara.h"
#include "motion.h"
#include "planner.h"

float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND;

void scara_set_axis_is_at_home(const AxisEnum axis) {

//SERIAL_ECHOLNPAIR("in scarasetaxis");
  if (axis == Z_AXIS)
    current_position.z = Z_HOME_POS;
  else {

    /**
     * SCARA homes XY at the same time
     */
    xyz_pos_t homeposition;
    //LOOP_XYZ(i) homeposition[i] = base_home_pos((AxisEnum)i);
homeposition[X_AXIS]=X_MAX_POS;
homeposition[Y_AXIS]=0;
homeposition[Z_AXIS]=Z_MAX_POS;

    // SERIAL_ECHOLNPAIR("homeposition X:", homeposition.x, " Y:", homeposition.y);

    /**
     * Get Home position SCARA arm angles using inverse kinematics,
     * and calculate homing offset using forward kinematics
     */
    inverse_kinematics(homeposition);
    forward_kinematics_SCARA(delta.a, delta.b);

    SERIAL_ECHOLNPAIR("Cartesian X:", cartes.x, " Y:", cartes.y);

    current_position[axis] = cartes[axis];

    update_software_endstops(axis);

  //  SERIAL_ECHOLNPAIR("fin");
  }

    current_position.y = 0;
      current_position.x = X_MAX_POS;
}

//static constexpr xy_pos_t scara_offset = { SCARA_OFFSET_X, SCARA_OFFSET_Y };

/**
 * Morgan SCARA Forward Kinematics. Results in 'cartes'.
 * Maths and first version by QHARLEY.
 * Integrated into Marlin and slightly restructured by Joachim Cerny.
 */
void forward_kinematics_SCARA(const float &a, const float &b) {
  //delta[X_AXIS] =f_scara[Y_AXIS]*cos((f_scara[X_AXIS]* 71) / 4068.0);  //theta
  //  delta[Y_AXIS] =  f_scara[Y_AXIS]*sin((f_scara[X_AXIS]* 71) / 4068.0);

  const float th =RADIANS(a)*cos((RADIANS(b)* 71) / 4068.0);  //theta
  const float ra =  (RADIANS(b)*sin((RADIANS(a)* 71) / 4068.0))  ;

  /*const float a_sin = sin(RADIANS(a)) * L1,
              a_cos = cos(RADIANS(a)) * L1,
              b_sin = sin(RADIANS(b)) * L2,
              b_cos = cos(RADIANS(b)) * L2;
*/
  cartes.set(th,  // theta
             ra); // theta+phi
  /*
    SERIAL_ECHOLNPAIR(
      "SCARA FK Angle a=", a,
      " b=", b,
      " a_sin=", a_sin,
      " a_cos=", a_cos,
      " b_sin=", b_sin,
      " b_cos=", b_cos
    );
    SERIAL_ECHOLNPAIR(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")");
  //*/
}

/**
 * Morgan SCARA Inverse Kinematics. Results in 'delta'.
 *
 * See http://forums.reprap.org/read.php?185,283327
 *
 * Maths and first version by QHARLEY.
 * Integrated into Marlin and slightly restructured by Joachim Cerny.
 */
void inverse_kinematics(const xyz_pos_t &raw) {

  float C2, S2, SK1, SK2, THETA, PSI;


  // Translate SCARA to standard XY with scaling factor
  const xy_pos_t spos = raw;
/*
  const float H2 = HYPOT2(spos.x, spos.y);
  if (L1 == L2)
    C2 = H2 / L1_2_2 - 1;
  else
    C2 = (H2 - (L1_2 + L2_2)) / (2.0 * L1 * L2);

  S2 = SQRT(1.0f - sq(C2));

  // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
  SK1 = L1 + L2 * C2;

  // Rotated Arm2 gives the distance from Arm1 to Arm2
  SK2 = L2 * S2;

  // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
  THETA = ATAN2(SK1, SK2) - ATAN2(spos.x, spos.y);
*/
  // Angle of Arm2
  //  PSI = ATAN2(S2, C2);
  static float rad, thea ;

      float xraw = spos.x,  // Translate SCARA to standard X Y
            yraw= spos.y;  // With scaling factor.
    rad=(sqrt(sq(xraw)+sq(yraw))/57.2958) ;
    thea=float(atan2(yraw, xraw));
  delta.set(DEGREES(thea), DEGREES(rad), raw.z);

  /*
    DEBUG_POS("SCARA IK", raw);
    DEBUG_POS("SCARA IK", delta);
    SERIAL_ECHOLNPAIR("  SCARA (x,y) ", sx, ",", sy, " C2=", C2, " S2=", S2, " Theta=", THETA, " Phi=", PHI);
  //*/
}

void scara_report_positions() {
  SERIAL_ECHOLNPAIR("SCARA Theta:", planner.get_axis_position_degrees(A_AXIS), "  Psi+Theta:", planner.get_axis_position_degrees(B_AXIS));
  SERIAL_EOL();
}

#endif // IS_SCARA
