/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <systemlib/param/param.h>

/**
 * Sensibilidad throtle en m/s2
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Throtle_sens, 5.0f);

/**
 * Sensibildiad ginada en DEG
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Yaw_sens, 20.0f);

/**
 * Sensibilidad de cabeceo y balanceo en DEG
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Atti_sens, 10.0f);

/**
 * Tiempo de respuesta PHI
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_phi_tau, 0.7f);
/**
 * Ganancia control PHI
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_phi_K_b, 3.0f);

/**
 * Integral control PHI
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_phi_f_i, 0.2f);


/**
 * Tiempo de respuesta THETA
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_theta_tau, 0.7f);
/**
 * Ganancia control THETA
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_theta_K_b, 3.0f);

/**
 * Integral control THETA
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_theta_f_i, 0.2f);

/**
 * Tiempo de respuesta PSI
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_psi_tau, 1.5f);
/**
 * Ganancia control PSI
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_psi_K_b, 2.5f);

/**
 * Integral control PSI
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_psi_f_i, 0.1f);

/**
 * Tiempo respuesta control P
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_p_tau, 0.3f);

/**
 * Ganancia control P
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_p_K_b, 6.0f);

/**
 * Tiempo respuesta control Q
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_q_tau, 0.3f);

/**
 * Ganancia control Q
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_q_K_b, 6.0f);

/**
 * Tiempo respuesta control R
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_r_tau, 0.5f);

/**
 * Ganancia control R
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_r_K_b, 5.0f);


/**
 * Deflexion Flaps en DEG
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Flaps_deg, 0.0f);