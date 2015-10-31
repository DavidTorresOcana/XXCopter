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
 * Proportional gain of altitude control
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Kp_h, 0.1f);

/**
 * Integral gain of altitude control
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Ki_h, 0.05f);

/**
 * Derivative gain of altitude control
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Kd_h, 0.05f);

/**
 * Proportional gain of TAS controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Kp_u, 0.1f);
/**
 * Integral gain of TAS controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Ki_u, 0.03f);

/**
 * Proportional gain of theta controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Kp_theta, 3.0f);
/**
 * Integral gain of theta controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Ki_theta, 0.1f);

/**
 * Proportional gain of phi controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Kp_phi, 10.0f);

/**
 * Integral gain of phi controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Ki_phi, 0.5f);

/**
 * Proportional gain of pith rate controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Kp_q, 0.07f);

/**
 * Integral gain of pith rate controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Ki_q , 0.01f);

/**
 * Proportional gain of roll rate controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Kp_p, 0.01f);

/**
 * Integral gain of roll rate controller
 *
 * RE-CHECK this.
 *
 * @min 
 * @max 
 * @
 */
PARAM_DEFINE_FLOAT(DBX_Ki_p , 0.0f);
