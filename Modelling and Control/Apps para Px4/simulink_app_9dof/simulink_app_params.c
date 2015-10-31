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

/**
 * Parameters defined by Simulink_app
 *
 */
 

#include <nuttx/config.h>
#include <systemlib/param/param.h>

PARAM_DEFINE_FLOAT(3C_X_COM, 0.0f);
PARAM_DEFINE_FLOAT(3C_Y_COM, 0.0f);
PARAM_DEFINE_FLOAT(3C_ALT_COM, 0.0f);

PARAM_DEFINE_FLOAT(3C_VX_YAW_COM, 0.0f);
PARAM_DEFINE_FLOAT(3C_VY_YAW_COM, 0.0f);
PARAM_DEFINE_FLOAT(3C_VZ_YAW_COM, 0.0f);

PARAM_DEFINE_FLOAT(3C_PHI_COM, 0.0f);
PARAM_DEFINE_FLOAT(3C_THETA_COM, 0.0f);
PARAM_DEFINE_FLOAT(3C_HEADING_COMD, 0.0f);

PARAM_DEFINE_INT32(3C_FLIGHT_MOD_REQT, 0);





