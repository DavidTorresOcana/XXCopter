// Simulink wrapper code for PX4 (PX4FMU & Pixhawk)

#include <nuttx/config.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/airspeed.h>

#include <uORB/topics/vehicle_local_position.h>


#include <drivers/drv_led.h>
#include <drivers/drv_rgbled.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>

// Include the generated files and code of Simulink: Autopilot
#include "dbx_control_ert_rtw/dbx_control.c"
#include "dbx_control_ert_rtw/dbx_control_data.c" // Aqui se cargan los datos y constantes con que se compila Similink
#include "dbx_control_params.c"
//

__EXPORT int dbx_control_main(int argc, char *argv[]);

const char *dev_rgbled = RGBLED0_DEVICE_PATH;
const char *dev_pwm = PWM_OUTPUT0_DEVICE_PATH;

static int simulink_task;
static bool thread_exit;
static bool pwm_enabled;

struct rgbled_rgbset_t{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

int step_size = 10; // fundamental sample time (ms). 100Hz
int i = 1;

// GCS parameters
struct {
    float Kp_h;
    float Ki_h;
    float Kd_h;
    float Kp_u;
    float Ki_u;
    float Kp_theta;
    float Ki_theta;
    float Kp_phi;
    float Ki_phi;
    float Kp_q;
    float Ki_q;
    float Kp_p;
    float Ki_p;	
    }		GCS_parameters;

int simulink_main(int argc, char *argv[])
{
  dbx_control_initialize();

  // declare data subscriptions
  int sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
  int pwm_inputs_sub = orb_subscribe(ORB_ID(input_rc));
  int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
  int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
  int bat_status_sub = orb_subscribe(ORB_ID(battery_status));
  int airspeed_sub = orb_subscribe(ORB_ID(airspeed));

  int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    
//   int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
//   int pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
//   int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
//   int global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

    
  // Declare data structs from subcriptions
  struct sensor_combined_s 				sensors;
  struct rc_input_values 				pwm_inputs;
  struct vehicle_attitude_s 			attitude;
  struct vehicle_gps_position_s 		gps;
  struct battery_status_s               bat_status;
  struct airspeed_s                     airspeed;
  struct vehicle_local_position_s 		local_pos;              /*< vehicle local position */

  // Declare pointers to GCS params
	struct {
	   param_t Kp_h;
	   param_t Ki_h;
	   param_t Kd_h;
	   
	   param_t Kp_u;
	   param_t Ki_u;
	   
	   param_t Kp_theta;
	   param_t Ki_theta;
	   
	   param_t Kp_phi;
	   param_t Ki_phi;

	   param_t Kp_q;
	   param_t Ki_q;
	   
	   param_t Kp_p;
	   param_t Ki_p;	   
	}	GCS_comms_pointers; 
    
    // Get the pointers to GCS params
	GCS_comms_pointers.Kp_h = param_find("DBX_Kp_h");
	GCS_comms_pointers.Ki_h = param_find("DBX_Ki_h");
	GCS_comms_pointers.Kd_h = param_find("DBX_Kd_h");

	GCS_comms_pointers.Kp_u = param_find("DBX_Kp_u");
	GCS_comms_pointers.Ki_u = param_find("DBX_Ki_u");
	
	GCS_comms_pointers.Kp_theta = param_find("DBX_Kp_theta");
	GCS_comms_pointers.Ki_theta = param_find("DBX_Ki_theta");
	
	GCS_comms_pointers.Kp_phi = param_find("DBX_Kp_phi");
	GCS_comms_pointers.Ki_phi = param_find("DBX_Ki_phi");

	GCS_comms_pointers.Kp_q = param_find("DBX_Kp_q"); 
	GCS_comms_pointers.Ki_q = param_find("DBX_Ki_q"); 
	
	GCS_comms_pointers.Kp_p = param_find("DBX_Kp_p"); 
	GCS_comms_pointers.Ki_p = param_find("DBX_Ki_p"); 	

	
  // Limiting the update rate
  orb_set_interval(sensors_sub, step_size);
  orb_set_interval(attitude_sub, step_size); 
  orb_set_interval(airspeed_sub, step_size); 

  // declare output devices
  int rgbled = open(dev_rgbled, 0);
  int pwm = open(dev_pwm, 0);
  // Declare here the other AUX PWM outputs???
    
  // initialize outputs
  ioctl(rgbled, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_ON);
  ioctl(pwm, PWM_SERVO_SET_ARM_OK, 0);
  ioctl(pwm, PWM_SERVO_ARM, 0);
  pwm_enabled = 0;

  struct pollfd fds[] = {
    { .fd = sensors_sub, .events = POLLIN },
  };

  // primary application thread
  while (!thread_exit) {
    int poll_return = poll(fds, 1, 1000);
    if (poll_return > 0) {
      if (fds[0].revents & POLLIN) {
        // assign sensor data
        orb_copy(ORB_ID(sensor_combined), sensors_sub, &sensors);
        orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude);
        orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);   
        orb_copy(ORB_ID(input_rc), pwm_inputs_sub, &pwm_inputs);      
        orb_copy(ORB_ID(battery_status), bat_status_sub, &bat_status);
        orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed);
		orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);

        dbx_control_U.runtime = hrt_absolute_time(); 
        dbx_control_U.mag_x = sensors.magnetometer_ga[0];
        dbx_control_U.mag_y = sensors.magnetometer_ga[1];
        dbx_control_U.mag_z = sensors.magnetometer_ga[2];
        dbx_control_U.acc_x = sensors.accelerometer_m_s2[0];
        dbx_control_U.acc_y = sensors.accelerometer_m_s2[1];
        dbx_control_U.acc_z = sensors.accelerometer_m_s2[2];
        dbx_control_U.gyro_x = sensors.gyro_rad_s[0];
        dbx_control_U.gyro_y = sensors.gyro_rad_s[1];
        dbx_control_U.gyro_z = sensors.gyro_rad_s[2];
        dbx_control_U.rate_roll = attitude.rollspeed;
        dbx_control_U.rate_pitch = attitude.pitchspeed;
        dbx_control_U.rate_yaw = attitude.yawspeed;
        dbx_control_U.att_roll = attitude.roll;
        dbx_control_U.att_pitch = attitude.pitch;
        dbx_control_U.att_yaw = attitude.yaw;
        dbx_control_U.q0 = attitude.q[0];
        dbx_control_U.q1 = attitude.q[1];
        dbx_control_U.q2 = attitude.q[2];
        dbx_control_U.q3 = attitude.q[3];
        dbx_control_U.baro_alt = sensors.baro_alt_meter;
        dbx_control_U.gps_sat = gps.satellites_used; 
        dbx_control_U.gps_lat = 0.0000001*(double)gps.lat; 
        dbx_control_U.gps_lon = 0.0000001*(double)gps.lon; 
        dbx_control_U.gps_alt = 0.001*(double)gps.alt; 
        dbx_control_U.gps_vel = gps.vel_m_s; 
        dbx_control_U.gps_vel_n = gps.vel_n_m_s; 
        dbx_control_U.gps_vel_e = gps.vel_e_m_s; 
        dbx_control_U.gps_vel_d = gps.vel_d_m_s;
        dbx_control_U.ch1 = pwm_inputs.values[0];
        dbx_control_U.ch2 = pwm_inputs.values[1];
        dbx_control_U.ch3 = pwm_inputs.values[2];
        dbx_control_U.ch4 = pwm_inputs.values[3];
        dbx_control_U.ch5 = pwm_inputs.values[4];
        dbx_control_U.ch6 = pwm_inputs.values[5];
        dbx_control_U.ch7 = pwm_inputs.values[6];
        dbx_control_U.ch8 = pwm_inputs.values[7];    
         
		/*----- Added inputs ---------*/
        dbx_control_U.gps_pdop = gps.eph; // pdop or hdop
        dbx_control_U.gps_vdop = gps.epv; // vdop
        dbx_control_U.bat_volts = bat_status.voltage_filtered_v; // Batery volts
        dbx_control_U.pitot_diff_pre = sensors.differential_pressure_filtered_pa; // Pitot presion dinamica
        dbx_control_U.TAS_mps = airspeed.true_airspeed_m_s; // TAS estimada
        
        if (i < 10) { // 10Hz loop
          i = i++;
        } else {          
          // check arm state
          if (dbx_control_Y.pwm_arm == 1 && pwm_enabled == 0) {
            // arm system
            pwm_enabled = 1;
            printf("\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\n");
          } else if (dbx_control_Y.pwm_arm == 0 && pwm_enabled == 1) {
            // disarm system
            ioctl(pwm, PWM_SERVO_SET(0), 1100);
            ioctl(pwm, PWM_SERVO_SET(1), 1100);
            ioctl(pwm, PWM_SERVO_SET(2), 1970);
            ioctl(pwm, PWM_SERVO_SET(3), 1898);
            ioctl(pwm, PWM_SERVO_SET(4), 1500);
            ioctl(pwm, PWM_SERVO_SET(5), 1500);
            ioctl(pwm, PWM_SERVO_SET(6), 1500);
            ioctl(pwm, PWM_SERVO_SET(7), 1500);
            pwm_enabled = 0;
            printf("\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\n");
          }
			// Read GCS parameters
			param_get(GCS_comms_pointers.Kp_h, 		&(GCS_parameters.Kp_h));
			param_get(GCS_comms_pointers.Ki_h, 		&(GCS_parameters.Ki_h));
			param_get(GCS_comms_pointers.Kd_h, 		&(GCS_parameters.Kd_h));
			param_get(GCS_comms_pointers.Kp_u, 		&(GCS_parameters.Kp_u));
			param_get(GCS_comms_pointers.Ki_u, 		&(GCS_parameters.Ki_u));
			param_get(GCS_comms_pointers.Kp_theta, 	&(GCS_parameters.Kp_theta));
			param_get(GCS_comms_pointers.Ki_theta, 	&(GCS_parameters.Ki_theta));
			param_get(GCS_comms_pointers.Kp_phi, 	&(GCS_parameters.Kp_phi));
			param_get(GCS_comms_pointers.Ki_phi, 	&(GCS_parameters.Ki_phi));
			param_get(GCS_comms_pointers.Kp_q, 		&(GCS_parameters.Kp_q));
			param_get(GCS_comms_pointers.Ki_q, 		&(GCS_parameters.Ki_q));
			param_get(GCS_comms_pointers.Kp_p, 		&(GCS_parameters.Kp_p));
			param_get(GCS_comms_pointers.Ki_p, 		&(GCS_parameters.Ki_p));
			
			// Declarar las ganancias de Simulink: se podria necesitar Casting!
			dbx_control_P.Kp_h = GCS_parameters.Kp_h;
			dbx_control_P.Ki_h  = GCS_parameters.Ki_h;
            dbx_control_P.Kd_h  = GCS_parameters.Kd_h;
            dbx_control_P.Kp_u  = GCS_parameters.Kp_u;
            dbx_control_P.Ki_u  = GCS_parameters.Ki_u;
            dbx_control_P.Kp_theta  = GCS_parameters.Kp_theta;
            dbx_control_P.Ki_theta  = GCS_parameters.Ki_theta;
            dbx_control_P.Kp_phi  = GCS_parameters.Kp_phi;
            dbx_control_P.Ki_phi  = GCS_parameters.Ki_phi;
            dbx_control_P.Kp_q  = GCS_parameters.Kp_q;
            dbx_control_P.Ki_q  = GCS_parameters.Ki_q;
            dbx_control_P.Kp_p  = GCS_parameters.Kp_p;
            dbx_control_P.Ki_p  = GCS_parameters.Ki_p;
            
          // output FMU LED signals
          if (dbx_control_Y.led_blue == 1) {
            led_on(LED_BLUE);
          } else {
            led_off(LED_BLUE);
          }
          if (dbx_control_Y.led_red == 1) {
            led_on(LED_RED);
          } else {
            led_off(LED_RED);
          }
          // output RGBLED signals
          rgbled_rgbset_t rgb_value;
          rgb_value.red = dbx_control_Y.rgb_red;
          rgb_value.green = dbx_control_Y.rgb_green;
          rgb_value.blue = dbx_control_Y.rgb_blue;
          ioctl(rgbled, RGBLED_SET_RGB, (unsigned long)&rgb_value);
          //print debug data
          printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t\n",
              (double)(dbx_control_U.runtime/1000000),
              (double)dbx_control_Y.debug1,
              (double)dbx_control_Y.debug2,
              (double)dbx_control_Y.debug3,
              (double)dbx_control_Y.debug4,
              (double)dbx_control_Y.debug5,
              (double)dbx_control_Y.debug6,
              (double)dbx_control_Y.debug7,
              (double)dbx_control_Y.debug8);
          
          // Sensors debuging and testing
//           printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t\n",
//               (double)(dbx_control_U.runtime/1000000),
//               (double)local_pos.x,
//               (double)local_pos.y,
//               (double)local_pos.z,
//               (double)gps.epv,
//               (double)local_pos.vx,
//               (double)local_pos.vy,
//               (double)local_pos.vz,
//               (double)airspeed.true_airspeed_m_s,
//               (double)sensors.baro_alt_meter,
//               (double)sensors.accelerometer_m_s2[2],
//               (double)sensors.gyro_rad_s[0]);
          i = 1;
        }
        // output pwm signals
        if (pwm_enabled == 1) {
          ioctl(pwm, PWM_SERVO_SET(0), dbx_control_Y.pwm1);
          ioctl(pwm, PWM_SERVO_SET(1), dbx_control_Y.pwm2);
          ioctl(pwm, PWM_SERVO_SET(2), dbx_control_Y.pwm3);
          ioctl(pwm, PWM_SERVO_SET(3), dbx_control_Y.pwm4);
          ioctl(pwm, PWM_SERVO_SET(4), dbx_control_Y.pwm5);
          ioctl(pwm, PWM_SERVO_SET(5), dbx_control_Y.pwm6);
          ioctl(pwm, PWM_SERVO_SET(6), dbx_control_Y.pwm7);
          ioctl(pwm, PWM_SERVO_SET(7), dbx_control_Y.pwm8);
        } else {
          ioctl(pwm, PWM_SERVO_SET(0), 1100);
          ioctl(pwm, PWM_SERVO_SET(1), 1100);
          ioctl(pwm, PWM_SERVO_SET(2), 1970);
          ioctl(pwm, PWM_SERVO_SET(3), 1898);
          ioctl(pwm, PWM_SERVO_SET(4), 1500);
          ioctl(pwm, PWM_SERVO_SET(5), 1500);
          ioctl(pwm, PWM_SERVO_SET(6), 1500);
          ioctl(pwm, PWM_SERVO_SET(7), 1500);
        }
        // execute simulink code
        dbx_control_step();
      }
    }
  }
  // disable pwm outputs
  ioctl(pwm, PWM_SERVO_SET(0), 1100);
  ioctl(pwm, PWM_SERVO_SET(1), 1100);
  ioctl(pwm, PWM_SERVO_SET(2), 1500);
  ioctl(pwm, PWM_SERVO_SET(3), 1970);
  ioctl(pwm, PWM_SERVO_SET(4), 1898);
  ioctl(pwm, PWM_SERVO_SET(5), 1500);
  ioctl(pwm, PWM_SERVO_SET(6), 1500);
  ioctl(pwm, PWM_SERVO_SET(7), 1500);
  ioctl(pwm, PWM_SERVO_DISARM, 0);
  // disable LEDs
  led_off(LED_BLUE);
  led_off(LED_RED);
  ioctl(rgbled, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
  // close sensor subscriptions
  close(sensors_sub);
  close(attitude_sub);
  close(pwm_inputs_sub);
  close(gps_sub);
  // terminate application thread
  exit(0);
}

int dbx_control_main(int argc, char *argv[])
{
  // start primary application thread
  if (!strcmp(argv[1], "start")) {
    thread_exit = false;
    simulink_task = px4_task_spawn_cmd("dbx_control",
      SCHED_DEFAULT,
      SCHED_PRIORITY_MAX - 15,
      10240,
      simulink_main,
      NULL);
    exit(0);
  }

  // terminate primary application thread
  if (!strcmp(argv[1], "stop")) {
    thread_exit = true;
    exit(0);
  }

  exit(1);
}