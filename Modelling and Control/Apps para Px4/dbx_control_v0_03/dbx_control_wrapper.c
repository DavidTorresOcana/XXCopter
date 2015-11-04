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
#include "dbx_control_ert_rtw/dbx_control_data.c" // Aqui se cargan los datos y constantes con que se compila Simulink
#include "dbx_control_params.c"

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
    float Throtle_sens;
    float Yaw_sens;
    float Roll_pich_sens;
    float phi_tau;
    float phi_K_b;
    float phi_f_i;
    float theta_tau;
    float theta_K_b;
    float theta_f_i;
    float psi_tau;
    float psi_K_b;
    float psi_f_i;
    float p_tau;	
    float p_K_b;
    float q_tau;
    float q_K_b;
    float r_tau;
    float r_K_b;
    float Flaps_ang_deg;
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
    param_t Throtle_sens;
    param_t Yaw_sens;
    param_t Roll_pich_sens;
    
    param_t phi_tau;
    param_t phi_K_b;
    param_t phi_f_i;
    
    param_t theta_tau;
    param_t theta_K_b;
    param_t theta_f_i;
    
    param_t psi_tau;
    param_t psi_K_b;
    param_t psi_f_i;
    
    param_t p_tau;	
    param_t p_K_b;
    
    param_t q_tau;
    param_t q_K_b;
    
    param_t r_tau;
    param_t r_K_b;
    
    param_t Flaps_ang_deg;
  }	GCS_comms_pointers;
  
  // Get the pointers to GCS params
  GCS_comms_pointers.Throtle_sens = param_find("DBX_Throtle_sens");
  GCS_comms_pointers.Yaw_sens = param_find("DBX_Yaw_sens");
  GCS_comms_pointers.Roll_pich_sens = param_find("DBX_Roll_pich_sens");
  
  GCS_comms_pointers.phi_tau = param_find("DBX_phi_tau");
  GCS_comms_pointers.phi_K_b = param_find("DBX_phi_K_b");
  GCS_comms_pointers.phi_f_i = param_find("DBX_phi_f_i");
  
  GCS_comms_pointers.theta_tau = param_find("DBX_theta_tau");
  GCS_comms_pointers.theta_K_b = param_find("DBX_theta_K_b");
  GCS_comms_pointers.theta_f_i = param_find("DBX_theta_f_i");
  
  GCS_comms_pointers.psi_tau = param_find("DBX_psi_tau");
  GCS_comms_pointers.psi_K_b = param_find("DBX_psi_K_b");
  GCS_comms_pointers.psi_f_i = param_find("DBX_psi_f_i");
  
  GCS_comms_pointers.p_tau = param_find("DBX_p_tau");
  GCS_comms_pointers.p_K_b = param_find("DBX_p_K_b");
  
  GCS_comms_pointers.q_tau = param_find("DBX_q_tau");
  GCS_comms_pointers.q_K_b = param_find("DBX_q_K_b");
  
  GCS_comms_pointers.r_tau = param_find("DBX_r_tau");
  GCS_comms_pointers.r_K_b = param_find("DBX_r_K_b");
  
  GCS_comms_pointers.Flaps_ang_deg = param_find("DBX_Flaps_ang_deg");


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
            ioctl(pwm, PWM_SERVO_SET(2), 1100);
            ioctl(pwm, PWM_SERVO_SET(3), 1500);
            ioctl(pwm, PWM_SERVO_SET(4), 1500);
            ioctl(pwm, PWM_SERVO_SET(5), 1500);
            ioctl(pwm, PWM_SERVO_SET(6), 1500);
            ioctl(pwm, PWM_SERVO_SET(7), 1500);
            pwm_enabled = 0;
            printf("\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\n");
          }
          
          // Read GCS parameters
			param_get(GCS_comms_pointers.Throtle_sens, 		&(GCS_parameters.Throtle_sens));
			param_get(GCS_comms_pointers.Yaw_sens, 		&(GCS_parameters.Yaw_sens));
			param_get(GCS_comms_pointers.Roll_pich_sens, 		&(GCS_parameters.Roll_pich_sens));
			param_get(GCS_comms_pointers.phi_tau, 		&(GCS_parameters.phi_tau));
			param_get(GCS_comms_pointers.phi_K_b, 		&(GCS_parameters.phi_K_b));
			param_get(GCS_comms_pointers.phi_f_i,       &(GCS_parameters.phi_f_i));
			param_get(GCS_comms_pointers.theta_tau, 	&(GCS_parameters.theta_tau));
			param_get(GCS_comms_pointers.theta_K_b, 	&(GCS_parameters.theta_K_b));
			param_get(GCS_comms_pointers.theta_f_i, 	&(GCS_parameters.theta_f_i));
			param_get(GCS_comms_pointers.psi_tau, 		&(GCS_parameters.psi_tau));
			param_get(GCS_comms_pointers.psi_K_b, 		&(GCS_parameters.psi_K_b));
			param_get(GCS_comms_pointers.psi_f_i, 		&(GCS_parameters.psi_f_i));
			param_get(GCS_comms_pointers.p_tau,          &(GCS_parameters.p_tau));
			param_get(GCS_comms_pointers.p_K_b,          &(GCS_parameters.p_K_b));
			param_get(GCS_comms_pointers.q_tau,          &(GCS_parameters.q_tau));
			param_get(GCS_comms_pointers.q_K_b,          &(GCS_parameters.q_K_b));
            param_get(GCS_comms_pointers.r_tau,          &(GCS_parameters.r_tau));
            param_get(GCS_comms_pointers.r_K_b,          &(GCS_parameters.r_K_b));
            param_get(GCS_comms_pointers.Flaps_ang_deg,          &(GCS_parameters.Flaps_ang_deg));

			// Declarar las ganancias de Simulink: se podria necesitar Casting!
			dbx_control_P.Throtle_sens = GCS_parameters.Throtle_sens;
			dbx_control_P.Yaw_sens  = GCS_parameters.Yaw_sens;
            dbx_control_P.Roll_pich_sens  = GCS_parameters.Roll_pich_sens;
            dbx_control_P.phi_tau  = GCS_parameters.phi_tau;
            dbx_control_P.phi_K_b  = GCS_parameters.phi_K_b;
            dbx_control_P.phi_f_i  = GCS_parameters.phi_f_i;
            dbx_control_P.theta_tau  = GCS_parameters.theta_tau;
            dbx_control_P.theta_K_b  = GCS_parameters.theta_K_b;
            dbx_control_P.theta_f_i  = GCS_parameters.theta_f_i;
            dbx_control_P.psi_tau  = GCS_parameters.psi_tau;
            dbx_control_P.psi_K_b  = GCS_parameters.psi_K_b;
            dbx_control_P.psi_f_i  = GCS_parameters.psi_f_i;
            dbx_control_P.p_tau  = GCS_parameters.p_tau;
            dbx_control_P.p_K_b  = GCS_parameters.p_K_b;
            dbx_control_P.q_tau  = GCS_parameters.q_tau;
            dbx_control_P.q_K_b  = GCS_parameters.q_K_b;
            dbx_control_P.r_tau  = GCS_parameters.r_tau;
            dbx_control_P.r_K_b  = GCS_parameters.r_K_b;
            dbx_control_P.Flaps_ang_deg  = GCS_parameters.Flaps_ang_deg;

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
          
//           // Sensors debuging and testing
//           printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t\n",
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
//               (double)sensors.gyro_rad_s[0],
//               (double)bat_status.voltage_filtered_v   );
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
          ioctl(pwm, PWM_SERVO_SET(2), 1100);
          ioctl(pwm, PWM_SERVO_SET(3), 1500);
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
  ioctl(pwm, PWM_SERVO_SET(2), 1100);
  ioctl(pwm, PWM_SERVO_SET(3), 1500);
  ioctl(pwm, PWM_SERVO_SET(4), 1500);
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