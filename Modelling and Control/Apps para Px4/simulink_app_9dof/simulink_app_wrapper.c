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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/battery_status.h>

#include <drivers/drv_led.h>
#include <drivers/drv_rgbled.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>

// Include the generated files and code of Simulink: Autopilot
#include "simulink_app_ert_rtw/simulink_app.c"
#include "simulink_app_params.c"
//

__EXPORT int simulink_app_main(int argc, char *argv[]);

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

int step_size = 20; // fundamental sample time (ms)
int i = 1;

// GCS parameters
struct {
    float X_comm;
    float Y_comm;
    float Alt_comm;
    float Vx_yaw_comm;
    float Vy_yaw_comm;
    float Vz_yaw_comm;
    float phi_comm;
    float theta_comm;
    float head_comm;
    int32_t flight_mode_request; 
}		GCS_parameters;

    
int simulink_main(int argc, char *argv[])
{
  simulink_app_initialize();

  // declare data subscriptions
  int sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
  int pwm_inputs_sub = orb_subscribe(ORB_ID(input_rc));
  int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
  int flow_sub = orb_subscribe(ORB_ID(optical_flow));
  int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
  int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
  int bat_status_sub = orb_subscribe(ORB_ID(battery_status));

//   int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
//   int pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
//   int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
//   int global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

    
  // Declare data structs from subcriptions
  struct sensor_combined_s 				sensors;
  struct rc_input_values 				pwm_inputs;
  struct vehicle_attitude_s 			attitude;
  struct optical_flow_s 				flow;
  struct vehicle_gps_position_s 		gps;
  struct vehicle_local_position_s 		local_pos;              /*< vehicle local position */

   struct battery_status_s               bat_status;
           
  struct {
       param_t X_comm;
       param_t Y_comm;
       param_t Alt_comm;
       
       param_t Vx_yaw_comm;
       param_t Vy_yaw_comm;
       param_t Vz_yaw_comm;
       
       param_t phi_comm;
       param_t theta_comm;
       param_t head_comm;

       param_t flight_mode_request;
   }	GCS_comms_pointers; 
    
        // Get the pointers to GCS params
   GCS_comms_pointers.X_comm = param_find("3C_X_CO");
   GCS_comms_pointers.Y_comm = param_find("3C_Y_COM");
   GCS_comms_pointers.Alt_comm = param_find("3C_Z_COM");
   
   GCS_comms_pointers.Vx_yaw_comm = param_find("3C_VX_YAW_COM");
   GCS_comms_pointers.Vy_yaw_comm = param_find("3C_VY_YAW_COM");
   GCS_comms_pointers.Vz_yaw_comm = param_find("3C_VZ_YAW_COM");
   
   GCS_comms_pointers.phi_comm = param_find("3C_PHI_COM");
   GCS_comms_pointers.theta_comm = param_find("3C_THETA_COM");
   GCS_comms_pointers.head_comm = param_find("3C_HEADING_COM");
   
   GCS_comms_pointers.flight_mode_request = param_find("3C_FLIGHT_MOD_REQT"); 

   
  // Limiting the update rate
  orb_set_interval(sensors_sub, step_size);
  orb_set_interval(attitude_sub, step_size); // ????????????????????????????
  orb_set_interval(local_pos_sub, step_size); // ????????????????????????????

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
        orb_copy(ORB_ID(optical_flow), flow_sub, &flow);
        orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);   
        orb_copy(ORB_ID(input_rc), pwm_inputs_sub, &pwm_inputs);      
        orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
        orb_copy(ORB_ID(battery_status), bat_status_sub, &bat_status);
				
       /// Read GCS parameters
        param_get(GCS_comms_pointers.X_comm, &(GCS_parameters.X_comm));
        param_get(GCS_comms_pointers.Y_comm, &(GCS_parameters.Y_comm));
        param_get(GCS_comms_pointers.Alt_comm, &(GCS_parameters.Alt_comm));
        param_get(GCS_comms_pointers.Vx_yaw_comm, &(GCS_parameters.Vx_yaw_comm));
        param_get(GCS_comms_pointers.Vy_yaw_comm, &(GCS_parameters.Vy_yaw_comm));
        param_get(GCS_comms_pointers.Vz_yaw_comm, &(GCS_parameters.Vz_yaw_comm));
        param_get(GCS_comms_pointers.phi_comm, &(GCS_parameters.phi_comm));
        param_get(GCS_comms_pointers.theta_comm, &(GCS_parameters.theta_comm));
        param_get(GCS_comms_pointers.head_comm, &(GCS_parameters.head_comm));
        param_get(GCS_comms_pointers.flight_mode_request, &(GCS_parameters.flight_mode_request));

        simulink_app_U.runtime = hrt_absolute_time(); 
        simulink_app_U.mag_x = sensors.magnetometer_ga[0];
        simulink_app_U.mag_y = sensors.magnetometer_ga[1];
        simulink_app_U.mag_z = sensors.magnetometer_ga[2];
        simulink_app_U.acc_x = sensors.accelerometer_m_s2[0];
        simulink_app_U.acc_y = sensors.accelerometer_m_s2[1];
        simulink_app_U.acc_z = sensors.accelerometer_m_s2[2];
        simulink_app_U.gyro_x = sensors.gyro_rad_s[0];
        simulink_app_U.gyro_y = sensors.gyro_rad_s[1];
        simulink_app_U.gyro_z = sensors.gyro_rad_s[2];
        simulink_app_U.rate_roll = attitude.rollspeed;
        simulink_app_U.rate_pitch = attitude.pitchspeed;
        simulink_app_U.rate_yaw = attitude.yawspeed;
        simulink_app_U.att_roll = attitude.roll;
        simulink_app_U.att_pitch = attitude.pitch;
        simulink_app_U.att_yaw = attitude.yaw;
        simulink_app_U.q0 = attitude.q[0];
        simulink_app_U.q1 = attitude.q[1];
        simulink_app_U.q2 = attitude.q[2];
        simulink_app_U.q3 = attitude.q[3];
        simulink_app_U.baro_alt = sensors.baro_alt_meter;
        simulink_app_U.sonar_dist = flow.ground_distance_m; 
        simulink_app_U.flow_x = flow.flow_comp_x_m;
        simulink_app_U.flow_y = flow.flow_comp_y_m;
        simulink_app_U.gps_sat = gps.satellites_used; 
        simulink_app_U.gps_lat = 0.0000001*(double)gps.lat; 
        simulink_app_U.gps_lon = 0.0000001*(double)gps.lon; 
        simulink_app_U.gps_alt = 0.001*(double)gps.alt; 
        simulink_app_U.gps_vel = gps.vel_m_s; 
        simulink_app_U.gps_vel_n = gps.vel_n_m_s; 
        simulink_app_U.gps_vel_e = gps.vel_e_m_s; 
        simulink_app_U.gps_vel_d = gps.vel_d_m_s;
        simulink_app_U.ch1 = pwm_inputs.values[0];
        simulink_app_U.ch2 = pwm_inputs.values[1];
        simulink_app_U.ch3 = pwm_inputs.values[2];
        simulink_app_U.ch4 = pwm_inputs.values[3];
        simulink_app_U.ch5 = pwm_inputs.values[4];
        simulink_app_U.ch6 = pwm_inputs.values[5];
        simulink_app_U.ch7 = pwm_inputs.values[6];
        simulink_app_U.ch8 = pwm_inputs.values[7];    
         
		/*----- Added inputs ---------*/
		
		simulink_app_U.Lo_pos_x  = local_pos.x;
		simulink_app_U.Lo_pos_y  = local_pos.y;
		simulink_app_U.Lo_pos_z  = local_pos.z;
		simulink_app_U.Lo_vex  = local_pos.vx;
		simulink_app_U.Lo_vey  = local_pos.vy;
		simulink_app_U.Lo_vez  = local_pos.vz;
		simulink_app_U.Lo_yaw  = local_pos.yaw;

        simulink_app_U.gps_pdop = gps.eph; // pdop or hdop
        simulink_app_U.gps_vdop = gps.epv; // vdop
        
        simulink_app_U.bat_volts = bat_status.voltage_filtered_v; // Batery volts
        
        // GCS params
        simulink_app_U.X_comm = GCS_parameters.X_comm;
        simulink_app_U.Y_comm = GCS_parameters.Y_comm;
        simulink_app_U.Alt_comm = GCS_parameters.Alt_comm;
        simulink_app_U.Vx_yaw_comm = GCS_parameters.Vx_yaw_comm;
        simulink_app_U.Vy_yaw_comm = GCS_parameters.Vy_yaw_comm;
        simulink_app_U.Vz_yaw_comm = GCS_parameters.Vz_yaw_comm;
        simulink_app_U.phi_comm = GCS_parameters.phi_comm;
        simulink_app_U.theta_comm = GCS_parameters.theta_comm;
        simulink_app_U.head_comm = GCS_parameters.head_comm;
        simulink_app_U.flight_mode_request = GCS_parameters.flight_mode_request;
                                        
        if (i < 5) { // 10Hz loop
          i = i++;
        } else {          
          // check arm state
          if (simulink_app_Y.pwm_arm == 1 && pwm_enabled == 0) {
            // arm system
            pwm_enabled = 1;
            printf("\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\n");
          } else if (simulink_app_Y.pwm_arm == 0 && pwm_enabled == 1) {
            // disarm system
            ioctl(pwm, PWM_SERVO_SET(0), 1000);
            ioctl(pwm, PWM_SERVO_SET(1), 1000);
            ioctl(pwm, PWM_SERVO_SET(2), 1000);
            ioctl(pwm, PWM_SERVO_SET(3), 1000);
            ioctl(pwm, PWM_SERVO_SET(4), 1000);
            ioctl(pwm, PWM_SERVO_SET(5), 1000);
            ioctl(pwm, PWM_SERVO_SET(6), 1000);
            ioctl(pwm, PWM_SERVO_SET(7), 1000);
            pwm_enabled = 0;
            printf("\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\n");
          }
          // output FMU LED signals
          if (simulink_app_Y.led_blue == 1) {
            led_on(LED_BLUE);
          } else {
            led_off(LED_BLUE);
          }
          if (simulink_app_Y.led_red == 1) {
            led_on(LED_RED);
          } else {
            led_off(LED_RED);
          }
          // output RGBLED signals
          rgbled_rgbset_t rgb_value;
          rgb_value.red = simulink_app_Y.rgb_red;
          rgb_value.green = simulink_app_Y.rgb_green;
          rgb_value.blue = simulink_app_Y.rgb_blue;
          ioctl(rgbled, RGBLED_SET_RGB, (unsigned long)&rgb_value);
          //print debug data
          printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",
              (double)(simulink_app_U.runtime/1000000),
              (double)simulink_app_Y.debug1,
              (double)simulink_app_Y.debug2,
              (double)simulink_app_Y.debug3,
              (double)simulink_app_Y.debug4,
              (double)simulink_app_Y.debug5,
              (double)simulink_app_Y.debug6,
              (double)simulink_app_Y.debug7,
              (double)simulink_app_Y.debug8);
          i = 1;
        }
        // output pwm signals
        if (pwm_enabled == 1) {
          ioctl(pwm, PWM_SERVO_SET(0), simulink_app_Y.pwm1);
          ioctl(pwm, PWM_SERVO_SET(1), simulink_app_Y.pwm2);
          ioctl(pwm, PWM_SERVO_SET(2), simulink_app_Y.pwm3);
          ioctl(pwm, PWM_SERVO_SET(3), simulink_app_Y.pwm4);
          ioctl(pwm, PWM_SERVO_SET(4), simulink_app_Y.pwm5);
          ioctl(pwm, PWM_SERVO_SET(5), simulink_app_Y.pwm6);
          ioctl(pwm, PWM_SERVO_SET(6), simulink_app_Y.pwm7);
          ioctl(pwm, PWM_SERVO_SET(7), simulink_app_Y.pwm8);
        } else {
          ioctl(pwm, PWM_SERVO_SET(0), 1000);
          ioctl(pwm, PWM_SERVO_SET(1), 1000);
          ioctl(pwm, PWM_SERVO_SET(2), 1000);
          ioctl(pwm, PWM_SERVO_SET(3), 1000);
          ioctl(pwm, PWM_SERVO_SET(4), 1000);
          ioctl(pwm, PWM_SERVO_SET(5), 1000);
          ioctl(pwm, PWM_SERVO_SET(6), 1000);
          ioctl(pwm, PWM_SERVO_SET(7), 1000);
        }
        // execute simulink code
        simulink_app_step();
      }
    }
  }
  // disable pwm outputs
  ioctl(pwm, PWM_SERVO_SET(0), 1000);
  ioctl(pwm, PWM_SERVO_SET(1), 1000);
  ioctl(pwm, PWM_SERVO_SET(2), 1000);
  ioctl(pwm, PWM_SERVO_SET(3), 1000);
  ioctl(pwm, PWM_SERVO_SET(4), 1000);
  ioctl(pwm, PWM_SERVO_SET(5), 1000);
  ioctl(pwm, PWM_SERVO_SET(6), 1000);
  ioctl(pwm, PWM_SERVO_SET(7), 1000);
  ioctl(pwm, PWM_SERVO_DISARM, 0);
  // disable LEDs
  led_off(LED_BLUE);
  led_off(LED_RED);
  ioctl(rgbled, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
  // close sensor subscriptions
  close(sensors_sub);
  close(attitude_sub);
  close(flow_sub);
  close(pwm_inputs_sub);
  close(gps_sub);
  // terminate application thread
  exit(0);
}

int simulink_app_main(int argc, char *argv[])
{
  // start primary application thread
  if (!strcmp(argv[1], "start")) {
    thread_exit = false;
    simulink_task = task_spawn_cmd("simulink_app",
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