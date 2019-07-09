/*
 * File:          Omniwheels.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 64
#define PI 3.1416


  double A_pos_1,A_pos_dif,B_pos_1,B_pos_dif, C_pos_1, C_pos_dif;
  double A_pos_2 = 0;
  double B_pos_2 = 0;
  double C_pos_2 = 0;
  double dis_value_left;
  double dis_value_right;
  double dis_sen_left;
  double dis_sen_right;
  double encoder_left;
  double encoder_right;
  double value_cm_left;
  double value_cm_right;
  double turn_l;
  double turn_r;
  double giro;
  float A_angv, B_angv, C_angv, A_linv, B_linv, C_linv, A_rpm, B_rpm, C_rpm;
  double dis_sen;
  double radio=0.075;

int main(int argc, char **argv)
{

  wb_robot_init();



  WbDeviceTag wheel_A = wb_robot_get_device("motor_A");
  WbDeviceTag wheel_B = wb_robot_get_device("motor_B");
  WbDeviceTag wheel_C = wb_robot_get_device("motor_C");
  WbDeviceTag ds_left = wb_robot_get_device("distance_sensor_left");
  WbDeviceTag ds_right = wb_robot_get_device("distance_sensor_left");
  WbDeviceTag A_pos = wb_robot_get_device("position_sensor_A");
  WbDeviceTag B_pos = wb_robot_get_device("position_sensor_B");
  WbDeviceTag C_pos = wb_robot_get_device("position_sensor_C");

  wb_position_sensor_enable(A_pos, TIME_STEP);
  wb_position_sensor_enable(B_pos, TIME_STEP);
  wb_position_sensor_enable(C_pos, TIME_STEP);
  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);




  while (wb_robot_step(TIME_STEP) != -1) {



  wb_position_sensor_get_value(A_pos);
  wb_position_sensor_get_value(B_pos);
  wb_position_sensor_get_value(C_pos);

  A_pos_1 = wb_position_sensor_get_value(A_pos);
  A_pos_dif = A_pos_1 - A_pos_2;
  A_pos_2 = A_pos_1;
  A_angv = A_pos_dif/0.064;
  A_linv = A_angv*radio;
  A_rpm = -1*A_angv*(60/(2*PI));

  B_pos_1 = wb_position_sensor_get_value(B_pos);
  B_pos_dif = B_pos_1 - B_pos_2;
  B_pos_2 = B_pos_1;
  B_angv = B_pos_dif/0.064;
  B_linv = B_angv*radio;
  B_rpm = -1*B_angv*(60/(2*PI));

  C_pos_1 = wb_position_sensor_get_value(C_pos);
  C_pos_dif = C_pos_1 - C_pos_2;
  C_pos_2 = C_pos_1;
  C_angv = C_pos_dif/0.064;
  C_linv = C_angv*radio;
  C_rpm = -1*(C_angv*(60/(2*PI)));

  dis_value_left = wb_distance_sensor_get_value(ds_left);
  dis_value_right = wb_distance_sensor_get_value(ds_right);
  value_cm_left = ((dis_value_left *.2)/65535);
  value_cm_right = ((dis_value_right *.2)/65535);

  printf("Distancia left: %.2f  \n", value_cm_left);
  printf("Distancia right: %.2f  \n", value_cm_right);

/////////////////////motor y giro//////////////////

  wb_motor_set_position(wheel_A, INFINITY);
  wb_motor_set_velocity(wheel_A, 0);
  wb_motor_set_position(wheel_B, INFINITY);
  wb_motor_set_velocity(wheel_B, -1);
  wb_motor_set_position(wheel_C, INFINITY);
  wb_motor_set_velocity(wheel_C, 1);


  if (value_cm_left<=0.17 ) {
     turn_l++;
   }
   if (turn_l>=1 && turn_l<=55){
     wb_motor_set_position(wheel_A, INFINITY);
     wb_motor_set_velocity(wheel_A, 1);
     wb_motor_set_position(wheel_B, INFINITY);
     wb_motor_set_velocity(wheel_B, 1);
     wb_motor_set_position(wheel_C, INFINITY);
     wb_motor_set_velocity(wheel_C, 1);
     turn_l++;
   }

   else {
     turn_l=0;
   }

   if (value_cm_right<0.17) {
     turn_r++;
   }

   if (turn_r>=1 && turn_r<=55){
     wb_motor_set_position(wheel_A, INFINITY);
     wb_motor_set_velocity(wheel_A, -1);
     wb_motor_set_position(wheel_B, INFINITY);
     wb_motor_set_velocity(wheel_B, -1);
     wb_motor_set_position(wheel_C, INFINITY);
     wb_motor_set_velocity(wheel_C, -1);
     turn_r++;
   }

   else {
     turn_r=0;
}


}

  wb_robot_cleanup();

  return 0;
}
