#include <controleur.h>
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <motors.h>


#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     12.84   //cm
#define NSTEP_ONE_TURN      1000
#define DEFAULT_SPEED		400

// **************   INTERNAL FUNCTIONS ******************
void move_forward_steps(int steps)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(DEFAULT_SPEED) ;
	right_motor_set_speed(DEFAULT_SPEED);
	while(left_motor_get_pos()<steps){
		chThdSleepMilliseconds(10);
	}
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
void move_forward(void)
{
	left_motor_set_speed(DEFAULT_SPEED);
	right_motor_set_speed(DEFAULT_SPEED);
}
void turn_left_arc(float radius)
{
	float r1=radius-WHEEL_DISTANCE/2;
	float r2=radius+WHEEL_DISTANCE/2;
	left_motor_set_speed(DEFAULT_SPEED*r1/r2) ;
	right_motor_set_speed(DEFAULT_SPEED);

}
void turn_right_arc(float radius)
{
	float r1=radius+WHEEL_DISTANCE/2;
	float r2=radius-WHEEL_DISTANCE/2;
	left_motor_set_speed(DEFAULT_SPEED) ;
	right_motor_set_speed(DEFAULT_SPEED*r2/r1);

}

void turn_left_arc_steps(int steps, float radius, int speed)
{
	float r1=radius-WHEEL_DISTANCE/2;
	float r2=radius+WHEEL_DISTANCE/2;
	int steps_r=steps*r2/radius;
	int steps_l=steps*r1/radius;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(speed*r1/r2) ;
	right_motor_set_speed(speed);
	while((fabs(left_motor_get_pos())<steps_l)& (fabs(right_motor_get_pos())<steps_r)){
		chThdSleepMilliseconds(10);
	}
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
void turn_right_arc_steps(int steps, float radius, int speed)
{
	float r1=radius+WHEEL_DISTANCE/2;
	float r2=radius-WHEEL_DISTANCE/2;
	int steps_r=steps*r2/radius;
	int steps_l=steps*r1/radius;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(speed*r1/r2) ;
	right_motor_set_speed(speed);
	while((fabs(left_motor_get_pos())<steps_l)& (fabs(right_motor_get_pos())<steps_r)){
		chThdSleepMilliseconds(10);
	}
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void motor_stop(void)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
void turn_right(void)
{
	left_motor_set_speed(DEFAULT_SPEED);
	right_motor_set_speed(-DEFAULT_SPEED);
}
void turn_left(void)
{
	left_motor_set_speed(-DEFAULT_SPEED);
	right_motor_set_speed(DEFAULT_SPEED);
}

void turn_right_angle(char angle)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(DEFAULT_SPEED);
	right_motor_set_speed(-DEFAULT_SPEED);
	while(left_motor_get_pos()<(NSTEP_ONE_TURN/WHEEL_PERIMETER)*(angle*PI*WHEEL_DISTANCE)/360);
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
}
void turn_left_angle(char angle)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(-DEFAULT_SPEED);
	right_motor_set_speed(+DEFAULT_SPEED);
	while(left_motor_get_pos()<-(NSTEP_ONE_TURN/WHEEL_PERIMETER)*(angle*PI*WHEEL_DISTANCE)/360);
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);

}


