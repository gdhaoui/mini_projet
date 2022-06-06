#include <controleur.h>
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <motors.h>
#include "audio_processing.h"

#define MAX_NB_INSTRUCTION  150
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     12.84   //cm
#define NSTEP_ONE_TURN      1000
#define DEFAULT_SPEED		400

unsigned int state_robot;
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
	left_motor_set_speed(200);
	right_motor_set_speed(-200);
}
void turn_left(void)
{
	left_motor_set_speed(-200);
	right_motor_set_speed(+200);
}
//void turn_left_steps(int steps);
//void turn_right_steps(int steps);
static THD_WORKING_AREA(controleur_thd_wa,1024);
static THD_FUNCTION(controleur_thd,arg)
{
	(void)arg;
	chRegSetThreadName(__FUNCTION__);
	int32_t command_tab[MAX_NB_INSTRUCTION]={0};
	int num_command=-1;
	while(1){
		switch(state_robot)
		{
			case GO_STATE_ROBOT:

				if(get_command()==GO_COMMAND)
				{
					num_command++;
					command_tab[num_command]=GO_COMMAND;
					move_forward();
					state_robot=GO_STATE_ROBOT;
					set_state_micro(GO_MICRO_STATE);
					chThdSleepMilliseconds(2000);
					break;
				}
				else if(get_command()==STOP_COMMAND)
				{
					num_command++;
					command_tab[num_command]=left_motor_get_pos();
					command_tab[num_command+1]=right_motor_get_pos();
					motor_stop();
					set_state_micro(STOP_MICRO_STATE);
					state_robot=STOP_STATE_ROBOT;
					chThdSleepMilliseconds(3000);
					break;
				}
				else
				{
					state_robot=GO_STATE_ROBOT;
					chThdSleepMilliseconds(100);
					break;
				}

		case STOP_STATE_ROBOT:

				if(get_command()==LEFT)
				{
					num_command++;
					command_tab[num_command]=RIGHT;
					turn_left();
					set_state_micro(GO_MICRO_STATE);
				    state_robot=GO_STATE_ROBOT;
				    break;
				}
				else if(get_command()==RIGHT)
				{
					num_command++;
					command_tab[num_command]=LEFT;
					turn_right();
					set_state_micro(GO_MICRO_STATE);
				    state_robot=GO_STATE_ROBOT;
				    break;
				}

				/*
				else if(get_command()==REVERSE_WAY)
				{
					//turn_left(90);
					set_state_micro(NOT_HEARING);
				    state_robot=BACK_STATE_ROBOT;
				    break;
				}
				*/

				else if(get_command()==TURN_ARC_LEFT)
				{
					num_command++;
					command_tab[num_command]=TURN_ARC_RIGHT;
					turn_left_arc_steps(500,9.5,300);
					set_state_micro(ARC_MICRO_STATE);
				    state_robot=ARC_STATE_ROBOT;
				    chThdSleepMilliseconds(2000);
				    break;
				}
				else if(get_command()==TURN_ARC_RIGHT)
				{
					num_command++;
					command_tab[num_command]=TURN_ARC_LEFT;
					turn_right_arc_steps(500,9.5,300);
					set_state_micro(ARC_MICRO_STATE);
				    state_robot=ARC_STATE_ROBOT;
					chThdSleepMilliseconds(2000);
					break;
				}
				chThdSleepMilliseconds(100);
				break;

			case ARC_STATE_ROBOT:

				if(get_command()==CONTINUE)
				{
					num_command++;
					command_tab[num_command]=left_motor_get_pos();
					command_tab[num_command+1]=right_motor_get_pos();
					command_tab[num_command+2]=get_old_command();
					if(get_old_command()==TURN_ARC_LEFT)
					{
						turn_left_arc(9.5);
						set_state_micro(GO_MICRO_STATE);
						state_robot=GO_STATE_ROBOT;
						break;
					}
					else if(get_old_command()==TURN_ARC_RIGHT)
					{
						turn_right_arc(9.5);
						set_state_micro(GO_MICRO_STATE);
						state_robot=GO_STATE_ROBOT;
						break;
					}
				}
				else if(get_command()==DEC_RAD)
				{
					if(get_old_command()==TURN_ARC_LEFT)
					{
						turn_left_arc_steps(500,9.5,-300);
						turn_left_arc_steps(500,8.5,300);
						state_robot=ARC_STATE_ROBOT;
						break;
					}
					else if(get_old_command()==TURN_ARC_RIGHT)
					{
						turn_right_arc_steps(500,9.5,-300);
						turn_right_arc_steps(500,8.5,300);
					    state_robot=ARC_STATE_ROBOT;
					    break;
					}
				}
				else if(get_command()==INC_RAD)
				{
					if(get_old_command()==TURN_ARC_LEFT)
					{
						turn_left_arc_steps(500,9.5,-300);
						turn_left_arc_steps(500,10.5,300);
						state_robot=ARC_STATE_ROBOT;
						break;
					}
					else if(get_old_command()==TURN_ARC_RIGHT)
					{
						turn_right_arc_steps(500,9.5,-300);
						turn_right_arc_steps(500,10.5,300);
						state_robot=ARC_STATE_ROBOT;
						break;
					}
				}
				chThdSleepMilliseconds(100);
				break;

				}
		}
}

void initialiser_controleur(void)
{
	state_robot=GO_STATE_ROBOT;
	set_state_micro(GO_MICRO_STATE);
	chThdCreateStatic(controleur_thd_wa,sizeof(controleur_thd_wa),NORMALPRIO,controleur_thd,NULL);
}
