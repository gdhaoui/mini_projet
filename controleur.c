#include <controleur.h>
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <motors.h>
#include "audio_processing.h"

#define RADIUS_R			20.0f
#define TEST_STEPS 			500
#define MAX_NB_INSTRUCTION  200
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
void move_backward_steps(int steps)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(-DEFAULT_SPEED) ;
	right_motor_set_speed(-DEFAULT_SPEED);
	while(left_motor_get_pos()>steps){
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

void turn_left_arc_steps(int steps_r,int steps_l, float radius, int speed)
{
	float r1=radius-WHEEL_DISTANCE/2;
	float r2=radius+WHEEL_DISTANCE/2;
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
void turn_right_arc_steps(int steps_r,int steps_l, float radius, int speed)
{
	float r1=radius+WHEEL_DISTANCE/2;
	float r2=radius-WHEEL_DISTANCE/2;
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
void turn_left_steps_return(int steps_l, int steps_r){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(-200) ;
	right_motor_set_speed(200);
	while((fabs(left_motor_get_pos())<-steps_l)& (fabs(right_motor_get_pos())<steps_r)){
		chThdSleepMilliseconds(10);
	}
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
void turn_right_steps_return(int steps_l, int steps_r){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(200) ;
	right_motor_set_speed(-200);
	while((fabs(left_motor_get_pos())<steps_l)& (fabs(right_motor_get_pos())<-steps_r)){
		chThdSleepMilliseconds(10);
	}
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void turn_right_angle(char angle)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(DEFAULT_SPEED);
	right_motor_set_speed(-DEFAULT_SPEED);
	while(left_motor_get_pos()<(NSTEP_ONE_TURN/WHEEL_PERIMETER)*(angle*PI*WHEEL_DISTANCE)/360)
	{
		chThdSleepMilliseconds(100);
	}
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
}
static THD_WORKING_AREA(controleur_thd_wa,1024);
static THD_FUNCTION(controleur_thd,arg)
{
	(void)arg;
	chRegSetThreadName(__FUNCTION__);
	int32_t command_tab[MAX_NB_INSTRUCTION]={0};
	int num_command=-1;
	static float radius=RADIUS_R;
	int incrementor=0;
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
					num_command++;
					command_tab[num_command]=right_motor_get_pos();
					motor_stop();
					set_state_micro(STOP_MICRO_STATE);
					state_robot=STOP_STATE_ROBOT;


					chThdSleepMilliseconds(2000);
					break;
				}

				else
				{
					set_state_micro(GO_MICRO_STATE);
					state_robot=GO_STATE_ROBOT;
					chThdSleepMilliseconds(100);
					break;
				}

		case STOP_STATE_ROBOT:
				if(get_command()==RUN_COMMAND)
				{
					num_command++;
					command_tab[num_command]=RUN_COMMAND;
					move_forward();
					set_state_micro(GO_MICRO_STATE);
					state_robot=GO_STATE_ROBOT;
					break;
				}
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


				else if(get_command()==REVERSE_WAY)
				{
					turn_right_angle(180);
					motor_stop();
					num_command-=2;
					set_state_micro(NOT_HEARING);
				    state_robot=BACK_STATE_ROBOT;
				    break;
				}


				else if(get_command()==TURN_ARC_LEFT)
				{
					num_command++;
					command_tab[num_command]=TURN_ARC_RIGHT;
					turn_left_arc_steps(TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,radius,(DEFAULT_SPEED/4)*3);
					set_state_micro(ARC_MICRO_STATE);
				    state_robot=ARC_STATE_ROBOT;
				    chThdSleepMilliseconds(2000);
				    break;
				}
				else if(get_command()==TURN_ARC_RIGHT)
				{

					turn_right_arc_steps(TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,radius,(DEFAULT_SPEED/4)*3);
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
					//num_command++;
					//command_tab[num_command]=radius;
					//command_tab[num_command+1]=left_motor_get_pos();
					//command_tab[num_command+2]=right_motor_get_pos();
					//num_command+=2;
					if(get_old_command()==TURN_ARC_LEFT)
					{
						num_command++;
						command_tab[num_command]=TURN_ARC_RIGHT;
						num_command++;
						command_tab[num_command]=radius;
						turn_left_arc(radius);
						set_state_micro(GO_MICRO_STATE);
						state_robot=GO_STATE_ROBOT;
						break;
					}
					else if(get_old_command()==TURN_ARC_RIGHT)
					{
						num_command++;
						command_tab[num_command]=TURN_ARC_LEFT;
						num_command++;
						command_tab[num_command]=radius;
						turn_right_arc(radius);
						set_state_micro(GO_MICRO_STATE);
						state_robot=GO_STATE_ROBOT;
						break;
					}
				}
				else if(get_command()==DEC_RAD)
				{
					if(get_old_command()==TURN_ARC_LEFT)
					{
						turn_left_arc_steps(TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,radius,-(DEFAULT_SPEED/4)*3);
						radius-=2;
						motor_stop();
						chThdSleepMilliseconds(100);
						turn_left_arc_steps(TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,radius,(DEFAULT_SPEED/4)*3);
						state_robot=ARC_STATE_ROBOT;
						break;
					}
					else if(get_old_command()==TURN_ARC_RIGHT)
					{
						turn_right_arc_steps(TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,radius,-(DEFAULT_SPEED/4)*3);
						radius-=2;
						motor_stop();
						chThdSleepMilliseconds(100);
						turn_right_arc_steps(TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,radius,-(DEFAULT_SPEED/4)*3);
					    state_robot=ARC_STATE_ROBOT;
					    break;
					}

				}
				else if(get_command()==INC_RAD)
				{
					if(get_old_command()==TURN_ARC_LEFT)
					{
						turn_left_arc_steps(TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,radius,-(DEFAULT_SPEED/4)*3);
						radius+=2;
						turn_left_arc_steps(TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,radius,(DEFAULT_SPEED/4)*3);
						state_robot=ARC_STATE_ROBOT;
						break;
					}
					else if(get_old_command()==TURN_ARC_RIGHT)
					{
						turn_right_arc_steps(TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,radius,-(DEFAULT_SPEED/4)*3);
						radius+=2;
						turn_right_arc_steps(TEST_STEPS*(radius+WHEEL_DISTANCE/2)/radius,TEST_STEPS*(radius-WHEEL_DISTANCE/2)/radius,radius,(DEFAULT_SPEED/4)*3);
						state_robot=ARC_STATE_ROBOT;
						break;
					}
				}
				chThdSleepMilliseconds(100);
				break;

			case BACK_STATE_ROBOT:
				if(num_command!=-3)
				{
					if(command_tab[num_command]==LEFT){
						command_tab[num_command]=NOTHING;
						turn_left_steps_return(command_tab[num_command+2],command_tab[num_command+1]);
						command_tab[num_command+1]=NOTHING;
						command_tab[num_command+2]=NOTHING;
						num_command-=3;
					}
					else if(command_tab[num_command]==RIGHT){
						command_tab[num_command]=NOTHING;
						turn_right_steps_return(command_tab[num_command+2],command_tab[num_command+1]);
						command_tab[num_command+1]=NOTHING;
						command_tab[num_command+2]=NOTHING;
						num_command-=3;
					}
					else if(command_tab[num_command]==TURN_ARC_LEFT){
						command_tab[num_command]=NOTHING;
						turn_left_arc_steps(command_tab[num_command+2],command_tab[num_command+3],command_tab[num_command+1],(DEFAULT_SPEED/4)*3);
						command_tab[num_command+1]=NOTHING;
						command_tab[num_command+2]=NOTHING;
						num_command-=3;
					}
					else if(command_tab[num_command]==TURN_ARC_RIGHT){
						command_tab[num_command]=NOTHING;
						turn_right_arc_steps(command_tab[num_command+2],command_tab[num_command+3],command_tab[num_command+1],(DEFAULT_SPEED/4)*3);
						command_tab[num_command+1]=NOTHING;
						command_tab[num_command+2]=NOTHING;
						command_tab[num_command+3]=NOTHING;
						num_command-=3;
					}
					else if(command_tab[num_command]==GO_COMMAND || command_tab[num_command]==RUN_COMMAND){
						command_tab[num_command]=NOTHING;
						move_forward_steps(command_tab[num_command+2]);
						command_tab[num_command+1]=NOTHING;
						command_tab[num_command+2]=NOTHING;
						num_command-=3;
					}
					else{
						num_command-=1;
					}

				}
				else{
						motor_stop();
						turn_right_angle(170);
						set_state_micro(GO_MICRO_STATE);
						state_robot=GO_STATE_ROBOT;
						chThdSleepMilliseconds(5000);
				}
		}
	}
}

void initialiser_controleur(void)
{
	state_robot=GO_STATE_ROBOT;
	set_state_micro(GO_MICRO_STATE);
	chThdCreateStatic(controleur_thd_wa,sizeof(controleur_thd_wa),NORMALPRIO,controleur_thd,NULL);
}
