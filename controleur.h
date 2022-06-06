

#ifndef CONTROLEUR_H_
#define CONTROLEUR_H_

void move_forward_steps(int steps);
void move_forward(void);
void turn_left_arc(float radius);
void turn_right_arc(float radius);
void turn_left_arc_steps(int steps, float radius, int speed);
void turn_right_arc_steps(int steps, float radius, int speed);
void motor_stop(void);
void turn_left(void);
void turn_right(void);
void turn_right_angle(char angle);
void turn_left_angle(char angle) ;
#endif /* CONTROLEUR_H_ */
