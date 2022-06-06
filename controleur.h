#ifndef CONTROLEUR_H_
#define CONTROLEUR_H_

enum state_robot
{
	GO_STATE_ROBOT=0,
	STOP_STATE_ROBOT=1,
	ARC_STATE_ROBOT=2,
	BACK_STATE_ROBOT=3
};
void initialiser_controleur(void);
#endif /* CONTROLEUR_H_ */
