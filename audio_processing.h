#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

enum etat_micro
{
	GO=0,
	STOP=1,
	ARC=2
};
enum command
{
	NOTHING=0,
	START=1,
	LEFT=2,
	RIGHT=3,
	REVERSE_WAY=4,
	TURN_ARC_LEFT=5,
	TURN_ARC_RIGHT=6,
	CONTINUE=7,
	DEC_RAD=8,
	INC_RAD=9
};



void set_etat(char value);
char get_command(void);
void processAudioData(int16_t *data, uint16_t num_samples);

#endif /* AUDIO_PROCESSING_H */
