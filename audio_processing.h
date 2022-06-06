#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

enum state_micro
{
	GO_MICRO_STATE=0,
	STOP_MICRO_STATE=1,
	ARC_MICRO_STATE=2,
	NOT_HEARING=3
};
enum command
{
	NOTHING=0,
	GO_COMMAND=1,
	STOP_COMMAND=2,
	LEFT=3,
	RIGHT=4,
	REVERSE_WAY=5,
	TURN_ARC_LEFT=6,
	TURN_ARC_RIGHT=7,
	CONTINUE=8,
	DEC_RAD=9,
	INC_RAD=10
};

void set_state_micro(char value);
unsigned int get_command(void);
void processAudioData(int16_t *data, uint16_t num_samples);
unsigned int get_old_command(void);

#endif /* AUDIO_PROCESSING_H */
