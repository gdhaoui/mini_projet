
#include "audio/microphone.h"
#include "audio_processing.h"
#include <arm_math.h>
#include <arm_const_structs.h>



#define MIN_VALUE_THRESHOLD	15000


#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_GO	   		16	//250Hz
#define FREQ_STOP		19  //300Hz
#define FREQ_LEFT		22	//350Hz
#define FREQ_RIGHT		25	//400HZ
#define FREQ_ARC_L		29  //450Hz
#define FREQ_ARC_R		32  //500Hz
#define FREQ_DEC_RAD	38  //600Hz
#define FREQ_INC_RAD	42	//650Hz
#define FREQ_BACK		27 	//422Hz
#define FREQ_CONTINUE	35  //550Hz
#define MAX_FREQ		50//e don't analyze after this index to not use resources for nothing

int state_micro=GO_MICRO_STATE;
int command=NOTHING;
int old_command=NOTHING;
/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);

}

void sound_for_command(float* data){

	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	switch(state_micro)
	{
		case GO_MICRO_STATE:
			if(max_norm_index >= (FREQ_GO-1) && max_norm_index <= (FREQ_GO+1)){
				command=GO_COMMAND;
				break;
			}
			else if(max_norm_index >= (FREQ_STOP-1) && max_norm_index <= (FREQ_STOP+1)){
				command=STOP_COMMAND;

				break;
			}
			else{
				command=NOTHING;
				break;
			}

		case STOP_MICRO_STATE:
			if(max_norm_index >= (FREQ_GO-1) && max_norm_index <= (FREQ_GO+1)){
							command=GO_COMMAND;
							break;
			}
			if(max_norm_index >= (FREQ_LEFT-1) && max_norm_index <= (FREQ_LEFT+1)){
				command=LEFT;
				break;
			}
			else if(max_norm_index >= (FREQ_RIGHT-1) && max_norm_index <= (FREQ_RIGHT+1)){
				command=RIGHT;
				break;
			}
			else if(max_norm_index >= (FREQ_BACK-1) && max_norm_index <= (FREQ_BACK+1)){
				command=REVERSE_WAY_COMMAND;

				break;
			}
			else if(max_norm_index >= (FREQ_ARC_L-1) && max_norm_index <= (FREQ_ARC_L+1)){
				command=TURN_ARC_LEFT;
				old_command=TURN_ARC_LEFT;
				break;
			}
			else if(max_norm_index >= (FREQ_ARC_L-1) && max_norm_index <= (FREQ_ARC_L+1)){
				command=TURN_ARC_RIGHT;
				old_command=TURN_ARC_LEFT;
				break;
			}
			else{
				command=NOTHING;
				break;
			}

		case ARC_MICRO_STATE:
			if(max_norm_index >= (FREQ_CONTINUE-1) && max_norm_index <= (FREQ_CONTINUE+1)){
				command=CONTINUE;
				break;
			}
			else if(max_norm_index >= (FREQ_DEC_RAD-1) && max_norm_index <= (FREQ_DEC_RAD+1)){
			    command=DEC_RAD;
				break;
			}
			else if(max_norm_index >= (FREQ_INC_RAD-1) && max_norm_index <= (FREQ_INC_RAD+1)){
				command=INC_RAD;
				break;
			}
			else{
				command=NOTHING;
				break;
			}

		case NOT_HEARING:
			command=NOTHING;
			old_command=NOTHING;
			break;

	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :*/
	//int16_t *data;
	//Buffer containing 4 times 160 samples. the samples are sorted by micro
	//so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
	//uint16_t num_samples;
	//Tells how many data we get in total (should always be 640)

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	static float micLeft_cmplx_input[2 * FFT_SIZE];
	//Arrays containing the computed magnitude of the complex numbers
	static float micLeft_output[FFT_SIZE];


	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part

		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		nb_samples++;
		micLeft_cmplx_input[nb_samples] = 0;
		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		//FFT proccessing
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		//Magnitude processing
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		nb_samples = 0;
		sound_for_command(micLeft_output);
	}
}
void set_state_micro(char value)
{
	state_micro=value;
}
unsigned int get_command(void)
{
	return command;
}
unsigned int get_old_command(void)
{
	return old_command;
}

