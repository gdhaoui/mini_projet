#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <leds.h>
#include <controleur.h>


//#include <audio_processing.h>
#include <sensors/proximity.h>
#include <communications.h>
#include <arm_math.h>




static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    //serial_start();
    //starts the USB communication
    //proximity_start();
    //usb_start();

    motors_init();
    //initialiser_capteur();
    //right_motor_set_speed(600);
    //left_motor_set_speed(600);

    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
   // mic_start(&processAudioData);
    //it_send_audio_to_conductor();
  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
