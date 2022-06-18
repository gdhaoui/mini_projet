
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "motors.h"
#include "controleur.h"
#include "audio_processing.h"
#include "audio/microphone.h"

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    motors_init();
    initialiser_controleur();
    mic_start(&processAudioData);

    /* Infinite loop. */
    while (1)
    {
    	chThdSleepMilliseconds(500);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
