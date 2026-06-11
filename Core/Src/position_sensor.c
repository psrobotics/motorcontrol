/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	for(int i = 0; i<n; i++){
		encoder->spi_tx_word = 0x0000;
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	}
	__HAL_SPI_ENABLE(&ENC_SPI);		// leave SPI enabled so the register-level read in ps_sample works
}

void ps_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	encoder->old_angle = encoder->angle_singleturn;

	/* SPI read of the absolute encoder via direct registers (much lighter than
	 * HAL_SPI_TransmitReceive + HAL_GPIO_WritePin in the control ISR, and with no
	 * 100ms-timeout / busy-state spinning).  SPI3 is left enabled by ps_warmup. */
	ENC_CS_LOW();									// CS low
	ENC_SPI.Instance->DR = ENC_READ_WORD;			// start the 16-bit frame
	while(!(ENC_SPI.Instance->SR & SPI_SR_RXNE));	// wait for the response word
	encoder->raw = ENC_SPI.Instance->DR;			// read result (clears RXNE)
	while(ENC_SPI.Instance->SR & SPI_SR_BSY);		// wait for bus idle before raising CS
	ENC_CS_HIGH();									// CS high

	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;


	/* Real angles in radians */
	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
	int int_angle = encoder->angle_singleturn;
	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int_angle = (int)encoder->elec_angle;
	encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
	//encoder->elec_angle = TWO_PI_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers
	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	if(!encoder->first_sample){
		encoder->turns = 0;
		if(encoder->angle_singleturn > PI_OVER_2_F){encoder->turns = -1;}
		else if(encoder->angle_singleturn < -PI_OVER_2_F){encoder->turns = 1;}
		encoder->first_sample = 1;
	}



	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;

	/* Velocity: fixed-window finite difference over (N_POS_SAMPLES-1) intervals, using a
	 * ring buffer instead of shifting the whole history array every sample.  ring_head
	 * holds the oldest entry (written N_POS_SAMPLES-1 calls ago); read it, then overwrite
	 * with the newest.  Identical window to the previous array-shift implementation. */
	float newest = encoder->angle_multiturn[0];
	float oldest = encoder->vel_ring[encoder->ring_head];
	encoder->vel_ring[encoder->ring_head] = newest;
	encoder->ring_head++;
	if(encoder->ring_head >= (N_POS_SAMPLES-1)){encoder->ring_head = 0;}
	encoder->velocity = (newest - oldest)/(dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

}

void ps_print(EncoderStruct * encoder, int dt_ms){
	printf("Raw: %d", encoder->raw);
	printf("   Linearized Count: %d", encoder->count);
	printf("   Single Turn: %f", encoder->angle_singleturn);
	printf("   Multiturn: %f", encoder->angle_multiturn[0]);
	printf("   Electrical: %f", encoder->elec_angle);
	printf("   Turns:  %d\r\n", encoder->turns);
	//HAL_Delay(dt_ms);
}
