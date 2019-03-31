/**
 *  copied from: 
 *  miniSDR v3
 *  By DD4WH and Frank Bösing
 *  GPL V3
 *  Changed by Dante Bauer
 *
 */

#include "AM_sync_demod.h"


void AMDemodSync::update(void)
{ 	if (!pass) return;
	audio_block_t *blocka, *blockb;
	int16_t *I_buffer, *Q_buffer;

	blocka = receiveWritable(0);
	blockb = receiveReadOnly(1);
	if (!blocka) 
		{
		if (blockb) release(blockb);
		return;
		}
	if (!blockb) 
		{
		release(blocka);
		return;
		}
	I_buffer = (int16_t *)(blocka->data);
	Q_buffer = (int16_t *)(blockb->data);
			

	// synchronous AM demodulation - the one with the PLL ;-)
	// code adapted from the wdsp library by Warren Pratt, GNU GPLv3
	// SYNCAM:
	static const float32_t omegaN = 200.0;     // PLL is able to grab a carrier that is not more than omegaN Hz away
	static const float32_t zeta = 0.15;     // the higher, the faster the PLL, the lower, the more stable the carrier is grabbed
	static const float32_t omega_min = 2.0 * PI * -4000.0 / SAMPLE_RATE;      // absolute minimum frequency the PLL can correct for
	static const float32_t omega_max = 2.0 * PI * 4000.0 / SAMPLE_RATE;     // absolute maximum frequency the PLL can correct for
	static const float32_t g1 = 1.0 - exp(-2.0 * omegaN * zeta / SAMPLE_RATE);     // used inside the algorithm
	static const float32_t g2 = -g1 + 2.0 * (1 - exp(-omegaN * zeta / SAMPLE_RATE) * cosf(omegaN / SAMPLE_RATE * sqrtf(1.0 - zeta * zeta)));       // used inside the algorithm
	static float32_t fil_out = 0.0f;
	static float32_t omega2 = 0.0f;
	static float32_t phzerror = 0.0f;

	float32_t ai;
	float32_t bi;
	float32_t aq;
	float32_t bq;
	float32_t det;
	float32_t Sin;
	float32_t Cos;
	float32_t del_out;
	float32_t corr[2];

		for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
		{

			Sin = sinf(phzerror);
			Cos = cosf(phzerror);
			ai = Cos * I_buffer[i];
			bi = Sin * I_buffer[i];
			aq = Cos * Q_buffer[i];
			bq = Sin * Q_buffer[i];

			corr[0] = +ai + bq;
			corr[1] = -bi + aq;
			
			//USB: (ai - bi) + (aq + bq);                    
            //LSB: (ai + bi) - (aq - bq);
			//AM: +ai + bq;
			I_buffer[i] = clip_q31_to_q15(corr[0]);
			//I_buffer[i] = (corr[0]);
			//I_buffer[i] = (ai - bi) + (aq + bq);

			// BEWARE: with a Teensy 3.2, this will really take a lot of time to calculate!
			// use a optimized atan2 in that case
			det = atan2f(corr[1], corr[0]);
			//det = atan2f(corr[1], corr[0]);

			del_out = fil_out;
			omega2 = omega2 + g2 * det;
			if (omega2 < omega_min) omega2 = omega_min;
			else if (omega2 > omega_max) omega2 = omega_max;
			fil_out = g1 * det + omega2;
			phzerror = phzerror + del_out;

			// wrap round 2PI, modulus
			while (phzerror >= 2 * PI) phzerror -= 2.0 * PI;
			while (phzerror < 0.0) phzerror += 2.0 * PI;
		}
	delta_omega = omega2;
	//SYNCAM		
	transmit(blocka);
	release(blocka);
	release(blockb);
} 