#ifndef HW_CONFIG_H
#define HW_CONFIG_H


/* Timer and PWM */
#define TIM_PWM			htim1				// PWM/ISR timer handle
#define TIM_CH_U		TIM_CHANNEL_1		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_W		TIM_CHANNEL_3		// Terminal W timer channel
#define INVERT_DTC		1					// PWM inverting (1) or non-inverting (0)

/* ISRs */
#define PWM_ISR			TIM1_UP_TIM10_IRQn	// PWM Timer ISR
#define CAN_ISR			CAN1_RX0_IRQn		// CAN Receive ISR

/* ADC */

#define ADC_CH_MAIN		hadc1				// ADC channel handle which drives simultaneous mode
#define ADC_CH_IA		hadc1					// Phase A current sense ADC channel handle.  0 = unused
#define ADC_CH_IB		hadc2				// Phase B current sense ADC channel handle.  0 = unused
#define ADC_CH_IC		0				// Phase C current sense ADC channel handle.  0 = unused
#define ADC_CH_VBUS		hadc3				// Bus voltage ADC channel handle.  0 = unused

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOA, GPIO_PIN_11  // Enable gate drive pin.
#define DRV_SPI			hspi1				// DRV SPI handle (DRV8323RS only)
#define DRV_CS			GPIOA, GPIO_PIN_4	// DRV CS pin (DRV8323RS only)

/* ===== DRV8323 variant selection (EASY CONFIG) =====================================
 * Match this to the populated gate-driver chip:
 *   DRV_USE_SPI 1  ->  DRV8323RS : firmware sets gain / OCP / PWM-mode / IDRIVE over SPI
 *   DRV_USE_SPI 0  ->  DRV8323RH : those are fixed by board STRAP RESISTORS (table below)
 *
 * --- DRV8323RH strap resistors that reproduce this firmware's old SPI defaults -------
 *   MODE   = 47k to AGND   -> 3x PWM mode
 *   GAIN   = tie to DVDD   -> 40 V/V    (set DRV_HW_CSA_GAIN below to match!)
 *   VDS    = Hi-Z (float)  -> 0.6V OCP trip   [75k to AGND for ~0.26V if you want tighter]
 *   IDRIVE = Hi-Z (float)  -> 120mA src / 240mA sink
 *   nFAULT -> external pull-up to DVDD ;  CAL -> tie low (or a GPIO for auto offset cal)
 *
 * DRV_HW_CSA_GAIN MUST equal the gain set by the GAIN strap (5/10/20/40 V/V) or the
 * current scaling (i_scale) will be wrong.  GAIN = 40 keeps I_SCALE unchanged.            */
#define DRV_USE_SPI		1					// 1 = DRV8323RS (SPI),  0 = DRV8323RH (hardware strap)
#define DRV_HW_CSA_GAIN	40					// (RH only) gain set by the GAIN strap: 5 / 10 / 20 / 40
//#define DRV_NFAULT	GPIOB, GPIO_PIN_X	// (RH only) uncomment+set if nFAULT is wired to the MCU

/* SPI encoder */
#define ENC_SPI			hspi3				// Encoder SPI handle
#define ENC_CS			GPIOA, GPIO_PIN_15	// Encoder SPI CS pin (HAL form, used at init)
/* Fast direct-register CS toggling (PA15) for the control ISR.  BSRR is atomic: low half
 * sets pins, high half (<<16) resets them.  Keep in sync with ENC_CS above. */
#define ENC_CS_LOW()	(GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16U)
#define ENC_CS_HIGH()	(GPIOA->BSRR = (uint32_t)GPIO_PIN_15)

/* ===== Encoder type selection (EASY CONFIG) ========================================
 * ENC_TYPE_ORIG   : original board sensor - one 16-bit SPI frame, angle read straight from
 *                   the frame, 16-bit (65536 CPR), SPI mode 0.
 * ENC_TYPE_MT6816 : MagnTek MT6816 - 14-bit (16384 CPR), SPI mode 3, TWO register reads
 *                   (0x03 = Angle<13:6>, 0x04 = Angle<5:0> + flags), clock <= 15.6MHz.
 * The matching SPI mode / prescaler is applied in spi.c via the ENC_SPI_* macros below.   */
#define ENC_TYPE_ORIG	0
#define ENC_TYPE_MT6816	1
#define ENC_TYPE		ENC_TYPE_ORIG		// <-- set to ENC_TYPE_MT6816 when an MT6816 is fitted

#if ENC_TYPE == ENC_TYPE_MT6816
  #define ENC_CPR			16384					// 14-bit
  #define ENC_LUT_SHIFT		7						// log2(ENC_CPR/128): 16384/128 = 2^7
  #define ENC_SPI_CPOL		SPI_POLARITY_HIGH		// MT6816 = SPI mode 3 (CPOL=1, CPHA=1)
  #define ENC_SPI_CPHA		SPI_PHASE_2EDGE
  #define ENC_SPI_PRESCALER	SPI_BAUDRATEPRESCALER_4	// SPI3 45MHz/4 = 11.25MHz (< 15.6MHz max)
  #define ENC_WARMUP_CMD	0x8300					// benign read of reg 0x03 (R/W=1, addr=0x03)
#else
  #define ENC_CPR			65536					// 16-bit
  #define ENC_LUT_SHIFT		9						// log2(ENC_CPR/128): 65536/128 = 2^9
  #define ENC_SPI_CPOL		SPI_POLARITY_LOW		// original sensor = SPI mode 0
  #define ENC_SPI_CPHA		SPI_PHASE_1EDGE
  #define ENC_SPI_PRESCALER	SPI_BAUDRATEPRESCALER_2
  #define ENC_WARMUP_CMD	0x0000
#endif
#define INV_CPR			1.0f/ENC_CPR
#define ENC_READ_WORD	0x00				// (original sensor) single-frame read command

/* Misc. GPIO */
#define LED         	GPIOC, GPIO_PIN_5	// LED Pin

/* CAN */
#define CAN_H			hcan1				// CAN handle

/* Other hardware-related constants */
#define I_SCALE 			0.0201416f  // Amps per A/D Count at 40X amplifier gain
#define V_SCALE 			0.0128906f    // Bus volts per A/D Count
#define DTC_MAX 			0.94f          	// Max duty cycle
#define DTC_MIN 			0.0f          	// Min duty cycle
#define DTC_COMP 			0.000f          // deadtime compensation (100 ns / 25 us)
#define DT					.000025f		// Loop period
#define ISR_BUDGET_CYCLES	4500u			// CPU cycles in one control period (180MHz / 40kHz) - ISR overrun threshold
#define EN_ENC_LINEARIZATION 1				// Enable/disable encoder linearization
#define V_BUS_MAX			40.0f			// max drive voltage (faults above this)

/* Current controller */
#define L_D .000368f			// D axis inductance (GIM6010-8: 368uH phase inductance)
#define L_Q .000368f			// Q axis inductance (GIM6010-8: 368uH phase inductance)
#define K_D .1f                    // Loop gain,  Volts/Amp
#define K_Q .1f                    // Loop gain,  Volts/Amp
#define K_SCALE 0.00012f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.045f                // PI zero, in radians per sample
#define KI_Q 0.045f                // PI zero, in radians per sample
#define OVERMODULATION 1.0f        // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA	.1f	// 1st order d/q current filter (not used in control)
#define VBUS_FILT_ALPHA		.1f		// 1st order bus voltage filter

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples


#endif
