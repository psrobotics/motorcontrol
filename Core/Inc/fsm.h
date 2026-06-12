/*
 * fsm.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */


#ifndef INC_FSM_H_
#define INC_FSM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>



#define MENU_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define INIT_TEMP_MODE      6

#define MENU_CMD			27
#define MOTOR_CMD			'm'
#define CAL_CMD				'c'
#define ENCODER_CMD			'e'
#define SETUP_CMD			's'
#define ZERO_CMD			'z'
#define ENTER_CMD			13

/* Deferred console render requests: set in ISR/event context, rendered by fsm_service()
 * in the main() background loop so no printf ever runs inside an interrupt. */
#define PRINT_MENU			(1u<<0)
#define PRINT_SETUP			(1u<<1)
#define PRINT_CAL_DONE		(1u<<2)
#define PRINT_ZERO			(1u<<3)


typedef struct{
	uint8_t state;
	uint8_t next_state;
	uint8_t state_change;
	uint8_t ready;
	char cmd_buff[8];
	char bytecount;
	char cmd_id;
	volatile uint32_t print_req;		// pending deferred console renders (bitmask of PRINT_*)
}FSMStruct;

void run_fsm(FSMStruct* fsmstate);
void fsm_rx_push(char c);				// enqueue a serial byte from the UART ISR (non-blocking)
void fsm_service(FSMStruct * fsmstate);	// drain serial input + render deferred output (call from main loop)
void update_fsm(FSMStruct * fsmstate, char fsm_input);
void fsm_enter_state(FSMStruct * fsmstate);
void fsm_exit_state(FSMStruct * fsmstate);
void enter_menu_state(void);
void enter_setup_state(void);
void enter_motor_mode(void);
void process_user_input(FSMStruct * fsmstate);

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_H_ */
