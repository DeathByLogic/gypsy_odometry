#ifndef _ENCODERS_H
#define _ENCODERS_H

// Constant Definitions
#define ENC_LEFT_A_PIN   	P8_40
#define ENC_LEFT_B_PIN   	P8_42
#define ENC_RIGHT_A_PIN  	P8_44
#define ENC_RIGHT_B_PIN  	P8_46

#define WHEEL_BASE			13.25      // Distance between wheels
#define WHEEL_DIM			4.875      // Diameter of wheels
#define ENC_COUNT			100        // Number of encoder counts per revolution

#define ENC_POLL_TIMEOUT	500
#define ENC_POLL_COUNT		4

// Type Definitions
typedef enum {
	STATE_FW_A,
	STATE_FW_B,
	STATE_FW_C,
	STATE_FW_D,
	STATE_RV_A,
	STATE_RV_B,
	STATE_RV_C,
	STATE_RV_D
} EncoderState;

//Function Prototypes
void encoders_init();
void encoders_uninit();
void update_location(Posistion *);
void *encoder_thread(void *);
void encoder_fsm(EncoderState *, int *, const bool, const bool);

// External Variables

#endif
