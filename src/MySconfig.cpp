/* d-diot Mini-BT-Pcb-328P-firmware - MySensors Configuration */

// Node ID. Uncomment the line below if you prefer to manually set an ID. Otherwise the controller will assign a number (1-254) automatically

//#define MY_NODE_ID 1

// Repeater mode. Uncomment the line below to activate the repeater mode. In this case the MCU will newer go to sleep, so is better to use an external power source (USB)

//#define MY_REPEATER_FEATURE

// Security: uncomment the first line to enable signing. If a ATSHA204 PIN is defined in PinConfig.cpp this backend will be used, otherwise software signing

//#define MY_SIGNING_REQUEST_SIGNATURES
#ifdef MY_SIGNING_REQUEST_SIGNATURES
#ifdef MY_SIGNING_ATSHA204_PIN
#define MY_SIGNING_ATSHA204
#endif
#ifndef MY_SIGNING_ATSHA204_PIN
#define MY_SIGNING_SOFT
#endif
#endif

// Radio type definition: comment out the first line to configure a NRF24L01 radio module instead of RFM69

#define MY_RADIO_RFM69
#ifdef MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RFM69_868MHZ
//#define MY_RFM69_RST_PIN 9
#endif
#ifndef MY_RADIO_RFM69
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_RF24_IRQ_PIN 2
#endif

// Set serial baud rate to 9600. Necessary for 1 Mhz MCU

#define MY_BAUD_RATE (9600ul)

// Signaling LEDs. Comment out the three lines below if you don't have the radio signaling LEDs soldered on the board.

#define MY_DEFAULT_ERR_LED_PIN 14
#define MY_DEFAULT_TX_LED_PIN 15
#define MY_DEFAULT_RX_LED_PIN 16
// If you have soldered the radio signaling LEDs and you want to disable them uncomment the three lines below and comment out the three lines above.
//#define DISABLE_ERR_LED 14
//#define DISABLE_TX_LED 15
//#define DISABLE_RX_LED 16

// Timeout before starting loop without gateway connection

#define MY_TRANSPORT_WAIT_READY_MS 30000
#define MY_SPLASH_SCREEN_DISABLED

// Ack

static const bool ack = false;