/* Mini-BT-Pcb-328P-firmware - PIN configuration

 On the basis of the devices attached to your board, comment or uncomment the following lines.
 Change the PIN number according to your needs. 
 PIN 2-13  = D2-D13
 PIN 14-21 = A0-A7 
 
 The following pins are not available and defined in MySConfig.cpp
 IRQ PIN = D2
 MY_RFM69_RST_PIN = D9

The signaling LED pin are defined in MySConfigg.cpp; comment the lines out if you need the pins for other scope
ERROR LED = A0
TX LED = A1
RX LED = A2

 Uncomment and define the MY_SIGNING_ATSHA204_PIN line if you want to use an ATSHA204 IC for signing.
*/

#define REED_SW_PIN 3
//#define BOOSTER_PIN 4
//#define EXT_PWR_SENSE_PIN 7
#define PWR_LED_PIN 8
//#define MY_SIGNING_ATSHA204_PIN 17
#define BATTERY_V_MEASURE_PIN 21