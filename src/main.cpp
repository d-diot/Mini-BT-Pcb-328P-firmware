// **************************** PLATFORMIO ********************************

// Platformio libs
#include <stdint.h>
#include <Arduino.h>

// ******************************* NODE CONFIGURATION **********************************

// Sketch name and version
const char sketch_name[] = "Mini-BT-Pcb-328P";
const char sketch_version[] = "1.0";

// Devices configuration
#include <MainConfig.cpp>

// MySensors configuration
#include <MySConfig.cpp>


// ************************ END OF CONFIG **********************************

// **************************** INIT ***************************************

// Libraries
#include <MySensors.h>
#include <Vcc.h>

// Object initialization
Vcc vcc(VccCorrection);

// CHILD_ID
#ifdef ENABLE_VCC_MONITOR
#define CHILD_ID_VCC_VOLTAGE 1
#endif

// Messages
#ifdef CHILD_ID_VCC_VOLTAGE
MyMessage msgVccValue(CHILD_ID_VCC_VOLTAGE, V_VOLTAGE);
#endif

// ********************************* END OF INIT **********************************

// ****************************** CUSTOM FUNCTIONS ********************************

// Mean VCC reads
float read_vcc(int reads = 5)
{
  float sum = 0;
  for (int i = 1; i <= reads; i++)
  {
    sum += vcc.Read_Volts();
    //Serial.println(sum);
  }
  return (float)round(sum / reads * 100) / 100;
}

// Wait before radio message (for CR2032 batteries)
void cr2032_wait()
{
  if (BATTERY_TYPE == 2)
  {
    sleep(CR2032_RADIO_WAIT_TIME);
  }
}

// ************************** END OF CUSTOM FUNCTIONS *****************************/

// ******************** MYSESNSORS PRESENTANTION FUNCTION ***************************/

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(sketch_name, sketch_version, ack);
  cr2032_wait();
// Register all sensors to gw (they will be created as child devices)
#ifdef CHILD_ID_VCC_VOLTAGE
  present(CHILD_ID_VCC_VOLTAGE, S_MULTIMETER, "Vcc Voltage", ack);
  cr2032_wait();
#endif
}

// ******************** END OF MYSESNSORS PRESENTANTION FUNCTION ************************/

// ********************************** ARDUINO SETUP *************************************/

void setup()
{
  pinMode(3, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
}

// ***************************** END OF ARDUINO SETUP ******************************

// ******************************* ARDUINO LOOP ************************************

void loop()
{
  // Send heartbeat only after sleep
#ifdef ENABLE_HEARTBEAT
//  sendHeartbeat();
//  cr2032_wait();
#endif

//  float vcc_voltage = read_vcc(MEAN_VCC_READS);
//  send(msgVccValue.set((float)vcc_voltage, 2), ack);
//  cr2032_wait();

  int reed_sw = digitalRead(3);
  if (reed_sw == 0)
  {
    digitalWrite(8, HIGH);
  }
  else
  {
    digitalWrite(8, LOW);
  }

  // smartSleep(digitalPinToInterrupt(3), CHANGE, 60000);
}