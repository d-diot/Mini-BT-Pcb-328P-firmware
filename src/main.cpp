// **************************** PLATFORMIO ********************************

// Platformio libs
#include <stdint.h>
#include <Arduino.h>

// ******************************* NODE CONFIGURATION **********************************

// Sketch name and version
const char sketch_name[] = "Mini-BT-Pcb-328P";
const char sketch_version[] = "1.0";

//  Pin configuration
#include <PinConfig.cpp>

// Devices configuration
#include <MainConfig.cpp>

// MySensors configuration
#include <MySconfig.cpp>

// Sampling interval configuration
#ifdef F_DEBUG
static const uint32_t UPDATE_INTERVAL = 3000;
#endif
#ifndef F_DEBUG
static const uint32_t UPDATE_INTERVAL = 900000;
#endif
static const uint8_t FORCE_UPDATE_N_READS = 10;

// ************************ END OF CONFIG **********************************

// **************************** INIT ***************************************

// Libraries
#include <MySensors.h>
#include <Vcc.h>
#ifdef REED_SW_PIN
#include <Bounce2.h>
#endif

// Object initialization
Vcc vcc(VccCorrection);
#ifdef REED_SW_PIN
Bounce debouncer = Bounce();
#endif

// CHILD_ID
#ifdef ENABLE_VCC_MONITOR
#define CHILD_ID_VCC_VOLTAGE 1
#endif
#ifdef ENABLE_EXT_PWR_MONITOR
#define CHILD_ID_EXT_POWER 2
#endif
#ifdef ENABLE_BOOSTER_MONITOR
#define CHILD_ID_BOOSTER 3
#endif
#ifdef REED_SW_PIN
#define CHILD_ID_REED_SW 4
#endif
#ifdef SOIL_MOISTURE_PIN
#define CHILD_ID_SOIL_HUM 6
#endif

// Messages
#ifdef CHILD_ID_VCC_VOLTAGE
MyMessage msgVccValue(CHILD_ID_VCC_VOLTAGE, V_VOLTAGE);
#endif
#ifdef CHILD_ID_EXT_POWER
MyMessage msgExtPower(CHILD_ID_EXT_POWER, V_TRIPPED);
#endif
#ifdef CHILD_ID_BOOSTER
MyMessage msgBooster(CHILD_ID_BOOSTER, V_TRIPPED);
#endif
#ifdef CHILD_ID_REED_SW
MyMessage msgReedSW(CHILD_ID_REED_SW, V_TRIPPED);
#endif
#ifdef SOIL_MOISTURE_PIN
MyMessage msgSoilHum(CHILD_ID_SOIL_HUM, V_HUM);
#endif

// First run
bool first_run = true;

// Smartleep
int8_t wake_up_mode;

// Vcc read status variables
float initial_vcc_voltage;
float vcc_voltage;
float last_vcc_voltage;
uint8_t nNoUpdatesVccLevel = 0;

// External power status variable
bool ext_power = false;
#ifdef EXT_PWR_SENSE_PIN
bool last_ext_power = false;
uint8_t nNoUpdatesExtPwr = 0;
#endif

// Power LED status variables
#ifdef PWR_LED_PIN
bool trigger_pwr_led_update = false;
unsigned long low_batt_led_on_start_time = 0;
#endif

// Booster status variables
#ifdef BOOSTER_PIN
bool boost_status = false;
bool last_boost_status = false;
uint8_t nNoUpdatesBooster = 0;
#endif

// Batteries status variables
#ifdef ENABLE_BATTERY_MONITOR
uint8_t batt_percent_value;
uint8_t last_batt_percent_value;
uint8_t nNoUpdatesBattPercent = 0;
#endif

// Reed switch status variables
#ifdef CHILD_ID_REED_SW
bool reed_sw_status = false;
bool last_reed_sw_status = false;
uint8_t nNoUpdatesReedSW = 0;
uint8_t cycle_counter = 0;
#endif

// Soil Moisture Sensor
#ifdef CHILD_ID_SOIL_HUM
uint16_t soil_analog_read = 0;
uint16_t soil_analog_last_read = 0;
uint16_t nNoUpdatesSoilMoisture = 0;

#endif

// ********************************* END OF INIT **********************************

// ****************************** CUSTOM FUNCTIONS ********************************

// Wait before radio message (for CR2032 batteries)
void cr2032_wait()
{
  if (BATTERY_TYPE == 2 && !ext_power)
  {
    wdt_reset();
    sleep(CR2032_RADIO_WAIT_TIME);
    wdt_reset();
  }
}

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

// Read Vcc
void update_Vcc_level()
{
  vcc_voltage = read_vcc(MEAN_VCC_READS);
  if (first_run || vcc_voltage <= last_vcc_voltage - VccTol || vcc_voltage >= last_vcc_voltage + VccTol || nNoUpdatesVccLevel == FORCE_UPDATE_N_READS)
  {
    last_vcc_voltage = vcc_voltage;
    nNoUpdatesVccLevel = 0;
#ifdef ENABLE_VCC_MONITOR
    send(msgVccValue.set((float)vcc_voltage, 2), ack);
    cr2032_wait();
#endif
#ifdef F_DEBUG
    Serial.print("Mean Vcc:");
    Serial.println(vcc_voltage);
#endif
  }
  else
  {
    nNoUpdatesVccLevel++;
  }
}

// Mean and smooth the analog reads
int analog_smooth(int PIN, int reads)
{
  int smoothed = 0;
  for (int i = 1; i <= reads; i++)
  {
    smoothed = (smoothed * (i - 1) + analogRead(PIN)) / i;
  }
  return smoothed;
}

// map function for floating values
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (x < in_min)
  {
    return out_min;
  }
  else if (x > in_max)
  {
    return out_max;
  }
  else
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
#ifdef CHILD_ID_EXT_POWER
  present(CHILD_ID_EXT_POWER, S_SPRINKLER, "External power", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_BOOSTER
  present(CHILD_ID_BOOSTER, S_SPRINKLER, "Booster", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_REED_SW
  present(CHILD_ID_REED_SW, S_DOOR, "Reed Switch", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_SOIL_HUM
  present(CHILD_ID_SOIL_HUM, S_HUM, "Soil Moisture", ack);
  cr2032_wait();
#endif
}

// ******************** END OF MYSESNSORS PRESENTANTION FUNCTION ************************/

// ********************************** ARDUINO SETUP *************************************/

void setup()
{
#ifdef POWER_PIN
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  wait(POWER_PIN_WAIT_TIME);
#endif
#ifdef EXT_PWR_SENSE_PIN
  pinMode(EXT_PWR_SENSE_PIN, INPUT);
#endif
#ifdef BOOSTER_PIN
  pinMode(BOOSTER_PIN, OUTPUT);
  digitalWrite(BOOSTER_PIN, LOW);
#endif
#ifdef PWR_LED_PIN
  pinMode(PWR_LED_PIN, OUTPUT);
  digitalWrite(PWR_LED_PIN, LOW);
#endif
#ifdef REED_SW_PIN
  pinMode(REED_SW_PIN, INPUT_PULLUP);
  debouncer.attach(REED_SW_PIN);
  debouncer.interval(DEBOUNCE_INTERVAL);
#endif
#ifdef SOIL_MOISTURE_PIN
  pinMode(SOIL_MOISTURE_PIN, INPUT);
#endif
#ifdef DISABLE_ERR_LED
  pinMode(DISABLE_ERR_LED, INPUT_PULLUP);
#endif
#ifdef DISABLE_TX_LED
  pinMode(DISABLE_TX_LED, INPUT_PULLUP);
#endif
#ifdef DISABLE_RX_LED
  pinMode(DISABLE_RX_LED, INPUT_PULLUP);
#endif
#ifdef AAA_BATT_CHEMISTRY
  if (AAA_BATT_CHEMISTRY == 0)
  {
    initial_vcc_voltage = vcc.Read_Volts();
  }
#endif
}

// ***************************** END OF ARDUINO SETUP ******************************

// ******************************* ARDUINO LOOP ************************************

void loop()
{
  // Update the Bounce instance:
#ifdef CHILD_ID_REED_SW
  debouncer.update();
#endif

  // To reduce radio traffic, disable forced updates after FORCE_UPDATE_N_READS when MY_REPEATER_FEATURE is active and MCU never goes to sleep
#ifdef MY_REPEATER_FEATURE
  if (wake_up_mode == -3)
  {
    nNoUpdatesVccLevel = 0;
#ifdef BOOSTER_PIN
    nNoUpdatesBooster = 0;
#endif
#ifdef EXT_PWR_SENSE_PIN
    nNoUpdatesExtPwr = 0;
#endif
#ifdef CHILD_ID_REED_SW
    nNoUpdatesReedSW = 0;
#endif
#ifdef ENABLE_BATTERY_MONITOR
    nNoUpdatesBattPercent = 0;
#endif
#ifdef CHILD_ID_SOIL_HUM
    nNoUpdatesSoilMoisture = 0;
#endif
  }
#endif

  // Turn ON power PIN
#ifdef POWER_PIN
  if (first_run || wake_up_mode != -3)
  {
    digitalWrite(POWER_PIN, HIGH);
    wdt_reset();
    wait(POWER_PIN_WAIT_TIME);
    wdt_reset();
  }
#endif

#ifdef F_DEBUG
  Serial.print("Wake up mode:");
  Serial.println(wake_up_mode);
#endif

// Send heartbeat only after sleep
#ifdef ENABLE_HEARTBEAT
#ifndef CHILD_ID_REED_SW
  if (wake_up_mode != -3)
  {
    sendHeartbeat();
    cr2032_wait();
#ifdef F_DEBUG
    Serial.println("Sending heartbeat");
#endif
  }
#endif
#ifdef CHILD_ID_REED_SW
  if (wake_up_mode != -3 && cycle_counter == 0)
  {
    sendHeartbeat();
    cr2032_wait();
#ifdef F_DEBUG
    Serial.println("Sending heartbeat");
#endif
  }
#endif
#endif

// Read the reed switch:
#ifdef CHILD_ID_REED_SW
  // Read the switch status. OPEN = HIGH, CLOSE = LOW
  reed_sw_status = debouncer.read() ? true : false;
  if (reed_sw_status != last_reed_sw_status || first_run || nNoUpdatesReedSW == FORCE_UPDATE_N_READS)
  {
    last_reed_sw_status = reed_sw_status;
    nNoUpdatesReedSW = 0;
    send(msgReedSW.set(reed_sw_status ? 1 : 0), ack);
    cr2032_wait();
#ifdef F_DEBUG
    Serial.print("Reed Switch: ");
    Serial.println(reed_sw_status);
#endif
  }
  else
  {
    nNoUpdatesReedSW++;
  }
#endif

  // Read Vcc
  update_Vcc_level();

// Activate Booster if necessary
#ifdef BOOSTER_PIN
  if (BOOSTER_POLICY == 0)
  {
    boost_status = false;
  }
  if (BOOSTER_POLICY == 1)
  {
    boost_status = true;
  }
  if (BOOSTER_POLICY == 2)
  {
    boost_status = (vcc_voltage < BoostThreshold) ? true : false;
  }
  if (first_run || boost_status != last_boost_status || nNoUpdatesBooster == FORCE_UPDATE_N_READS)
  {
    digitalWrite(BOOSTER_PIN, (boost_status) ? HIGH : LOW);
    last_boost_status = boost_status;
    nNoUpdatesBooster = 0;
#ifdef ENABLE_BOOSTER_MONITOR
    send(msgBooster.set(boost_status ? 1 : 0), ack);
    cr2032_wait();
#endif
#ifdef F_DEBUG
    Serial.print("Booster:");
    Serial.println(boost_status);
#endif
  }
  else
  {
    nNoUpdatesBooster++;
  }
#endif

// Soil Moisture Sensor
#ifdef CHILD_ID_SOIL_HUM
  soil_analog_read = analog_smooth(SOIL_MOISTURE_PIN, SOIL_ANALOG_READS);
#ifdef F_DEBUG
  Serial.print("Soil moisture analog read:");
  Serial.println(soil_analog_read);
#endif
  if (first_run || soil_analog_read <= soil_analog_last_read - SOIL_ANALOG_TOLLERANCE || soil_analog_read >= soil_analog_last_read + SOIL_ANALOG_TOLLERANCE || nNoUpdatesSoilMoisture == FORCE_UPDATE_N_READS)
  {
    nNoUpdatesSoilMoisture = 0;
    soil_analog_last_read = soil_analog_read;
    send(msgSoilHum.set(int(100 - map(constrain(soil_analog_last_read, WET_ANALOG_VALUE, DRY_ANALOG_VALUE), WET_ANALOG_VALUE, DRY_ANALOG_VALUE, 0, 100))), ack);
    cr2032_wait();
#ifdef F_DEBUG
    Serial.print("Soil moisture:");
    Serial.println(int(100 - map(constrain(soil_analog_last_read, WET_ANALOG_VALUE, DRY_ANALOG_VALUE), WET_ANALOG_VALUE, DRY_ANALOG_VALUE, 0, 100)));
#endif
  }
  else
  {
    nNoUpdatesSoilMoisture++;
  }
#endif

// Detect external power presence. Logic is reversed: HIGH = no external power, LOW = external_power
#ifdef EXT_PWR_SENSE_PIN
  ext_power = digitalRead(EXT_PWR_SENSE_PIN) ? false : true;
  if (first_run || ext_power != last_ext_power || nNoUpdatesExtPwr == FORCE_UPDATE_N_READS)
  {
    last_ext_power = ext_power;
    nNoUpdatesExtPwr = 0;
    trigger_pwr_led_update = false;
#ifdef CHILD_ID_EXT_POWER
    send(msgExtPower.set(ext_power ? 1 : 0), ack);
    cr2032_wait();
#endif
#ifdef F_DEBUG
    Serial.print("External Power:");
    Serial.println(ext_power);
#endif
#ifdef PWR_LED_PIN
    if (ext_power)
    {
      digitalWrite(PWR_LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(PWR_LED_PIN, LOW);
    }
#ifdef F_DEBUG
    Serial.print("Power LED status (ext power):");
    Serial.println(digitalRead(PWR_LED_PIN));
#endif
#endif
  }
  else
  {
    nNoUpdatesExtPwr++;
  }
#endif

// Read battery level
#ifdef ENABLE_BATTERY_MONITOR
  if (ext_power)
  {
    batt_percent_value = 0;
  }
  else if (!ext_power)
  {
    // CR2032: V min = 2.4, V max = 3.0
    if (BATTERY_TYPE == 2)
    {
#ifndef BATTERY_V_MEASURE_PIN
      batt_percent_value = (uint8_t)round(vcc.Read_Perc(CR2032_V_MIN, CR2032_V_MAX));
#endif
#ifdef BATTERY_V_MEASURE_PIN
      float batt_voltage_measure = (float)(analog_smooth(BATTERY_V_MEASURE_PIN, MEAN_V_BATT_READS) / 1023.0) * vcc_voltage * V_BATT_CORRECTION;
#ifdef F_DEBUG
      Serial.print("Battery voltage measure: ");
      Serial.println(batt_voltage_measure);
#endif
      batt_percent_value = (uint8_t)round(mapf(batt_voltage_measure, CR2032_V_MIN, CR2032_V_MAX, 0.0, 100.0));
#endif
    }
    // AAA bateries
    else if (BATTERY_TYPE == 1)
    {
      // Autodetect battery type
      if (AAA_BATT_CHEMISTRY == 0)
      {
        if (initial_vcc_voltage > NIMH_VMAX_THRESHOLD)
        {
#ifndef BATTERY_V_MEASURE_PIN
          batt_percent_value = (uint8_t)round(vcc.Read_Perc(ALK_V_MIN, ALK_V_MAX));
#endif
#ifdef BATTERY_V_MEASURE_PIN
          float batt_voltage_measure = (float)(analog_smooth(BATTERY_V_MEASURE_PIN, MEAN_V_BATT_READS) / 1023.0) * vcc_voltage * V_BATT_CORRECTION;
#ifdef F_DEBUG
          Serial.print("Battery voltage measure: ");
          Serial.println(batt_voltage_measure);
#endif
          batt_percent_value = (uint8_t)round(mapf(batt_voltage_measure, ALK_V_MIN, ALK_V_MAX, 0.0, 100.0));
#endif
        }
        else
        {
#ifndef BATTERY_V_MEASURE_PIN
          batt_percent_value = (uint8_t)round(vcc.Read_Perc(NIMH_V_MIN, NIMH_V_MAX));
#endif
#ifdef BATTERY_V_MEASURE_PIN
          float batt_voltage_measure = (float)(analog_smooth(BATTERY_V_MEASURE_PIN, MEAN_V_BATT_READS) / 1023.0) * vcc_voltage * V_BATT_CORRECTION;
#ifdef F_DEBUG
          Serial.print("Battery voltage measure: ");
          Serial.println(batt_voltage_measure);
#endif
          batt_percent_value = (uint8_t)round(mapf(batt_voltage_measure, NIMH_V_MIN, NIMH_V_MAX, 0.0, 100.0));
#endif
        }
      }
      // Custom V min and V max
      else if (AAA_BATT_CHEMISTRY == 1)
      {
#ifndef BATTERY_V_MEASURE_PIN
        batt_percent_value = (uint8_t)round(vcc.Read_Perc(CUSTOM_V_MIN, CUSTOM_V_MAX));
#endif
#ifdef BATTERY_V_MEASURE_PIN
        float batt_voltage_measure = (float)(analog_smooth(BATTERY_V_MEASURE_PIN, MEAN_V_BATT_READS) / 1023.0) * vcc_voltage * V_BATT_CORRECTION;
#ifdef F_DEBUG
        Serial.print("Battery voltage measure: ");
        Serial.println(batt_voltage_measure);
#endif
        batt_percent_value = (uint8_t)round(mapf(batt_voltage_measure, CUSTOM_V_MIN, CUSTOM_V_MAX, 0.0, 100.0));
#endif
      }
      // NiMH batteries (rechargeable)
      else if (AAA_BATT_CHEMISTRY == 2)
      {
#ifndef BATTERY_V_MEASURE_PIN
        batt_percent_value = (uint8_t)round(vcc.Read_Perc(NIMH_V_MIN, NIMH_V_MAX));
#endif
#ifdef BATTERY_V_MEASURE_PIN
        float batt_voltage_measure = (float)(analog_smooth(BATTERY_V_MEASURE_PIN, MEAN_V_BATT_READS) / 1023.0) * vcc_voltage * V_BATT_CORRECTION;
#ifdef F_DEBUG
        Serial.print("Battery voltage measure: ");
        Serial.println(batt_voltage_measure);
#endif
        batt_percent_value = (uint8_t)round(mapf(batt_voltage_measure, NIMH_V_MIN, NIMH_V_MAX, 0.0, 100.0));
#endif
      }
      // Alkaline batteries (disposable)
      else if (AAA_BATT_CHEMISTRY == 3)
      {
#ifndef BATTERY_V_MEASURE_PIN
        batt_percent_value = (uint8_t)round(vcc.Read_Perc(ALK_V_MIN, ALK_V_MAX));
#endif
#ifdef BATTERY_V_MEASURE_PIN
        float batt_voltage_measure = (float)(analog_smooth(BATTERY_V_MEASURE_PIN, MEAN_V_BATT_READS) / 1023.0) * vcc_voltage * V_BATT_CORRECTION;
#ifdef F_DEBUG
        Serial.print("Battery voltage measure: ");
        Serial.println(batt_voltage_measure);
#endif
        batt_percent_value = (uint8_t)round(mapf(batt_voltage_measure, ALK_V_MIN, ALK_V_MAX, 0.0, 100.0));
#endif
      }
      // Undefined battery chemistry
      else
      {
        batt_percent_value = 0;
      }
    }
    // undefined battery type
    else
    {
      batt_percent_value = 0;
    }
  }
  if (first_run || batt_percent_value <= last_batt_percent_value - BATTERY_PERCENT_TOLERANCE || batt_percent_value >= last_batt_percent_value + BATTERY_PERCENT_TOLERANCE || nNoUpdatesBattPercent == FORCE_UPDATE_N_READS)
  {
    nNoUpdatesBattPercent = 0;
    last_batt_percent_value = batt_percent_value;
    sendBatteryLevel(batt_percent_value);
    cr2032_wait();
#ifdef F_DEBUG
    Serial.print("Battery Percent:");
    Serial.println(batt_percent_value);
#endif
  }
  else
  {
    nNoUpdatesBattPercent++;
  }
#endif

// Low battery pwr LED blink
#ifdef PWR_LED_PIN
  if (!ext_power)
  {
    if (batt_percent_value <= LOW_BATTERY_THRESHOLD)
    {
      digitalWrite(PWR_LED_PIN, HIGH);
      low_batt_led_on_start_time = millis();
      trigger_pwr_led_update = true;
    }
#ifdef F_DEBUG
    Serial.print("Power LED status (low battery):");
    Serial.println(digitalRead(PWR_LED_PIN));
#endif
  }
#endif

  // Set first run to false
  if (first_run)
  {
    first_run = false;
  }

  // Turn of the PWR LED (low batery)
  if (!ext_power && trigger_pwr_led_update)
  {
    while (millis() < low_batt_led_on_start_time + LOW_BATTERY_BLINK_TIME)
    {
      continue;
    }
    low_batt_led_on_start_time = 0;
    digitalWrite(PWR_LED_PIN, LOW);
    trigger_pwr_led_update = false;
  }

  // Set wake_up_mode to -3 if sleeping is disabled
#ifdef MY_REPEATER_FEATURE
  wake_up_mode = -3;
#endif

// Smartsleep only if MY_REPEATER_FEATURE is not enabled
#ifndef MY_REPEATER_FEATURE
// Turn OFF power PIN before sleeping
#ifdef POWER_PIN
  digitalWrite(POWER_PIN, LOW);
#endif
#ifdef REED_SW_PIN
  if (cycle_counter >= CYCLE_BEFORE_SLEEP)
  {
    cycle_counter = 0;
#ifdef F_DEBUG
    Serial.println("Sleeping");
#endif
    wake_up_mode = smartSleep(digitalPinToInterrupt(REED_SW_PIN), CHANGE, REED_SW_SLEEP_INTERVAL);
  }
  else
  {
#ifdef F_DEBUG
    Serial.print("Finished cycle: ");
    Serial.println(cycle_counter);
#endif
    cycle_counter++;
  }

#endif
#ifndef REED_SW_PIN
#ifdef F_DEBUG
  Serial.println("Sleeping");
#endif
  wake_up_mode = smartSleep(UPDATE_INTERVAL);
#endif
#endif
}