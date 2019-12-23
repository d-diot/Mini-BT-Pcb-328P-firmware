// Debug configuration

#define MY_DEBUG
#define F_DEBUG
//#define MY_DEBUG_VERBOSE_SIGNING

// Batteries configuration

// AAA batteries = 1, CR2032 = 2
#define BATTERY_TYPE 2
#define ENABLE_BATTERY_MONITOR
#define BATTERY_PERCENT_TOLERANCE 2
// AAA batteries chemistry: 0 = Autodetect, 1 = Custom V min and V max, 2 = NiMH (rechargeable), 3 = Alkaline (disposable)
#define AAA_BATT_CHEMISTRY 0
const float NIMH_VMAX_THRESHOLD = 2.6;
const float NIMH_V_MIN = 2.0;
const float NIMH_V_MAX = 2.4;
const float ALK_V_MIN = 2.0;
const float ALK_V_MAX = 3.0;
const float CR2032_V_MIN = 2.4;
const float CR2032_V_MAX = 3.0;
const float CUSTOM_V_MIN = 1.8;
const float CUSTOM_V_MAX = 3.3;
// Milliseconds to wait after radio module activity (necessary only with CR2032 batteries)
#define CR2032_RADIO_WAIT_TIME 400

// Power LED parameters

#ifdef PWR_LED_PIN
// Brightness level: 0 - 255. Auto = -1. Any number different from: -1, 0 - 255 = LED OFF
const int LOW_BATTERY_LED_BRIGHTNESS = -1;
// Battery percentage threshold to activate the LED
#define LOW_BATTERY_THRESHOLD 25
// Blink time (ms)
#define LOW_BATTERY_BLINK_TIME 300
#endif

// Booster configuration

#ifdef BOOSTER_PIN
#define ENABLE_BOOSTER_MONITOR
// booster policy: ALWAYS OFF = 0, ALWAYS ON = 1, AUTO = 2
#define BOOSTER_POLICY 2
// Vcc min voltage to activate the booster when BOOSTER_POLICY = 2
const float BoostThreshold = 2.7;
#endif

// Ext power monitor

#define ENABLE_EXT_PWR_MONITOR
#ifdef PWR_LED_PIN
// Brightness level: 0 - 255. Auto = -1. Any number different from: -1 or 0 - 255 = LED OFF
const int EXT_POWER_LED_BRIGHTNESS = -1;
#endif

// Vcc read configuration

#define ENABLE_VCC_MONITOR
#define MEAN_VCC_READS 3
// Measured Vcc by multimeter divided by reported Vcc
const float VccCorrection = 1.0 / 1.0;
const float VccTol = 0.05;

// Heartbeat

#define ENABLE_HEARTBEAT


// Power pin parameters

#ifdef POWER_PIN
#define POWER_PIN_WAIT_TIME 300
#endif