#include "adc.h"
#include "gpio.h"

#include "cabin_controller.h"
#include "emergency_input.h"
#include "contact_charger.h"
#include "voltage_control.h"
#include "child_lock_control.h"
#include "serial_receiver.h"
#include "debug_print.h"
#include "landing_lever.h"
#include "android_handler.h"
#include "pwm_devices.h"
#include "led_indication.h"
#include "linear_actuator.h"

void setup ()
{
//  timer_start_tim (); // Timer delay startup
  init_pwm ();
  clearAllOutput ();
}

void loop ()
{
  device_health_indication();
  monitor_wifi ();
  Emergency_Read ();
  checkChildLock ();
  displayCCicon ();
  checkbatteryStatus ();
//  monitor_ll();
  monitor_pwm();
  transmit_cabin_msg ();
  process_la_sequence();
}


