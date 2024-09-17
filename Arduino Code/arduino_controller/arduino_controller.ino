// RGB Fan Controller Arduino code

// BIG CHANGES:
// 1) Remove serial profile sending
// 2) Implement backup to eeprom
// 3) Implement restore from EEPROM + reload conf
// 4) Implement fast FRAM write
// 5) Implement fast RAM read

// Version 143


#include <Wire.h>
//#include <WSWire.h>  // Better I2C library
#include <LCD_I2C.h>
#include <Adafruit_PWMServoDriver.h>
#include <RTClib.h>
#define DECODE_NEC          // Specify decodng protocol, saves loads of memory
#include <IRremote.hpp>

// Some variables that can be changed
const int software_ver = 3;  // Filesystem version
const bool debug = false;  // Enable excessive debug messages

const byte s_ack = 6;  // Acknowledgement byte for serial comms
const byte s_err = 21;  // Error byte for serial comms

#define TWI_FREQ 400000L  // Override default I2C speed

#define btn_bright_up 92
#define btn_bright_down 93
#define btn_pause 65
#define btn_power 64
#define btn_red_1 88
#define btn_green_1 89
#define btn_blue_1 69
#define btn_white_1 68
#define btn_red_2 84
#define btn_green_2 85
#define btn_blue_2 73
#define btn_white_2 72
#define btn_red_3 80
#define btn_green_3 81
#define btn_blue_3 77
#define btn_white_3 76
#define btn_red_4 28
#define btn_green_4 29
#define btn_blue_4 30
#define btn_white_4 31
#define btn_red_5 24
#define btn_green_5 25
#define btn_blue_5 26
#define btn_white_5 27
#define btn_red_up 20
#define btn_green_up 21
#define btn_blue_up 22
#define btn_speed_up 23
#define btn_red_down 16
#define btn_green_down 17
#define btn_blue_down 18
#define btn_speed_down 19
#define btn_diy_1 12
#define btn_diy_2 13
#define btn_diy_3 14
#define btn_auto 15
#define btn_diy_4 8
#define btn_diy_5 9
#define btn_diy_6 10
#define btn_flash 11
#define btn_jump_3 4
#define btn_jump_7 5
#define btn_fade_3 6
#define btn_fade_7 7

#define addr_mux 0x70
#define addr_lcd 0x27
#define addr_eeprom 0x50
#define addr_dac 0x62
#define addr_fram 0x50
#define addr_pwm 0x40
#define addr_rtc 0x68

#define pin_infrared 2
#define pin_rpm 3
#define pin_buzzer 4
#define pin_led 5
#define pin_pwm_enable 6
#define pin_eeprom_wp 7
#define pin_fram_wp 8
#define pin_rpm_inhibit 12
#define pin_rpm_a0 9
#define pin_rpm_a1 10
#define pin_rpm_a2 11
#define pin_builtin_led 13
#define pin_therm_1 A2
#define pin_therm_2 A0
#define pin_therm_3 A6
#define pin_therm_4 A1
#define pin_mux_reset A3
#define pin_ldr A7

// Structs for data
struct task {
  // Scheduling information, default values can be changed
  // Highest priority is chosen when task deadlines clash
  uint8_t function;
  uint8_t priority;
  uint16_t timing;
  uint32_t deadline;
  bool enabled;
} schedule[8] = {
  // Function, priority, timing, deadline, enabled
  {0, 1, 9000, 1500, true},  // Dummy task, default if no tasks enabled
  {1, 6, 100, 1550, true},  // update_led
  {2, 5, 1000, 1600, true},  // adjust_fan
  {3, 4, 150, 1650, true},  // ir_code_update
  {4, 2, 750, 1700, true},  // measure_temps
  {5, 3, 1000, 5000, true},  // lcd_display
  {6, 2, 250, 6000, true},  // read fan RPMs
  {7, 1, 10, 5500, true}  // serial_command
};


// TODO:
// - Allow linking of profiles so that same colour is used for other RGB channels
struct rgb_profile {  // 72 bytes?
  bool power : 1;  // Is RGB channel turned on
  bool pause : 1;  // Is RGB channel paused
  bool use_random : 1; // Use random values for next sequence value
  bool smooth : 1;  // Transition to colours smoothly
  uint8_t link : 2;  // Link to other RGB channels, 2 bits so values 0 (not linked), 1 (link rgb 1), 2 (link rgb 2)
  
  float brightness;  // Brightness multiplier, between 0 and 1
  uint8_t seq_pos;  // Current position in sequence
  uint8_t seq_len;  // Sequence length
  float rate;  // Change rate for transition, between 0 and 1, e.g. 0.1 = 10 steps between colours
  float transition;  // Transition tracking between colours
  uint16_t red[10];  // Red values
  uint16_t green[10];  // Green values
  uint16_t blue[10];  // Blue values
} rgb[3] = {
// pw pa rn smo ln   br   sp sl rat   tr colours
  {1, 0, 0, 0, 0, 0.4, 0, 6, 0.05, 0.0, {4095, 4095, 0, 0, 0, 4095, 0, 0, 0, 0},
                                        {0, 4095, 4095, 4095, 0, 0, 0, 0, 0, 0},
                                        {0, 0, 0, 4095, 4095, 4095, 0, 0, 0, 0}},
  {1, 0, 0, 0, 0, 0.4, 0, 2, 0.05, 0.0, {0, 0, 0, 4095, 0, 0, 0, 0, 0, 0},  // Example of smooth random fade
                                        {0, 0, 0, 4095, 170, 0, 0, 0, 0, 0},
                                        {0, 0, 0, 4095, 0, 170, 0, 0, 0, 0}},
  {1, 0, 0, 0, 0, 0.4, 0, 6, 0.05, 0.0, {4095, 4095, 0, 0, 0, 4095, 0, 0, 0, 0},  // Example of rainbow jump
                                        {0, 4095, 4095, 4095, 0, 0, 0, 0, 0, 0},
                                        {0, 0, 0, 4095, 4095, 4095, 0, 0, 0, 0}},
};

struct fan_profile {  // 32 bytes?
  bool pwm;  // true = pwm (max voltage), false = dc (no PWM)
  bool allow_off;  // true = fan off if temperature below first node, false = does not turn off
  uint16_t max_voltage;  // maximum voltage for fan, such as for 5v compatibility
  uint8_t changerate;  // value to change speed by each update
  uint8_t hysteresis;  // hysteresis in temperature sense, only do something if it changes by 
//                        more than this amount
  int8_t last_temperature;  // records last temperature fan was updated for
  int16_t last_fan_speed;  // records last fan speed, used for smooth speed adjustment
  int16_t target_fan_speed;  // fan speed set by temperature
  int8_t default_temp;  // When temperatures not present, use this as a default value
  // Each temperature multiplied by factor
  // final temperature = sum(multiplied temperatures) / sum(factors)
  // Missing temperatures (PC not connected) and those below -10 (no probe) will be treated as default_temp
  uint8_t factor[8];  // Source temperature factor, bias towards each temperature reading
  int8_t temperature[4];  // Temperature nodes
  int16_t fan_speed[4];  // Fan speed for temperature nodes
} fan[6] = {
// PWM    Off    Volt CR  Hy LT  L_Sp  T_sp  dT   Factor                    Temperature       Fan speed
  {false, true, 4095, 50, 2, 20, 2000, 2000, 50, {1, 2, 0, 0, 0, 0, 0, 0}, {55, 60, 70, 80}, {1600, 1700, 2000, 3500}},
  {false, true, 4095, 50, 2, 20, 2000, 2000, 50, {1, 2, 0, 0, 0, 0, 0, 0}, {55, 60, 70, 80}, {1600, 1700, 2000, 4000}},
  {false, false, 1700, 10, 0, 20, 1400, 1400, 50, {1, 0, 0, 0, 0, 0, 0, 0}, {55, 60, 65, 70}, {600, 900, 1300, 1700}},
  {true, false, 4095, 50, 2, 20, 2000, 2000, 40, {1, 2, 0, 0, 0, 0, 0, 0}, {30, 50, 70, 80}, {1700, 1800, 1850, 1900}},
  {true, false, 4095, 50, 3, 20, 2000, 2000, 50, {2, 2, 2, 2, 0, 0, 0, 0}, {30, 50, 70, 80}, {1000, 2000, 3000, 4000}},
  {true, false, 4095, 50, 3, 20, 2000, 2000, 50, {2, 2, 2, 2, 0, 0, 0, 0}, {30, 50, 70, 80}, {1000, 2000, 3000, 4000}}
};

// Constants
const int mux_pins[] = {pin_rpm_a0, pin_rpm_a1, pin_rpm_a2};
const uint8_t fan_order[] = {6, 7, 0, 5, 3, 4, 1, 2};  // TODO: THIS IS WRONG FIX IT

// FRAM-based variables
uint16_t addr_schedule;
uint16_t addr_gamma;
uint16_t addr_rgb;
uint8_t num_rgb_profiles;
uint16_t addr_fan;
uint8_t num_fan_profiles;
uint16_t addr_strings;
uint8_t strings_length;
uint8_t num_strings;

// Arrays and associated variables
int16_t temperatures[8] = {};  // First 4 are thermistors, last 4 for PC readings
uint32_t fan_rpm[8] = {};  // Array to store fan RPMs
volatile bool rpm_check;  // First trigger may give spurious results, this prevents that
volatile uint32_t fan_time_1 = 0;  // Variables for quick fan RPM measuring
volatile uint32_t fan_time_2 = 0;
volatile uint32_t rpm_current_micros;
volatile uint32_t interrupt_triggers = 0;
int fan_rpm_read = 0;  // Variable to store which fan is being read

// Other variables
uint8_t num_tasks;  // Calculated in setup to identify number of scheduled tasks
uint8_t current_bus;  // Tracks current bus selected in I2C multiplexer
uint8_t display_mode = 1;
uint8_t last_display_mode = 0;
uint16_t light_level = 800;  // Default brightness level, 0 - 1024
uint8_t led_select = 0;  // Which LED channel is selected, 0 = all, 1/2/3 = single RGB channel
bool red_led_override = false;  // Override for red LED, for IR receive light.
uint32_t lcd_timeout = 0;  // Timer for LCD custom message override
uint32_t last_serial_time;  // Used for serial timeout, such as error occurred. Hardcoded 1000ms
uint8_t serial_func;  // Non-blocking serial, records which function currently active
uint8_t serial_sub;  // Used within serial functions to track active section

LCD_I2C lcd(addr_lcd, 16, 2); // Default address of most PCF8574 modules
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//Adafruit_MCP4725 dac;
RTC_PCF8523 rtc;
DateTime current_time;


// TODO: Improve startup, remove 2 second error wait - Display white char for successful init
void setup() {

  bool setup_error = false;
  
  // Set pin modes first to make sure it's all safe
  pinMode(pin_infrared, INPUT);  // IR
  pinMode(pin_rpm, INPUT_PULLUP);  // RPM signal
  pinMode(pin_buzzer, OUTPUT);  // Buzzer
  pinMode(pin_led, OUTPUT);  // Front LED
  pinMode(pin_pwm_enable, OUTPUT);  // PWM Enable
  pinMode(pin_eeprom_wp, OUTPUT);  // EEPROM Write-Protect
  pinMode(pin_fram_wp, OUTPUT);  // FRAM Write-Protect
  pinMode(pin_rpm_inhibit, OUTPUT);  // RPM signal inhibit
  pinMode(pin_rpm_a2, OUTPUT);  // RPM A2
  pinMode(pin_rpm_a1, OUTPUT);  // RPM A1
  pinMode(pin_rpm_a0, OUTPUT);  // RPM A0
  pinMode(pin_builtin_led, OUTPUT);  // Arduino LED
  pinMode(pin_therm_4, INPUT);  // Thermistor 4
  pinMode(pin_therm_2, INPUT);  // Thermistor 2
  pinMode(pin_therm_3, INPUT);  // Thermistor 3
  pinMode(pin_mux_reset, OUTPUT);  // I2C Mux Reset
  // A4 and A5 are I2C pins and don't need to be set here
  pinMode(pin_therm_1, INPUT);  // Thermistor 1
  pinMode(pin_ldr, INPUT);  // LDR

  digitalWrite(pin_builtin_led, HIGH);  // Indicator to let us know the Arduino is starting

  // Set required pins to HIGH. By default, OUTPUTs are low.
  digitalWrite(pin_mux_reset, HIGH);  // I2C Mux resets when this is low, we need it high to work
  digitalWrite(pin_eeprom_wp, HIGH);  // Write-protect EEPROM when high
  digitalWrite(pin_fram_wp, HIGH);  // Write-protect FRAM when high
  digitalWrite(mux_pins[0], HIGH);
  digitalWrite(mux_pins[1], HIGH);
  digitalWrite(mux_pins[2], HIGH);
  digitalWrite(pin_rpm_inhibit, HIGH);  // CD4051 mux needs inhibit high to enable
  
  num_tasks = sizeof(schedule) / sizeof(schedule[0]);  // Calculate number of tasks in schedule
  
  // Set up Serial to communicate with computer if it's listening
  Serial.begin(115200);          // Define baud rate
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println(F("RGB Fan Controller Project"));
  Serial.println(F("--------------------------"));
  
  
  Serial.println(F("Init I2C"));
  delay(10);  // A small delay just to make sure the Mux chip is out of reset
  Wire.begin();

  while (true)
  {
    if (check_exists(addr_mux))
    {
      Serial.println(F("Mux OK"));
      break;
    }
    else
    {
      setup_error = true;
      digitalWrite(pin_led, HIGH);
      Serial.println(F("MUX ERR"));
      delay(1000);  // Keep trying to connect, no Mux means no I2C!
      digitalWrite(pin_led, LOW);
      delay(1000);
    }
  }
  digitalWrite(pin_led, LOW);
  digitalWrite(pin_builtin_led, LOW);

  Serial.println(F("Init LCD"));
  delay(10);

  // - CHECK LCD EXISTS!!!!!

  // There seems to be an issue where using select_bus doesn't allow the LCD to initialise properly
  Wire.beginTransmission(addr_mux);  // TCA9548A address is 0x70
  Wire.write(1 << 0);          // send byte to select bus
  Wire.endTransmission();
  if (not check_exists(addr_lcd)) {
    // It is believed that the LCD library has a default mode if no LCD was detected
    setup_error = true;
  }
  
  lcd.begin(false);
  lcd.backlight();
  lcd_clearline(0);
  lcd.print(F("Initialising..."));

  lcd_clearline(1);
  lcd.print(F("PWM - PCA9685"));
  select_bus(6);
  if ( check_exists(0x40) )
  {
    Serial.println(F("PWM OK"));
    pwm.begin();
    pwm.setPWMFreq(1600);
    pwm.setPWM(9, 0, 3000);  // LCD backlight
//    delay(100);  // Delay just so we get to read the message on the LCD
  }
  else {
    setup_error = true;
    display_error(F("PWM - PCA9685"));
  }

  lcd_clearline(0);
  lcd.print(F("Initialising..."));
  lcd_clearline(1);
  lcd.print(F("FRAM"));
  select_bus(6);
  if ( check_exists(addr_pwm) )
  {
    Serial.println(F("FRAM OK"));
    if (not fs_check()) {
//      setup_error = true;
      display_error(F("FRAM VER"));
    }
//    delay(100);  // Delay just so we get to read the message on the LCD
  }
  else {
    setup_error = true;
    display_error(F("FRAM"));
  }


  int8_t found_dac = -1;
  for (int i = 0; i < 4; i++) {
    select_bus(i + 2);
    if ( check_exists(addr_dac) )
    {
      lcd_clearline(0);
      lcd.print(F("Initialising..."));
      Serial.print(F("DAC "));
      Serial.print(i);
      Serial.println(F(" OK"));
      lcd_clearline(1);
      lcd.print(F("DAC - "));
      lcd.print(i);
      found_dac = i;
    }
    else {
      /////////////////
      // It would be nice to send a nice error message with DAC number
      // But I just can't get it to work. This'll have to suffice.
      setup_error = true;
      display_error(F("DAC      "));
    }
  }

  lcd_clearline(0);
  lcd.print(F("Initialising..."));
  lcd_clearline(1);
  lcd.print(F("EEPROM - 24LC256"));
  select_bus(1);
  if ( check_exists(addr_fram) )
  {
    Serial.println(F("EEPROM OK"));
//    delay(100);  // Delay just so we get to read the message on the LCD
  }
  else {
    setup_error = true;
    display_error(F("EEPROM - 24LC256"));
  }

  lcd_clearline(0);
  lcd.print(F("Initialising..."));
  lcd_clearline(1);
  lcd.print(F("RTC - PCF8523"));
  select_bus(7);
  if ( check_exists(addr_rtc) )
  {
    Serial.println(F("RTC OK"));
    rtc.begin();
    // If the clock is on battery, you may not want the clock to be automatically adjusted
    // Move the adjustment into the IF statement to prevent always updating the time
//    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//    Serial.println(F("RTC set"));
    if (!rtc.initialized() or rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println(F("RTC set"));
      rtc.start();  // Ensure clock is running
      display_error(F("Clock time lost!"));
    }
    rtc.start();  // Ensure clock is running
//    delay(100);  // Delay just so we get to read the message on the LCD
  }
  else {
    setup_error = true;
    display_error(F("RTC - PCF8523"));
  }

  lcd_clearline(1);
  lcd.print(F("IR Receiver"));
  Serial.println(F("Enable IR"));
  IrReceiver.begin(pin_infrared, false);  // Read up on IR enable_led_feedback
//  delay(100);  // Delay just so we get to read the message on the LCD

  lcd_clearline(1);
  lcd.print(F("RPM Interrupt"));
  attachInterrupt(digitalPinToInterrupt(pin_rpm), rpm_interrupt, RISING);
//  delay(100);

  lcd_clearline(1);
  lcd.print(F("Load Config"));
  load_config();

  // If any errors occurred, don't fully start. Sit here and do nothing.
  if (setup_error == true) {
    lcd.print(F("Error"));
    cmd_halt();  // Use this as a halting function. Will send a serial byte but that's ok
  }

  lcd_clearline(1);
  lcd.print(F("Complete!"));
//  delay(100);  // Delay just so we get to read the message on the LCD

  Serial.println(F("Serial ready"));
  
}

void(* resetFunc) (void) = 0;  // A function which causes a crash, triggers software reset

void loop() {
  // Scheduling
  uint8_t soonest_task = 0;  // Tracks task ID with closest deadline
  uint8_t soonest_priority = 0; // Tracks the priority of this task
  int32_t soonest_deadline = 1000;  // Default delay is 1 second if no tasks
  uint8_t tasks_position = 0;  // Tracks position in list, for updating deadline
  uint32_t cur_time = millis();  // Rolls over after 49 days, probably not an issue
  int32_t task_deadline;  // Time to next deadline, can be negative if we're behind

  /*
     Scheduling:
     [task_id, priority, timing, deadline]

     task_id - task number, used to switch function
     priority - tasks with highest priority win in schedule clash
     timing - how often a task should run, in milliseconds
     deadline - when the task is next due to run
  */

  for (uint8_t i = 0; i < num_tasks; i++)  // For each task
  {
    if (schedule[i].enabled) {
      task_deadline = schedule[i].deadline - cur_time; // Calculate time to deadline
  
      // Update 'soonest' if deadline sooner OR if deadline the same and priority higher
      if ((task_deadline < soonest_deadline) or
          (task_deadline == soonest_deadline and schedule[i].priority > soonest_priority))
      {
        soonest_task = schedule[i].function;
        soonest_priority = schedule[i].priority;
        soonest_deadline = task_deadline;
        tasks_position = i;
      }
    }
  }

  if (soonest_deadline >= 0) {
    if (not red_led_override) {
      digitalWrite(pin_led, LOW);
    }
    
    // If we've got time before the next task deadline
    delay(soonest_deadline);  // Sleep until time required
  }
  else if (soonest_deadline > -5) {
    // If we're slightly behind on task deadline, try tweaking the deadline slightly
    // to avoid this happening again. Should spread tasks out so they don't all occur at the
    // same time and cause this to happen.
    schedule[tasks_position].deadline += 1;  // Shift deadline to try and avoid future conflicts
  }
  else {
    digitalWrite(pin_led, HIGH);
    
    // When we're behind, we set the deadline to now, so that it is put safely into the future
    // If we've missed a couple of tasks, rushing them won't achieve much
    // This also helps stop an endless loop of a task failing to catch up
    if (debug) {
      Serial.print(F("Task behind: "));  // We are running behind more than expected!
      Serial.print(tasks_position);
      Serial.print(F(", time: "));
      Serial.println(soonest_deadline);
    }
    schedule[tasks_position].deadline = cur_time;
  }
  
  schedule[tasks_position].deadline += schedule[tasks_position].timing; // Update task deadline

  uint32_t task_time = micros();  // Track how long each task took to execute

  switch (soonest_task)
  {
    case 0: break;  // Default task, do nothing
    case 1:
      update_leds(); break;
    case 2:
      adjust_fans(); break;
    case 3:
      ir_receive(); break;
    case 4:
      measure_sensors(); break;
    case 5:
      update_display(); break;
    case 6:
      read_rpm(); break;
    case 7:
      serial_command(); break;
    default:
      Serial.println(F("Un Func")); break;
  }

  // Function to print out any slow tasks taking over 1ms to complete
  uint32_t task_time_end = micros();
  task_time = task_time_end - task_time;
  if (task_time > 1000 and debug) {
    Serial.print(F("Task uS: "));
    Serial.print(soonest_task);
    Serial.print(F(", "));
    Serial.println(task_time);
  }
}



void serial_command() {
  // Main function for handling serial communications with PC

  byte check_byte;
  byte read_byte;
  
  if (serial_func == 0) {
    if (not red_led_override) {
      digitalWrite(pin_led, LOW);
    }
    
    if (Serial.available() >= 2) {
      // Serial trigger format with error checking
      // check byte which is (255 - function code)
      // read byte for function code
      // When added, result should be 255
      check_byte = Serial.read();
      read_byte = Serial.read();
  
      if (check_byte + read_byte != 255) {
        // If we have an error, send a nack and return
        Serial.write(s_err);
        return;
      }
      else {
        serial_func = read_byte;
        serial_sub = 0;
        digitalWrite(pin_led, HIGH);
        last_serial_time = millis();
      }
    }
    else {
      // Return if there is nothing to do, prevent infinite error in switch statement
    return;
    }
  }
  else {
    if (Serial.available() > 0) {
      // If we have serial communication, update last_serial_time
      last_serial_time = millis();
    }
    else if (millis() > last_serial_time + 1000) {
      // If we haven't had a serial update in 1000ms
      // Force reset the serial so it doesn't just stay stuck
      serial_func = 0;
    }
  }

  switch (serial_func) {
    case 1:  // Halt
      cmd_halt();
      break;
    case 2:  // Alive
      cmd_alive();
      break;
    case 3:  // Load Config
      cmd_load_config();
      break;
    case 4:  // Write FRAM
      cmd_write_fram();
      break;
    case 5:  // Write temperature
      cmd_write_temp();
      break;
//    case 6:  // Write RGB
//      cmd_write_rgb();
//      break;
    case 7:  // Write clock time
      cmd_write_rtc();
      break;
//    case 8:  // Write fan profile
//      cmd_write_fan();
//      break;
    case 9:  // Write LCD message
      cmd_write_lcd();
      break;
    case 10:  // Read RPM
      cmd_read_rpm();
      break;
    case 11:  // Read temperature
      cmd_read_temp();
      break;
    case 12:  // Read FRAM
      cmd_read_fram();
      break;
    case 13:  // Set pins
      cmd_set_pins();
      break;
    case 14:  // FRAM backup
      cmd_backup_fram();
      break;
    case 15:  // FRAM restore
      cmd_restore_fram();
      break;
    case 16:  // FRAM fast write
      cmd_fastwrite_fram();
      break;
    case 17:  // FRAM fast read
      cmd_fastread_fram();
      break;
    default:  // Not recognised
      Serial.write(s_err);
      serial_func = 0;
      break;
  }
}

void cmd_halt() {
  // Infinite loop to halt the Arduino, flashes light to indicate halted
  
  while (true) {
    digitalWrite(pin_led, HIGH);
    delay(500);
    digitalWrite(pin_led, LOW);
    delay(500);
  }
  serial_func = 0;  // Reset serial to begin new function
}

void cmd_alive() {
  Serial.write(s_ack);
  // Return uptime millis, 4 bytes
  send_serial_uint32(millis());
  serial_func = 0;  // Reset serial to begin new function
}

void cmd_write_fram() {
  // Loop of address and value, address is 2 bytes, value is 1 byte
  // send back 0 if successful, 1 if error for each byte (used to ensure each byte is transferred)
  // All bytes 255 255 255, end data write

  uint16_t fram_addr;
  byte fram_data;

//  Serial.print(F("FRAM now data"));
//  Serial.print("F");
  if (serial_sub == 0) {
    Serial.write(s_ack);
    serial_sub = 1;  // This function is now active and we don't need to keep sending ack
  }

  if (Serial.available() >= 3) {
    fram_addr = read_serial_uint16();
    fram_data = Serial.read();

    if (fram_addr == 65535 and fram_data == 255) {
//      Serial.println(F("FRAM END"));
      Serial.write(s_ack);
      serial_func = 0;  // Reset serial to begin new function
      return;
    }
    else {
      // Check if address is too big for FRAM
      if (fram_addr < 32786) {
        fram_write_byte(fram_addr, fram_data);
        Serial.write(s_ack);
      }
      else {
        Serial.write(s_err);
      }
    }
  }
}

void cmd_write_temp() {
  byte read_byte;

  if (serial_sub == 0) {
    Serial.write(s_ack);
    serial_sub = 1;  // This function is now active and we don't need to keep sending ack
  }

  if (Serial.available() >= 4) {
    // Receive 4 int temperature values for temps 4 - 7 (index 0)
    for (int i = 0; i < 4; i++) {
      temperatures[i + 4] = Serial.read();
    }
    Serial.write(s_ack);
    serial_func = 0;  // Reset serial to begin new function
  }
}

void cmd_read_rpm() {
  byte read_byte;
  
//  Serial.print(F("RPM"));
  Serial.write(s_ack);

  for (int i = 0; i < 8; i++) {
    send_serial_uint32(fan_rpm[i]);
  }

  serial_func = 0;  // Reset serial to begin new function
}

void cmd_read_temp() {
//  Serial.print(F("R Temp"));
  Serial.write(s_ack);

  for (int i = 0; i < 8; i++) {
    Serial.write(temperatures[i]);
  }
  Serial.write(s_ack);
  serial_func = 0;  // Reset serial to begin new function
}

void cmd_read_fram() {
//  Serial.print(F("R FRAM"));
  if (serial_sub == 0) {
    Serial.write(s_ack);
    serial_sub = 1;  // This function is now active and we don't need to keep sending ack
  }
  
  // Loop of address, address is 2 bytes
  // send back byte at location followed by int 1, or data 0 and int 0 if error
  // Receive 255 255, end data read
  uint16_t fram_addr;
  byte fram_data;
  
  if (Serial.available() >= 2) {
    fram_addr = read_serial_uint16();
    if (fram_addr == 65535) {
      Serial.write(s_ack);
      serial_func = 0;  // Reset serial to begin new function
      return;
    }
    
    // Fram data should be updated from the fram_read function
    if (fram_addr < 32768) {
      fram_data = fram_read_byte(fram_addr);
//      fram_data = 128;
      Serial.write(fram_data);
      Serial.write(s_ack);
    }
    else {
      Serial.write(0);
      Serial.write(s_err);
    }
    
  }
}

void cmd_write_rtc() {
  // Function to receive 32 bit timestamp for updating the RTC
  if (serial_sub == 0) {
    Serial.write(s_ack);
    serial_sub = 1;
  }

  if (Serial.available() >= 4) {
    //  Serial.print(F("Await time"));
    uint32_t ser_timestamp;
    ser_timestamp = read_serial_uint32();
    select_bus(7);
    rtc.adjust(ser_timestamp);
    Serial.write(s_ack);
    serial_func = 0;
  }
}

void cmd_write_lcd() {
  // Function to write a custom message to the LCD
  // This reads two sets of 16 byte ascii strings and displays them directly to the LCD
  // Strings must be padded to 16 bytes (with spaces)!
  byte read_byte;

  if (serial_sub == 0) {
    Serial.write(s_ack);
    serial_sub = 1;
  }

  if (serial_sub == 1) {
    if (Serial.available() >= 1) {
      // Get timeout first, and set timeout var
      // Timeout is current millis plus seconds (millis * 1000)
      lcd_timeout = millis() + (Serial.read() * 1000);
      Serial.write(s_ack);
      serial_sub = 2;
    }
  }

  // Get and display message

  if (serial_sub == 2) {
    if (Serial.available() >= 16) {
      select_bus(0);
      lcd.setCursor(0,0);
      for (int i = 0; i < 16; i++) {
        read_byte = Serial.read();
        lcd.write(read_byte);
      }
      serial_sub = 3;
      Serial.write(s_ack);
    }
  }

  if (serial_sub == 3) {
    if (Serial.available() >= 16) {
      select_bus(0);
      lcd.setCursor(0,1);
      for (int i = 0; i < 16; i++) {
        read_byte = Serial.read();
        lcd.write(read_byte);
      }
      serial_func = 0;
      Serial.write(s_ack);
    }
  }

  // This will trigger an LCD clear when normal update_display runs again
  last_display_mode = 0;
}

void cmd_load_config() {
  // Function to trigger the load_config function
  Serial.write(s_ack);
  load_config();
  serial_func = 0;
}

void cmd_set_pins() {
  // Function to turn pins on or off
  //  set_pin 1 - pin_pwm_enable 6 - Set LOW to enable PWM (LOW by default)
  //  set_pin 2 - pin_eeprom_wp 7 - Set HIGH to enable Write Protect (HIGH by default)
  //  set_pin 3 - pin_fram_wp 8 - Set HIGH to enable Write Protect (HIGH by default) IS THIS BROKEN????
  //  set_pin 4 - pin_rpm_inhibit 12 - Set LOW to inhibit RPM (Set HIGH by default)
  Serial.write(s_ack);
  
  if (Serial.available() >= 2) {
    uint8_t set_pin = Serial.read();
    byte set_value = Serial.read();

    switch (set_pin) {
      case 1: set_pin = pin_pwm_enable; break;
      case 2: set_pin = pin_eeprom_wp; break;
      case 3: set_pin = pin_fram_wp; break;
      case 4: set_pin = pin_rpm_inhibit; break;
      default: Serial.write(s_err); set_pin = 0; break;
    }
    
    if (set_pin != 0) {
      if (set_value == 0) {
        digitalWrite(set_pin, LOW);
      }
      else {
        digitalWrite(set_pin, HIGH);
      }
      Serial.write(s_ack);
    }
    serial_func = 0;  // Reset serial to begin new function
  }
}

void cmd_backup_fram() {
  // Busses:
  //  1 - EEPROM 0x50
  //  6 - FRAM 0x50   PWM 0x40

  // This will trigger an LCD clear when normal update_display runs again
  last_display_mode = 0;

  // Disable EEPROM Write-Protect allows us to write to it
  digitalWrite(pin_eeprom_wp, LOW);

  // We will only be using max 16 bytes per transaction
  byte buf[16];

  // LCD write backup message on top line
  select_bus(0);
  lcd.clear();
  lcd.print(F("FRAM backup"));
  
  for (uint16_t addr = 0; addr < 32768; addr += 16) {
    // Due to Arduino I2C library limitations, we can do a maximum transfer of 30 bytes (+2 address bytes)
    //   before the buffer is full.
    // EEPROM pages are 64 bytes.
    // Using 16 bytes for simplicity

    // Select FRAM and read bytes into the buffer
    fram_start(addr);
    Wire.endTransmission();
    Wire.requestFrom(addr_fram, 16);
    for (uint8_t i = 0; i < 16; i++) {  // Then get those bytes and place in buffer
      buf[i] = Wire.read();
    }
    Wire.endTransmission();

    // Select EEPROM and write bytes from the buffer
    eeprom_start(addr);
    for (uint8_t i = 0; i < 16; i++) {
      // Write the buffer bytes to EEPROM (max 64 at once, per 24LC256 spec)
      Wire.write(buf[i]);
    }
    Wire.endTransmission();
    delay(5);  // Delay for EEPROM writing (minimum time per spec is 5ms)

    // Update LCD with status:
    select_bus(0);
    lcd.setCursor(0,1);
    lcd.print(addr);
    lcd.print(F(" / 32768"));
  }

  // Re-enable EEPROM Write-Protect so we cannot write to it
  // This will prevent any accidental data corruption
  digitalWrite(pin_eeprom_wp, HIGH);

  // Only send ack when done, comms should pause until this is received
  Serial.write(s_ack);
  serial_func = 0;  // Reset serial to begin new function
}

void cmd_restore_fram() {
  // Busses:
  //  1 - EEPROM 0x50
  //  6 - FRAM 0x50   PWM 0x40

  // This will trigger an LCD clear when normal update_display runs again
  last_display_mode = 0;

  uint16_t addr = 0;

  // Due to Arduino I2C library limitations, we can do a maximum transfer of 30 bytes (+2 address bytes)
  byte buf[30];

  // LCD write backup message on top line
  select_bus(0);
  lcd.clear();
  lcd.print(F("FRAM restore"));
  
  for (addr = 0; addr < 32760; addr += 30) {
    // 32,768 MOD 30 leaves remainder 8 to do at the end
    // Select EEPROM and read bytes into the buffer
    eeprom_start(addr);
    Wire.endTransmission();
    Wire.requestFrom(addr_eeprom, 30);  // After sending address, request bytes
    for (uint8_t i = 0; i < 30; i++) {  // Then get those bytes and place in buffer
      buf[i] = Wire.read();
    }
    Wire.endTransmission();

    // Select FRAM and write bytes from the buffer
    fram_start(addr);
    for (uint8_t i = 0; i < 30; i++) {  // Write the buffer bytes to FRAM
      Wire.write(buf[i]);
    }
    Wire.endTransmission();

    // Update LCD with status:
    select_bus(0);
    lcd.setCursor(0,1);
    lcd.print(addr);
    lcd.print(F(" / 32768"));

    // No delay is required for writing FRAM, it is fast!
    
  }

  eeprom_start(addr);
  Wire.endTransmission();
  Wire.requestFrom(addr_eeprom, 8);  // After sending address, request bytes
  for (uint8_t i = 0; i < 8; i++) {  // Then get those bytes and place in buffer
    buf[i] = Wire.read();
  }

  // Select FRAM and write bytes from the buffer
  fram_start(addr);
  for (uint8_t i = 0; i < 8; i++) {  // Write the buffer bytes to FRAM
    Wire.write(buf[i]);
  }
  Wire.endTransmission();

  // Reload config
  load_config();

  // Only send ack when done, comms should pause until this is received
  Serial.write(s_ack);
  serial_func = 0;  // Reset serial to begin new function
}

void cmd_fastwrite_fram() {
  // Function to write FRAM quickly, sending 16 bytes at a time
  // This is BLOCKING
  // Note: Attempted 32 bytes but due to Serial and I2C buffers being 32 bytes, 32 byte writes do not work

  Serial.write(s_ack);

  byte buf[16];

  for (uint16_t addr = 0; addr < 32768; addr += 16) {
    
    // Wait for 16 bytes to arrive
    uint8_t bytes_read = 0;
    while (bytes_read < 16) {
      if (Serial.available() > 0) {
        buf[bytes_read] = Serial.read();
        bytes_read += 1;
      }
      else {
        delayMicroseconds(500);
      }
    }
    
    // Ready to receive more data, or all data received.
    // May improve speed by allowing computer to send data when FRAM being written
    Serial.write(s_ack);
    
    // Select FRAM and write bytes from the buffer
    fram_start(addr);
    for (uint8_t i = 0; i < 16; i++) {  // Write the buffer bytes to FRAM
      Wire.write(buf[i]);
    }
    Wire.endTransmission();
  }

  serial_func = 0;  // Reset serial to begin new function
}

void cmd_fastread_fram() {
  // Function to read FRAM as quickly as possible, using hardcoded 32 byte buffer to match fram writing
  // This is BLOCKING
  // Unlike FRAM write, the incoming data is not in the same buffer so we can read 32 bytes
  // We can also send 32 bytes together on Serial, unlike I2C

  Serial.write(s_ack);

  byte buf[32];

  for (uint16_t addr = 0; addr < 32768; addr += 32) {

    // Select FRAM and read bytes into the buffer
    fram_start(addr);
    Wire.endTransmission();
    Wire.requestFrom(addr_fram, 32);  // After sending address, request bytes
    for (uint8_t i = 0; i < 32; i++) {  // Then get those bytes and place in buffer
      buf[i] = Wire.read();
    }
    Wire.endTransmission();

    for (uint8_t i = 0; i < 32; i++) {  // Send FRAM bytes to serial
      Serial.write(buf[i]);
    }

    // Wait for PC ack
    while (not Serial.available() >= 1) {
      delay(1);
    }
    if (Serial.read() != s_ack) {
      // Abort if PC sends error
      break;
    }
  }

  serial_func = 0;  // Reset serial to begin new function
  
}


bool fs_check() {
  // This function checks that the FRAM has the correct filesystem on it.

  // Bool to check if FS checks failed, when error found this is set to false
  bool fs_valid = true;

  // Check if version byte matches to global const software_ver
  uint16_t ver_byte;
  uint16_t* ver_byte_ptr = &ver_byte;
  fram_read(0, ver_byte_ptr, 1);
  if (ver_byte != software_ver) {
    fs_valid = false;
  }

  if (fs_valid == true) {
    // Read table location and items, use for fs_locate()
    // Bytes 14 and 15
    if (debug) {
      Serial.println(F("FS OK"));
    }
  }
  else {
    display_error(F("FS Version"));
  }

  return fs_valid;
}

void load_config() {
  // Function to load all config variables from FRAM
  Serial.println("Load config");

  //  8-9 - (int16) Schedule table location
  addr_schedule = fram_read_uint16(8);
  
  //  10-11 - (int16) Gamma correction table location
  addr_gamma = fram_read_uint16(10);
  
  //  12-13 - (int16) RGB profiles location
  addr_rgb = fram_read_uint16(12);
  //  14-14 - (int8) Number of RGB profiles
  num_rgb_profiles = fram_read_uint8(14);
  
  //  15-16 - (int16) Fan profiles location
  addr_fan = fram_read_uint16(15);
  //  17-17 - (int8) Number of fan profiles
  num_fan_profiles = fram_read_uint8(17);

  //  18-19 - (int16) FRAM strings location
  addr_strings = fram_read_uint16(18);
  //  20-20 - (int8) FRAM string length
  strings_length = fram_read_uint8(20);
  //  21-22 - (int8) Number of FRAM strings
  num_strings = fram_read_uint16(21);

  uint8_t profile;
  
  //  23-25 - (3 bytes - 1 byte per profile) Default RGB profiles
  for (uint8_t i = 0; i < 3; i++) {
    profile = fram_read_uint8(23 + i);
    load_rgb_profile(i, profile, true);
  }
  
  //  26-31 - (6 bytes - 1 byte per profile) Default fan profiles
  for (uint8_t i = 0; i < 6; i++) {
    profile = fram_read_uint8(26 + i);
    load_fan_profile(i, profile);
  }

  // UPDATE THESE TO REFLECT PYTHON CODE ====================== ???
  // Low reading, low val, high read, high val

  //  32-47 - (16 bytes - 4 bytes per therm) Thermistor calibration values

  //  48-51 - (4 bytes - 4 bytes per LDR) LDR calibration values ???
  
}


void update_display() {
  // This function controls what is displayed on the LCD
  // Because we expect to use the LCD on I2C bus 0, we set this first


  // TODO ######################################################################
  // Initial LCD mode setup - if coming out of timeout or mode change,
  // then run mode setup (e.g. display static parts of mode)
  // This is an optimisation and will reduce LCD screen update time
  
  
  // This will only update when lcd_timeout is less than the current millis
  if (millis() > lcd_timeout) {

    select_bus(0);
    if (display_mode != last_display_mode) {
      lcd.clear();
      last_display_mode = display_mode;
    }
    lcd.setCursor(0,0);
    
    if (display_mode == 1)  // Display the time
    {
        select_bus(7);
        current_time = rtc.now();
        select_bus(0);
        lcd.setCursor(4,0);
  //      lcd.print(current_time.unixtime());
        if (current_time.hour() < 10) {
          lcd.print(F("0"));
        }
        lcd.print(current_time.hour(), DEC);
        lcd.print(F(":"));
        if (current_time.minute() < 10) {
          lcd.print(F("0"));
        }
        lcd.print(current_time.minute(), DEC);
        lcd.print(F(":"));
        if (current_time.second() < 10) {
          lcd.print(F("0"));
        }
        lcd.print(current_time.second(), DEC);
  
        lcd.setCursor(3,1);
        lcd.print(current_time.year(), DEC);
        lcd.print(F("/"));
        if (current_time.month() < 10) {
          lcd.print(F("0"));
        }
        lcd.print(current_time.month(), DEC);
        lcd.print(F("/"));
        if (current_time.day() < 10) {
          lcd.print(F("0"));
        }
        lcd.print(current_time.day(), DEC);
  
    }

    else if (display_mode == 2) // Display fan RPM
    {
        static uint8_t rpm_display_group;
        lcd.clear();
  
  //      RPM 1 - channel 6 (starting from 0)
  //      RPM 2 - channel 7
  //      RPM 3 - channel 0
  //      RPM 4 - channel 5
  //      RPM 5 - channel 3
  //      RPM 6 - channel 4
        
  //      if (rpm_display_group == 0) {
  //        print_fan_rpm(1, 6, 0);
  //        print_fan_rpm(2, 7, 1);
  //      }
  //      else if (rpm_display_group == 1) {
  //        print_fan_rpm(3, 0, 0);
  //        print_fan_rpm(4, 5, 1);
  //      }
  //      else if (rpm_display_group == 2) {
  //        print_fan_rpm(5, 3, 0);
  //        print_fan_rpm(6, 4, 1);
  //      }
  
        if (rpm_display_group == 0) {
          print_fan_rpm(0, 0, 0);
          print_fan_rpm(1, 1, 1);
        }
        else if (rpm_display_group == 1) {
          print_fan_rpm(2, 2, 0);
          print_fan_rpm(3, 3, 1);
        }
        else if (rpm_display_group == 2) {
          print_fan_rpm(4, 4, 0);
          print_fan_rpm(5, 5, 1);
        }
        
        rpm_display_group += 1;
        if (rpm_display_group > 2) {
          rpm_display_group = 0;
        }
    }
  
    else if (display_mode == 3) // Display temperatures
    {
        // lcd.clear();
//        lcd.print(F("TH1:   "));
        fprint(1, false);
        lcd.setCursor(4,0);
        print_temperature_value(temperatures[0]);
        lcd.setCursor(8,0);
//        lcd.print(F("TH2:   "));
        fprint(2, false);
        lcd.setCursor(12,0);
        print_temperature_value(temperatures[1]);
        lcd.setCursor(0,1);
//        lcd.print(F("TH3:   "));
        fprint(3, false);
        lcd.setCursor(4,1);
        print_temperature_value(temperatures[2]);
        lcd.setCursor(8,1);
//        lcd.print(F("TH4:   "));
        fprint(4, false);
        lcd.setCursor(12,1);
        print_temperature_value(temperatures[3]);
    }

    else if (display_mode == 4) // Display Arduino uptime
    {
        // Get uptime using millis, this should rollover at 49 days
        fprint(63, false);
        lcd.setCursor(1,1);

        uint32_t millis_left = millis();  // Reduced as we calculate things
        uint32_t time_val;  // Used for holding results of calculations

        // Get days
        time_val = millis_left / 86400000;
        millis_left = millis_left % 86400000;
        if (time_val < 10) {
          fprint(66, false);  // fram 66 is zero (0)
        }
        lcd.print(time_val);
        fprint(65, false);

        // Get hours
        time_val = millis_left / 3600000;
        millis_left = millis_left % 3600000;
        if (time_val < 10) {
          fprint(66, false);  // fram 66 is zero (0)
        }
        lcd.print(time_val);
        fprint(64, false);

        // Get minutes
        time_val = millis_left / 60000;
        millis_left = millis_left % 60000;
        if (time_val < 10) {
          fprint(66, false);  // fram 66 is zero (0)
        }
        lcd.print(time_val);
        fprint(64, false);

        // Get seconds
        time_val = millis_left / 1000;
        if (time_val < 10) {
          fprint(66, false);  // fram 66 is zero (0)
        }
        lcd.print(time_val);
    }
  
    else
    {
        display_mode = 1;
    }
  }
}

void print_fan_rpm(uint8_t fan, uint8_t mux_channel, uint8_t row) {
  // Function to help print fan rpm to LCD
  select_bus(0);
  lcd.setCursor(0,row);
//  lcd.print(F("Fan "));
//  lcd.print(fan);
//  lcd.print(": ");
  fprint(10 + fan, false);
  lcd.print(fan_rpm[mux_channel]);
}

void print_temperature_value(int16_t temperature) {
  // Function to format for missing temperature probes
  // Print '--' if temperature below -10c (probe missing = very low reading)
  if (temperature < -10) {
//    lcd.print(F("--"));
    fprint(9, false);
  }
  else {
    lcd.print(temperature);
  }
}


void ir_handler(uint16_t ir_number) {
  switch (ir_number)
  {
    case btn_bright_up:
         fprint(16, true);
         change_led_brightness(true);
         break;
    case btn_bright_down:
         fprint(17, true);
         change_led_brightness(false);
         break;
    case btn_pause: fprint(18, true); led_pause_button(); break;
    case btn_power: fprint(19, true);
      led_power_button();  // and also turn the lights off
//      ir_rgb_profile(0); 
      break;

    case btn_red_1: fprint(20, true);
      ir_rgb_profile(1); break;
    case btn_green_1: fprint(21, true);
      ir_rgb_profile(2); break;
    case btn_blue_1: fprint(22, true);
      ir_rgb_profile(3); break;
    case btn_white_1: fprint(23, true);
      ir_rgb_profile(4); break;

    case btn_red_2: fprint(24, true);
      ir_rgb_profile(5); break;
    case btn_green_2: fprint(25, true);
      ir_rgb_profile(6); break;
    case btn_blue_2: fprint(26, true);
      ir_rgb_profile(7); break;
    case btn_white_2: fprint(27, true);
      ir_rgb_profile(8); break;  // Making the whites up, may be wrong

    case btn_red_3: fprint(28, true);
      ir_rgb_profile(9); break;
    case btn_green_3: fprint(29, true);
      ir_rgb_profile(10); break;
    case btn_blue_3: fprint(30, true);
      ir_rgb_profile(11); break;
    case btn_white_3: fprint(31, true);
      ir_rgb_profile(12); break;

    case btn_red_4: fprint(32, true);
      ir_rgb_profile(13); break;
    case btn_green_4: fprint(33, true);
      ir_rgb_profile(14); break;
    case btn_blue_4: fprint(34, true);
      ir_rgb_profile(15); break;
    case btn_white_4: fprint(35, true);
      ir_rgb_profile(16); break;

    case btn_red_5: fprint(36, true);
      ir_rgb_profile(17); break;
    case btn_green_5: fprint(37, true);
      ir_rgb_profile(18); break;
    case btn_blue_5: fprint(38, true);
      ir_rgb_profile(19); break;
    case btn_white_5: fprint(39, true);
      ir_rgb_profile(20); break;

    case btn_red_up: fprint(40, true); change_led_colour(true, 1); break;
    case btn_green_up: fprint(41, true); change_led_colour(true, 2); break;
    case btn_blue_up: fprint(42, true); change_led_colour(true, 3); break;
    case btn_speed_up: fprint(43, true);
      change_led_rate(true); break;

    case btn_red_down: fprint(44, true); change_led_colour(false, 1); break;
    case btn_green_down: fprint(45, true); change_led_colour(false, 2); break;
    case btn_blue_down: fprint(46, true); change_led_colour(false, 3); break;
    case btn_speed_down: fprint(47, true); 
      change_led_rate(false); break;

    // RGB channel 1 (front case) - pwm 0, 1, 2
    // RGB channel 2 (main case) - pwm 3, 4, 5
    // RGB channel 3 (power and HDD LED) - pwm 6, 7, 8
    case btn_diy_1: fprint(48, true);
      led_select = 3; break;
    case btn_diy_2: fprint(49, true);
      led_select = 1; break;
    case btn_diy_3: fprint(50, true);
      led_select = 2; break;
    case btn_auto: 
      fprint(51, true);
      display_mode += 1;
      break;

    case btn_diy_4:
      fprint(52, true); 
      if (led_select == 1) {
        // If front case RGB, toggle link to front panel
        // DIY 2 = led_select 1 = rgb 0 = front case
        if (rgb[0].link != 1) {
          rgb[0].link = 1;
        }
        else {
          rgb[0].link = 0;
        }
      }
      if (led_select == 2) {
        // If main case RGB, toggle link to front panel
        // DIY 3 = led_select 2 = rgb 1 = main case
        if (rgb[1].link != 1) {
          rgb[1].link = 1;
        }
        else {
          rgb[1].link = 0;
        }
      }
      break;
      
    case btn_diy_5:
      fprint(53, true);
      if (led_select == 2) {
        // If front or main case RGB
        // If link not to panel RGB, set. Otherwise, unset
        // led_select 2 == rgb 1 == main case
        if (rgb[1].link != 2) {
          rgb[1].link = 2;
        }
        else {
          rgb[1].link = 0;
        }
      }
      break;
    
    
    case btn_diy_6: fprint(54, true);
      led_select = 0; break;
    case btn_flash: fprint(55, true);
      ir_rgb_profile(21);
      break;

    case btn_jump_3: fprint(56, true);
      ir_rgb_profile(22); break;
    case btn_jump_7: fprint(57, true); 
      ir_rgb_profile(23); break;
    case btn_fade_3: fprint(58, true);
      ir_rgb_profile(24); break;
    case btn_fade_7: fprint(59, true);
      ir_rgb_profile(25); break;

    default:
      fprint(60, true); break;
  }
}

void change_led_rate(bool rate_up) {
  // Function to increase or decrease RGB sequence speed

  float steps;
  float step_change;
  
  if (led_select == 0) {
    // If all channels, make speed all based on front RGB, profile 2 (0, 1, 2)
    rgb[0].rate = rgb[2].rate;
    rgb[1].rate = rgb[2].rate;
  }
  
  for (int i = 0; i < 3; i++) {
    if ((i + 1) == led_select or led_select == 0) {
      steps = 1 / rgb[i].rate;  // Get the current steps between colours
      if (steps > 10.0) {
        step_change = steps / 7.0;
      }
      else {
        step_change = 1.0;
      }
      
      if (rate_up and steps > 1.0) {
        steps -= step_change;  // Increase steps
      }
      else if (not rate_up and steps < 500.0) {
        // Steps is a float so we have about 5-6 digits of accuracy, could go up to 100,000 or so theoretically
        steps += step_change;  // Decrease steps
      }

      rgb[i].rate = 1.0 / steps;  // Calculate the new rate, likely to be some error but should work

//      // NEW Code - not sure if this actually works
//      if (rate_up and rgb[i].rate < 1.0) {
//        rgb[i].rate = rgb[i].rate * 1.2;
//        if (rgb[i].rate > 1.0) {
//          rgb[i].rate = 1.0;
//        }
//      }
//      if (not rate_up and rgb[i].rate > 0.01) {
//        rgb[i].rate = rgb[i].rate / 1.2;
//        if (rgb[i].rate < 0.01) {
//          rgb[i].rate = 0.01;
//        }
//      }
    }
  }
}

void change_led_brightness(bool bright_up) {
  // Function to increase or decrease LED brightness
  if (led_select == 0) {
    // If all channels, make speed all based on front RGB, profile 2 (0, 1, 2)
    rgb[0].brightness = rgb[2].brightness;
    rgb[1].brightness = rgb[2].brightness;
  }

  for (int i = 0; i < 3; i++) {
    if ((i + 1) == led_select or led_select == 0) {
      if (bright_up and rgb[i].brightness < 1.0) {
        rgb[i].brightness += 0.05;
        if (rgb[i].brightness > 1.0) {
          rgb[i].brightness = 1.0;
        }
      }
      if (not bright_up and rgb[i].brightness > 0.2) {
        rgb[i].brightness = rgb[i].brightness -= 0.05;
        if (rgb[i].brightness < 0.15) {
          rgb[i].brightness = 0.15;
        }
      }
    }
  }
}

void change_led_colour(bool colour_up, uint8_t button_colour) {
  // Function to modify LED colours using the remote colour buttons
  // rate_up - up = true, down = false
  // colour - 1 = red, 2 = green, 3 = blue

  for (int i = 0; i < 3; i++) {
    // For RGB channels
    if ((i + 1) == led_select or led_select == 0) {
      // If channel selected
      // i + 1 is the target channel, 0 is all channels so triggers each time
      
      if (rgb[i].seq_len > 1 and rgb[i].use_random) {
        // If in light sequence with random colours
        // Note - if not random sequence, don't allow changing the sequence
        if (colour_up) {
          // Rate up increases minimum colour brightness
          switch (button_colour) {
            case 1:
              if (rgb[i].red[2] < 4032) {
                rgb[i].red[2] += 64;
              }
              break;
            case 2:
              if (rgb[i].green[2] < 4032) {
                rgb[i].green[2] += 64;
              }
              break;
            case 3:
              if (rgb[i].blue[2] < 4032) {
                rgb[i].blue[2] += 64;
              }
              break;
            default: break;
          }
        }
        else {
          // Rate down decreases minimum colour brightness
          switch (button_colour) {
            case 1:
              if (rgb[i].red[2] >= 64) {
                rgb[i].red[2] -= 64;
              }
              break;
            case 2:
              if (rgb[i].green[2] >= 64) {
                rgb[i].green[2] -= 64;
              }
              break;
            case 3:
              if (rgb[i].blue[2] >= 64) {
                rgb[i].blue[2] -= 64;
              }
              break;
            default: break;
          }
        }
      }
      else if (rgb[i].seq_len == 1) {
        // If displaying a static colour
        if (colour_up) {
          // If rate up, increase the target colour
          switch (button_colour) {
            case 1:
              if (rgb[i].red[rgb[i].seq_pos] < 4032) {
                rgb[i].red[rgb[i].seq_pos] += 64;
              }
              break;
            case 2:
              if (rgb[i].green[rgb[i].seq_pos] < 4032) {
                rgb[i].green[rgb[i].seq_pos] += 64;
              }
              break;
            case 3:
              if (rgb[i].blue[rgb[i].seq_pos] < 4032) {
                rgb[i].blue[rgb[i].seq_pos] += 64;
              }
              break;
            default: break;
          }
        }
        else {
          // If rate down, decrease the target colour
          switch (button_colour) {
            case 1: 
              if (rgb[i].red[rgb[i].seq_pos] >= 64) {
                rgb[i].red[rgb[i].seq_pos] -= 64;
              }
              break;
            case 2: 
              if (rgb[i].green[rgb[i].seq_pos] >= 64) {
                rgb[i].green[rgb[i].seq_pos] -= 64;
              }
              break;
            case 3: 
              if (rgb[i].blue[rgb[i].seq_pos] >= 64) {
                rgb[i].blue[rgb[i].seq_pos] -= 64;
              }
              break;
            default: break;
          }
        }
      }
    }
  }
}

void led_power_button() {
  // Function to turn LEDs on and off with power button, toggles power var

  // If all channels selected, set to front RGB value
  if (led_select == 0) {
    rgb[0].power = rgb[2].power;
    rgb[1].power = rgb[2].power;
  }

  for (int i = 0; i < 3; i++) {
    if ((i + 1) == led_select or led_select == 0) {
      // If channel selected, toggle power
      rgb[i].power = 1 - rgb[i].power;
    }
  }
}

void led_pause_button() {
  // Function to set LED rate to 0 and back with pause button, toggles pause var

  // If all channels selected, set to front RGB value
  if (led_select == 0) {
    rgb[0].pause = rgb[2].pause;
    rgb[1].pause = rgb[2].pause;
  }
  
  for (int i = 0; i < 3; i++) {
    if ((i + 1) == led_select or led_select == 0) {
      // If channel selected, toggle pause
      rgb[i].pause = 1 - rgb[i].pause;
    }
  }
}



void update_leds() {
// Function to update LED colours
// RGB channel 1 (front case) - pwm 0, 1, 2
// RGB channel 2 (main case) - pwm 3, 4, 5
// RGB channel 3 (power and HDD LED) - pwm 6, 7, 8
  
  int target_pos;
  float target_colour[3] = {};
  float current_colour[3] = {};
  uint16_t new_colour[3] = {};
  
  for (int c = 0; c < 3; c++) {
    // For each RGB channel
    if (rgb[c].transition >= 1) {
      // If completed a transition
      // Set sequence position to next colour
      // Reset transition
      rgb[c].seq_pos += 1;
      rgb[c].transition = 0;

      if (rgb[c].seq_pos >= rgb[c].seq_len) {
        // If we reached the end of the sequence, reset sequence position to the start
        rgb[c].seq_pos = 0;
      }

      if (rgb[c].use_random) {
        // If random, copy previous target colour to current colour
        // Then create a new random target colour
        // New target colour will use random between sequence[2] and sequence[3]
        // - This allows for biased randoms, like hues of blue, or pastel shades
        rgb[c].red[0] = rgb[c].red[1];
        rgb[c].green[0] = rgb[c].green[1];
        rgb[c].blue[0] = rgb[c].blue[1];
        rgb[c].red[1] = random(rgb[c].red[2], rgb[c].red[3]);
        rgb[c].green[1] = random(rgb[c].green[2], rgb[c].green[3]);
        rgb[c].blue[1] = random(rgb[c].blue[2], rgb[c].blue[3]);
        rgb[c].seq_pos = 0;
      }
    }

    if (rgb[c].seq_len > 1) {
      // If we have a sequence, get the next colour
      target_pos = rgb[c].seq_pos + 1;
      if (target_pos >= rgb[c].seq_len) {
        // Loop back to the start if we reach the end of the sequence
        target_pos = 0;
      }
      target_colour[0] = rgb[c].red[target_pos];
      target_colour[1] = rgb[c].green[target_pos];
      target_colour[2] = rgb[c].blue[target_pos];
    }
    else {
      // If no sequence, target colour is the same colour as the current colour
      target_colour[0] = rgb[c].red[rgb[c].seq_pos];
      target_colour[1] = rgb[c].green[rgb[c].seq_pos];
      target_colour[2] = rgb[c].blue[rgb[c].seq_pos];
    }

    current_colour[0] = rgb[c].red[rgb[c].seq_pos];
    current_colour[1] = rgb[c].green[rgb[c].seq_pos];
    current_colour[2] = rgb[c].blue[rgb[c].seq_pos];

    if (rgb[c].smooth) {
      // If smooth, calculate blend between colours
      new_colour[0] = current_colour[0] + ((target_colour[0] - current_colour[0]) * rgb[c].transition);
      new_colour[1] = current_colour[1] + ((target_colour[1] - current_colour[1]) * rgb[c].transition);
      new_colour[2] = current_colour[2] + ((target_colour[2] - current_colour[2]) * rgb[c].transition);
      
    }
    else {
      // If not smooth, jump to next colour
      new_colour[0] = target_colour[0];
      new_colour[1] = target_colour[1];
      new_colour[2] = target_colour[2];
    }

    float transition_val = 0.0;
    if (not rgb[c].pause) {
      // Check if RGB channel is paused, if it is not paused we update the transition
      transition_val = rgb[c].rate;
    }
    rgb[c].transition += transition_val;

    // And ignore all of that if we have a link!
    // We want to keep the RGB sources updating so they resume properly

    // If we have processed RGB for front panel
    if (c == 2) {
      // If links for front panel, we'll use our calculated values
      if (rgb[0].link == 1) {
        set_rgb(0, new_colour[0], new_colour[1], new_colour[2]);
      }
      if (rgb[1].link == 1) {
        set_rgb(1, new_colour[0], new_colour[1], new_colour[2]);
      }
    }

    // If we have processed RGB for front case
    if (c == 0) {
      // If main case has link to front case
      if (rgb[1].link == 2) {
        set_rgb(1, new_colour[0], new_colour[1], new_colour[2]);
      }
    }

    // If not linked, set this channels RGB!
    if (not rgb[c].link) {
      set_rgb(c, new_colour[0], new_colour[1], new_colour[2]);
    }
  }

  set_pwm(9, light_level * 4); // LCD Backlight
}

void adjust_fans() {
  // Function to set fan speeds based on temperature/speed nodes in fan profile

  uint16_t sum_temperatures;  // UPDATE: Set to uint16_t instead of int
  uint16_t sum_factors;
  int fan_temperature;
  float ratio;

  for (int fn = 0; fn < 6; fn++) {
    // For each fan
    sum_temperatures = 0;
    sum_factors = 0;
    for (int temp = 0; temp < 8; temp++) {
      // Go through all 8 temperatures
      // Each temperature is multiplied by the respective temperature factor
      if (temperatures[temp] <= 0) {
        // If no temperature reading from probe, use default temperature
        sum_temperatures += fan[fn].default_temp * fan[fn].factor[temp];
      }
      else {
        sum_temperatures += temperatures[temp] * fan[fn].factor[temp];
      }
      // The factors are also summed
      sum_factors += fan[fn].factor[temp];
    }
    
    // Then divided by the number of factors to get the biased average
    fan_temperature = sum_temperatures / sum_factors;

    // Now we move on to calculating fan speed based on the fan_temperature
    // First up, set fan_speed to zero in case the fan should be turned off
    uint16_t target_speed = 0;

    if ((fan_temperature > fan[fn].last_temperature + fan[fn].hysteresis) or
        (fan_temperature < fan[fn].last_temperature - fan[fn].hysteresis)) {
      // If the temperature has changed by more than the hysteresis
      
      if (fan_temperature <= fan[fn].temperature[0]) {
        // If temperature below first node, set target to first node fan speed if it cannot be turned off
        if (not fan[fn].allow_off) {
          target_speed = fan[fn].fan_speed[0];
        }
        // else the fan will be turned off because target_speed is 0
      }
      else if (fan_temperature >= fan[fn].temperature[3]) {
        // If the max temperature node is reached, set target to max speed for that node
        target_speed = fan[fn].fan_speed[3];
      }
      else {
        // Otherwise calculate it from the node list
        for (int i = 0; i < 3; i++) {
          // Check each temperature node
          if (fan_temperature >= fan[fn].temperature[i] and fan_temperature < fan[fn].temperature[i + 1]) {
            // If the fan_temperature falls between two nodes calculate proportion between the nodes
            float t_proportion = fan_temperature - fan[fn].temperature[i];
            float t_range = fan[fn].temperature[i + 1] - fan[fn].temperature[i];
            ratio = t_proportion / t_range;

            // fan_speed calculated from the proportion of temperature between the two nodes.
            target_speed = fan[fn].fan_speed[i] + ((fan[fn].fan_speed[i + 1] - fan[fn].fan_speed[i])
                                                  * ratio);
          }
        }
      }

      fan[fn].target_fan_speed = target_speed;

      // Update the last fan temperature as temperature calculations are complete
      if (fan_temperature > fan[fn].last_temperature) {
        // If the temperature is increasing, subtract the hysteresis
        fan[fn].last_temperature = fan_temperature - fan[fn].hysteresis;
      }
      else {
        // If the temperature is decreasing, add the hysteresis
        fan[fn].last_temperature = fan_temperature + fan[fn].hysteresis;
      }
    }

    // By default, the fan speed we set the fan to will be the target speed
    // Then we check if that speed is increasing too much over the last speed
    uint16_t fan_speed;
    fan_speed = fan[fn].target_fan_speed;
    
    if (fan[fn].target_fan_speed > fan[fn].last_fan_speed + fan[fn].changerate) {
      if (fan[fn].last_fan_speed != 0) {
        // If fan_speed is increasing by more than changerate and it wasn't off, limit it to changerate
        fan_speed = fan[fn].last_fan_speed + fan[fn].changerate;
      }
      else {
        // If fan was off, set it to minimum speed so it ramps up slowly
        fan_speed = fan[fn].fan_speed[0];
      }
    }
    
    else if (fan[fn].target_fan_speed < fan[fn].last_fan_speed - fan[fn].changerate) {
      // If fan_speed is decreasing by more than the changerate, limit it to changerate
      fan_speed = fan[fn].last_fan_speed - fan[fn].changerate;
    }

    if (fan_speed < fan[fn].fan_speed[0] and fan[fn].allow_off) {
      fan_speed = 0;
    }

    // Update last fan speed now that fan speed calculations are complete
    fan[fn].last_fan_speed = fan_speed;

    // OPTIMISE - Only set DAC and PWM fixed values on fan mode change, not every time
    if (fan[fn].pwm or fn >= 4) {
      // PWM enabled, or fans 4 and 5 which only support PWM. Fan 0-3 can be DC
      // If PWM, set voltage to maximum and use fan_speed to control the PWM
      set_dac(fn, fan[fn].max_voltage);

      // Fan PWM channel is fan number + 10
      // i.e. fan 0 uses PWM 10
      // and fan 5 uses PWM 15
      // ISSUE - ATtiny PWM is inverted, so invert fan speed.
      set_pwm(fn + 10, 4095 - fan_speed);
    }
    else {
      // If DC, set PWM to maximum and use fan_speed to control the voltage
      if (fan_speed > fan[fn].max_voltage) {
        fan_speed = fan[fn].max_voltage;
      }
      set_dac(fn, fan_speed);
      // Set PWM to maximum
      set_pwm(fn + 10, 4095);
    }
  }
}


void measure_sensors() {
  // Function to update temperatures of all thermistors
  // Map integer readings to celcius based on previous experiments, may need adjusting
  temperatures[0] = map(analogRead(pin_therm_1), 128, 830, 0, 100);
  temperatures[1] = map(analogRead(pin_therm_2), 128, 830, 0, 100);
  temperatures[2] = map(analogRead(pin_therm_3), 128, 830, 0, 100);
  temperatures[3] = map(analogRead(pin_therm_4), 128, 830, 0, 100);

  light_level = analogRead(A7);

//  Serial.println(F("Thermistor readings:"));
//  for (int i = 0; i < 4; i++) {
//    Serial.println(temperatures[i]);
//  }
}

void read_rpm() {
  //   Set address of the CD4051 mux chip to measure different fan's RPM
//  digitalWrite(pin_rpm_a0, bitRead(fan_rpm_read, 0));
//  digitalWrite(pin_rpm_a1, bitRead(fan_rpm_read, 1));
//  digitalWrite(pin_rpm_a2, bitRead(fan_rpm_read, 2));

  fan_rpm_read += 1;
  if (fan_rpm_read > 7) {
    fan_rpm_read = 0;
  }
  
  if (fan_time_1 != 0 and fan_time_2 != 0) {
    // If successful RPM reading
    uint32_t rpm_time = fan_time_2 - fan_time_1;

    if (rpm_time > 0) {
      // rpm = 60,000,000 microseconds [1 second] / rpm_time [microseconds for revolution]
      // Fans provide two pulses per revolution, so we need to half this
      // Therefore we'll use 30,000,000
      fan_rpm[fan_rpm_read] = 30000000 / rpm_time;
    }

    if (debug) {
      Serial.print(F("RPM ints: "));
      Serial.println(interrupt_triggers);
      for (int debug_rpm = 0; debug_rpm < 8; debug_rpm++) {
        Serial.println(fan_rpm[debug_rpm]);
      }
    }
  }
  else {
    // If not successful reading RPM, set to 0
    fan_rpm[fan_rpm_read] = 0;
  }

  // Set mux channel for each loop (occurs after each RPM reading)
  for (int i = 0; i < 3; i++) {
    // bitRead returns bit value for number at certain position
    byte mux_bit = bitRead(fan_order[fan_rpm_read], i);
    digitalWrite(mux_pins[i], mux_bit);
  }

  // Reset counters so new RPM can be measured
  // Because the interrupt can happen at any time, it's important to disable interrupts
  // temporarily. We need to do this as quickly as possible.
  noInterrupts();
  rpm_check = false;
  fan_time_1 = 0;
  fan_time_2 = 0;
  interrupt_triggers = 0;
  // Re-attach interrupt to get it working again
  interrupts();

}

void rpm_interrupt() {
  // Interrupt handler for measuring fan RPM
  // This function needs to run exceedingly quickly so as to not block the infra-red interrupt
  // To make measurements quick, we record the microseconds between two pulses
  // rpm_check serves to discard the first partial RPM signal when changing channels
  interrupt_triggers += 1;
  rpm_current_micros = micros();
  if (rpm_check == false) {
    rpm_check = true;
  }
  else if (fan_time_1 == 0) {
    fan_time_1 = rpm_current_micros;
  }
  else if (fan_time_2 == 0) {
    fan_time_2 = rpm_current_micros;
  }
}

// TODO: Find a good way to detect power back on
void power_off_reset() {
  // Function to wait until power returns
  // At this point, only the Arduino will be powered

  Serial.println(F("Shutdown"));
  
  while (true) {
      delay(1000);
  }
  
  // Reset the arduino
  resetFunc();
}


void set_rgb(uint8_t channel, uint16_t r, uint16_t g, uint16_t b) {
  // Function to set RGB and put brightness checks in place for power lights

  // Set brightness
  float brightness_val = 0.0;
  // Check if RGB channel is on, if it is, use specified brightness, otherwise it will be dark
  if (rgb[channel].power) {
    brightness_val = rgb[channel].brightness;
  }
  r = r * brightness_val;
  g = g * brightness_val;
  b = b * brightness_val;
  
  // Check minimum brightness for power LEDs
  if (channel == 2) {
    uint16_t max_brightness;
    // Set max to red, then compare green and blue
    max_brightness = r;
    if (g > max_brightness) {
      max_brightness = g;
    }
    if (b > max_brightness) {
      max_brightness = b;
    }

    if (max_brightness < 700) {
      max_brightness = 700 - max_brightness;  // Re-use max_brightness variable as offset
      r += max_brightness;
      g += max_brightness;
      b += max_brightness;
    }
  }

  set_pwm((channel * 3) + 0, r);
  set_pwm((channel * 3) + 1, g);
  set_pwm((channel * 3) + 2, b);
}

void set_pwm(uint8_t channel, uint16_t pwm_val) {
  // Function to set PWM channel values
  
  //  pwm.setPWM(pin, 4096, 0);  // pin fully on
  //  pwm.setPWM(pin, 0, 4096);  // pin fully off
  //  pwm.setPWM(channel, on_tick, off_tick)

  // Channel 9 is LCD backlight, set minimum brightness
  if (channel == 9) {
    if (pwm_val < 110) {
      pwm_val = 110;
    }
  }

  // Gamma correction for LEDs only
  if (channel < 9) {
    uint16_t lookup_val;
    // select_bus(6) done automatically by fram_read, pwm and fram on same channel
    lookup_val = fram_read_uint16(addr_gamma + (pwm_val * 2));
    select_bus(6);
    pwm.setPWM(channel, 0, lookup_val);
  }
  else {
    select_bus(6);
    pwm.setPWM(channel, 0, pwm_val);
  }
}

void set_dac(uint8_t dac_number, uint16_t voltage) {
  // Function to quickly write DAC outputs
  // Manual I2C
  // Data writing: Control bits, rest of MSB, LSB
  // To write DAC in fastmode, 2 bytes written:
  // 00 (C1 and C2 for fast-write operation)
  // 00 (No power-down bits, for normal operation)
  // XXXX (Most significant 4 bits)
  // XXXX XXXX (Least significant 8 bits)

  // Select bus is fan number (starting from 0), fan 0 = bus 2, so add 2 for each fan
  select_bus(dac_number + 2);
  Wire.beginTransmission(0x62);
  Wire.write((voltage >> 8) & 0x0f);  // Bitshift high bits and ensure first 4 bits are 0 with AND
  Wire.write(voltage & 0xff);  // Least significant bits
  Wire.endTransmission();
}

void ir_receive() {
  // Function to receive IR codes, passes them to the handler for processing
  if (IrReceiver.decode())
  {
    uint16_t ir_code = IrReceiver.decodedIRData.command;
//    IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
    if (ir_code != 0)
    {
      digitalWrite(pin_led, HIGH);
      red_led_override = true;
//      Serial.print(F("IR code: "));
//      Serial.println(ir_code);
      ir_handler(ir_code);
    }
    IrReceiver.resume();
  }
  else {
    digitalWrite(pin_led, LOW);
    red_led_override = false;
  }
}

void select_bus(uint8_t bus) {
  // Select bus of TCA9548A
  //  0 - LCD 0x27
  //  1 - EEPROM 0x50
  //  2 - DAC_1 0x62
  //  3 - DAC_2 0x62
  //  4 - DAC_3 0x62
  //  5 - DAC_4 0x62
  //  6 - FRAM 0x50   PWM 0x40
  //  7 - RTC 0x68

  if (bus != current_bus)  // If we need to change the bus
  {
    Wire.beginTransmission(addr_mux);  // TCA9548A address is 0x70
    Wire.write(1 << bus);          // send byte to select bus
    Wire.endTransmission();
    current_bus = bus;  // Update the current bus *important*
  }
}

bool check_exists(int address) {
  // Function to check an I2C device exists on the currently selected bus
  // Warning - the bus must be selected in the mux first!
  byte error;
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error == 0)
  {
    return true;
  }
  else
  {
    Serial.print(F("I2C ERR: "));
    Serial.println(error);
    return false;
  }
}

void lcd_clearline(uint8_t row) {
  // Function to clear one line of LCD, rather than the whole thing
  // Clears row and returns cursor to start of specified row
  select_bus(0);
  lcd.setCursor(0, row);
  for (int i = 0; i<16; i++) {
    lcd.print(" ");  // A blank character to clear display (a 'space')
  }
  lcd.setCursor(0, row);  // Go back to start of the row
}

void display_error(String dev_name) {
  // Displays an error message and prints dev_name on second row
  // Also sends error on Serial
  
  digitalWrite(pin_led, HIGH);
  digitalWrite(pin_buzzer, HIGH);
  Serial.print(F("I2C ERR: "));
  Serial.println(dev_name);
  select_bus(0);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("I2C ERROR:"));
  lcd.setCursor(0,1);
  lcd.print(dev_name);
  delay(1500);
  digitalWrite(pin_buzzer, LOW);
  digitalWrite(pin_led, LOW);
}


void ir_rgb_profile(uint8_t profile) {
  // Function to simplify settings RGB profiles from IR remote
  // Prevents the need to identify which profiles need to be changed

  for (int i = 0; i < 3; i++) {
    // For RGB channels
    if ((i + 1) == led_select or led_select == 0) {
      // i + 1 is the target channel, 0 is all channels so triggers each time
      load_rgb_profile(i, profile, false);
    }
  }
}

void load_rgb_profile(uint8_t channel, uint8_t profile, bool full_load) {
  // Function to load RGB profile from FRAM
  // Reads from a set base address and calculates offset using 'profile'
  // 77 bytes

  uint16_t address = addr_rgb + (profile * 77);  // PREDEFINED LOCATION FOR RGB PROFILE STORAGE

  if (profile >= num_rgb_profiles) {
    // Valid profile range, remote has 25 profiles, 5 for user?
    // If outside of this range, abort
    return;
  }

  // Load vars
  rgb[channel].link = fram_read_uint8(address);
  rgb[channel].use_random = fram_read_uint8(address + 1);
  rgb[channel].smooth = fram_read_uint8(address + 2);
  rgb[channel].seq_pos = fram_read_uint8(address + 7);
  rgb[channel].seq_len = fram_read_uint8(address + 8);
  rgb[channel].transition = fram_read_float(address + 13);

  if (full_load) {
    // If we want to read the full profile and not maintain current brightness or rate
    rgb[channel].brightness = fram_read_float(address + 3);
    rgb[channel].rate = fram_read_float(address + 9);
  }

  // Load RGB arrays
  for (int i = 0; i < 10; i++) {
    rgb[channel].red[i] = fram_read_uint16(address + 17 + (i * 6));
    rgb[channel].green[i] = fram_read_uint16(address + 19 + (i * 6));
    rgb[channel].blue[i] = fram_read_uint16(address + 21 + (i * 6));
  }
}

void load_fan_profile(uint8_t channel, uint8_t profile) {
  // Function to load fan profile from FRAM
  // Reads from a base offset and calculates address using profile

  uint16_t address = addr_fan + (profile * 32);  // PREDEFINED LOCATION FOR FAN PROFILE STORAGE

  if (profile >= num_fan_profiles) {
    // Valid profile range, allowing 2 profiles for each fan channel
    // If not valid, abort
    return;
  }

  fan[channel].pwm = fram_read_uint8(address);
  fan[channel].allow_off = fram_read_uint8(address + 1);
  fan[channel].max_voltage = fram_read_uint16(address + 2);
  fan[channel].changerate = fram_read_uint8(address + 4);
  fan[channel].hysteresis = fram_read_uint8(address + 5);
  fan[channel].last_temperature = fram_read_uint8(address + 6);
  fan[channel].last_fan_speed = fram_read_uint16(address + 7);
  fan[channel].target_fan_speed = fram_read_uint16(address + 9);
  fan[channel].default_temp = fram_read_uint8(address + 11);

  for (int i = 0; i < 8; i++) {
    fan[channel].factor[i] = fram_read_uint8(address + 12 + (i));
  }

  for (int i = 0; i < 4; i++) {
    fan[channel].temperature[i] = fram_read_uint8(address + 20 + (i * 3));
    fan[channel].fan_speed[i] = fram_read_uint16(address + 21 + (i * 3));
  }
}


void fprint(uint8_t string_num, bool serial) {
  // Function to print null-terminated FRAM string to LCD or Serial
  // Read FRAM byte, check if null, if not then print
  // Caution: Serial strings may need to include newline char
  // Note: Could be optimised by reading in 4 byte chunks

  uint16_t address;
  uint16_t max_addr;
  byte f_data;

  if (string_num >= num_strings) {
    // If string is out of range, use default error string instead
    string_num = 0;
  }

  address = addr_strings + (string_num * strings_length);
  max_addr = address + strings_length;
  
  while (address < max_addr) {
//    select_bus(6);
    f_data = fram_read_byte(address);
    
    address += 1;
    
    if (f_data != 0) {
      // If not a null character signalling end of string
      if (serial) {
//          Serial.print(f_data[i]);
        Serial.write(f_data);
      }
      else {
        select_bus(0);  // Select LCD
//        lcd.print(f_data);
        lcd.write(f_data);
      }
    }
    else {
//      return;
      break;
    }
  }
  
  if (serial) {
    // Finish serial print with a newline
    Serial.println();  // Newline
  }
  else {
    // If we were printing to LCD, we want to keep LCD bus selected
    select_bus(0);
  }
//  Serial.println();
}


void fram_start(uint16_t address) {
  // Select FRAM I2C bus
  select_bus(6);
  // Talk to FRAM and send address
  Wire.beginTransmission(addr_fram);
  Wire.write(address >> 8);  // Most significant bit
  Wire.write(address & 0xff);  // Least significant bit
}

uint8_t fram_read_uint8(uint16_t address) {
  uint8_t read_val;
  
  fram_start(address);
  Wire.endTransmission();
  // After sending address, request bytes
  Wire.requestFrom(addr_fram, 1);

  read_val = Wire.read();

  return read_val;
}

uint16_t fram_read_uint16(uint16_t address) {
  uint8_t bytes[2] = {};
  uint16_t value;
  
  fram_start(address);
  Wire.endTransmission();
  // After sending address, request bytes
  Wire.requestFrom(addr_fram, 2);

  for (int i = 0; i < 2; i++) {
    bytes[i] = Wire.read();
  }

  memcpy(&value, bytes, 2);  // Copy array bytes to uint16 memory location
  return value;

}

float fram_read_float(uint16_t address) {
  uint8_t bytes[4] = {};
  float value;
  
  fram_start(address);
  Wire.endTransmission();
  // After sending address, request bytes
  Wire.requestFrom(addr_fram, 4);

  for (int i = 0; i < 4; i++) {
    bytes[i] = Wire.read();
  }

  memcpy(&value, bytes, 4);  // Copy array bytes to uint16 memory location
  return value;
}

void fram_write_uint8(uint16_t address, uint8_t value) {
  fram_start(address);

  Wire.write(value);

  Wire.endTransmission();
}

void fram_write_uint16(uint16_t address, uint16_t value) {
  fram_start(address);

  Wire.write(value & 255);
  Wire.write((value >> 8) & 255);

  Wire.endTransmission();
}

void fram_write_float(uint16_t address, float value) {
  // https://forum.arduino.cc/t/serial-write-a-float-value/110198/5
  // May write bytes backwards?
  byte * bits = (byte *) &value;

  fram_start(address);

  Wire.write(bits, 4);

  Wire.endTransmission();
}

byte fram_read_byte(uint16_t fram_address) {
  // Function to easily read a single byte of FRAM

  fram_start(fram_address);
  Wire.endTransmission();
  
  // After sending address, request bytes
  Wire.requestFrom(addr_fram, 1);

  if (Wire.available()) {
    return(Wire.read());
  }
  else if (debug) {
    Serial.println(F("FRAM read failed"));
  }
}

void fram_read(uint16_t fram_address, uint16_t* buf_pointer, int bytes) {
  // Code to read FRAM into an array specified by pointer, and number of bytes

  fram_start(fram_address);
  Wire.endTransmission();
  
  // After sending address, request bytes
  Wire.requestFrom(addr_fram, bytes);
  // For each byte that comes in, write to next address in array
  for (int i = 0; i < bytes; i++) {
    if (Wire.available()) {
      *(buf_pointer + i) = Wire.read();
    }
    else if (debug) {
        Serial.println(F("FRAM read failed"));
    }
  }
}

void fram_write_byte(uint16_t fram_address, byte fram_data) {
  // Function to easily write a single byte to FRAM

  fram_start(fram_address);

  Wire.write(fram_data);

  Wire.endTransmission();
}

void fram_write(uint16_t fram_address, uint16_t* buf_pointer, int bytes) {
  // Code to write an array specified by pointer to the FRAM devices

  fram_start(fram_address);

  // Write all bits to FRAM
  for (int i = 0; i < bytes; i++) {
    Wire.write(*(buf_pointer + i));
  }
  
  Wire.endTransmission();
}


void eeprom_start(uint16_t address) {
  // Select EEPROM I2C bus
  select_bus(1);
  // Talk to EEPROM and send address
  Wire.beginTransmission(addr_eeprom);
  Wire.write(address >> 8);  // Most significant bit
  Wire.write(address & 0xff);  // Least significant bit
}



float read_serial_float() {
  // Function to read 4 byte floats from Serial and return float variable
  // https://forum.arduino.cc/t/how-to-covert-4-bytes-to-float/612320
  uint8_t bytes[4] = {};  // Array to store bytes
//  static_assert(sizeof(float) == 4, "float expected to be 4 bytes");
  float v;  // Float to store value in
  for (int i = 0; i < 4; i++) {  // Read serial 4 bytes into array
    bytes[i] = Serial.read();
  }
  memcpy(&v, bytes, 4);  // Copy array bytes to float memory location
  return v;
}

uint16_t read_serial_uint16() {
  // Function to read 2 byte int16s from Serial and return int16 variable
  uint8_t bytes[2] = {};  // Array to store bytes
  uint16_t v;
  for (int i = 0; i < 2; i++) {  // Read serial 2 bytes into array
    bytes[i] = Serial.read();
  }
  memcpy(&v, bytes, 2);  // Copy array bytes to uint16 memory location
  return v;
}

uint32_t read_serial_uint32() {
  // Function to read 4 byte int32s from Serial and return int32 variable
  uint8_t bytes[4] = {};  // Array to store bytes
  uint32_t v;
  for (int i = 0; i < 4; i++) {  // Read serial 2 bytes into array
    bytes[i] = Serial.read();
  }
  memcpy(&v, bytes, 4);  // Copy array bytes to uint16 memory location
  return v;
}
//
void send_serial_uint16(uint16_t value) {
  // Function to send an int16 as two bytes over Serial
  Serial.write(value & 255);
  Serial.write((value >> 8) & 255);
}

void send_serial_uint32(uint32_t value) {
  // Function to send an int32 as four bytes over Serial
  Serial.write(value & 255);
  Serial.write((value >> 8) & 255);
  Serial.write((value >> 16) & 255);
  Serial.write((value >> 24) & 255);
}
