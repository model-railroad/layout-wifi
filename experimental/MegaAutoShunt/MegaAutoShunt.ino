#include <LiquidCrystal.h>
#include <Wire.h>

#include "LiquidCrystal_I2C.h"

#include <stdlib.h>
#include <ctype.h>

#undef DEBUG
#define LED_PIN         13
#define LDR0_PIN        0
#define BUTTON1_PIN     30
#define RELAY1_PIN      52
#define RELAY2_PIN      53

#define RELAY_DEFAULT   LOW

#define SENSOR0_THRESHOLD 700
#define LOCO 113

#define LOCO_H (0xC0 + (LOCO >> 8))
#define LOCO_L (LOCO & 0xFF)

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 lines display
byte cmd_buf[10];

// --------------------------------

long next_blink = 0;
void blink() {
  digitalWrite(LED_PIN, HIGH);
  delay(100 /*ms*/);
  digitalWrite(LED_PIN, LOW);
  next_blink = millis() + 1000; // 1s
}

void loop_blink() {
  long now = millis();
  if (next_blink < now) {
    blink();
  }
}

// --------------------------------

void setup_blink() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void setup_pause() {
  for (int i = 0; i < 4; i++) {
    blink();
    delay(250);
  }
}


void setup_lcd() {
  Serial.println("LCD init");
  lcd.init();
  lcd.display();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("AutoShunt Demo #");
  String s(LOCO);
  lcd.print(s);
}

void setup_ir() {
  lcd.setCursor(0,2);
  lcd.print("IR:");
  pinMode(A0, INPUT_PULLUP);
  digitalWrite(A0, HIGH);
}

void setup_time() {
  lcd.setCursor(0,3);
  lcd.print("Uptime:");
}

void setup_nce() {
  Serial1.begin(9600 /*SERIAL_8N1*/);
}

void setup_ssr() {
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, RELAY_DEFAULT);
  digitalWrite(RELAY2_PIN, RELAY_DEFAULT);
}

void setup_buttons() {
  // Note: when using a real button, connect to GND and use INPUT_PULLUP
  // in which case the low state means the button is pressed.
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
}

void setup() {
    setup_ssr();
    setup_blink();
    setup_pause();
    setup_lcd();
    setup_ir();
    setup_buttons();
    setup_time();
    setup_nce();
}

// -------------------------------

// Direction: 0=Normal (IN1), 1=Reverse (IN2)
void set_turnout(int direction) {
  int pin = direction ? RELAY2_PIN : RELAY1_PIN;
  digitalWrite(pin, RELAY_DEFAULT == HIGH ? LOW : HIGH);
  delay(150); // ms
  digitalWrite(pin, RELAY_DEFAULT);
}

void print_number(int col, int row, int value, int len) {
  String s(value);
  lcd.setCursor(col, row);
  for (int l = s.length(); l < len; l++) {
    lcd.print(" ");
  }
  lcd.print(s);  
}


long next_time = 0;
void loop_time() {
  long mil = millis();
  if (next_time < mil) {
    next_time = mil + 100; // 0.5s
    
    int sec = mil / 1000;
    int dec = (mil / 100) % 10;
    String s(sec);
    lcd.setCursor(20 - 2 - s.length(), 3);
    lcd.print(s);
    lcd.print('.');
    lcd.print(dec);
  }
}

// -------------------------------

#define N_LDR_WIN 8
int ldr_win[N_LDR_WIN];
int ldr_win_idx = 0;

int sensor0_activated = 0;

void loop_ir() {
  int a = analogRead(LDR0_PIN);
  print_number(3, 2, a, 4);

  ldr_win[ldr_win_idx++] = a;
  if (ldr_win_idx == N_LDR_WIN) { ldr_win_idx = 0; }

  int avg = 0;
  for (int i = 0; i < N_LDR_WIN; i++) { avg += ldr_win[i]; }
  avg /= N_LDR_WIN;

  if (avg < SENSOR0_THRESHOLD) {
    sensor0_activated = 1;
  }
  
  print_number(8, 2, avg, 4);
}

int is_sensor0_activated() {
  int i = sensor0_activated;
  if (i != 0) {
    sensor0_activated = 0;
  }
  return i;
}

// -------------------------------

int pressed = 0;

void loop_button() {
  int b = digitalRead(BUTTON1_PIN);
  if (b == LOW) {
    pressed = 1;
  }
}

int is_button_pressed() {
  int i = pressed;
  if (i != 0) {
    pressed = 0;
  }
  return i;
}

void block_till_button_released() {
  do {
    loop_button();
  } while (is_button_pressed());
}

// -------------------------------

void _cmd_loco_A2(int op, int data) {
  cmd_buf[0] = 0xA2;
  cmd_buf[1] = LOCO_H;
  cmd_buf[2] = LOCO_L;
  cmd_buf[3] = op;
  cmd_buf[4] = data;
  Serial1.write(cmd_buf, 5);
  blink();

  // A2 command replies with 1 byte: ! for OK, or 1 if address is incorrect
  lcd.setCursor(19, 1);
  if (Serial1.available()) {
    char c = Serial1.read();
    lcd.print(c);
  }
  blink();
}

int current_light_F0;

void cmd_light(int light_state) {
  current_light_F0 = (light_state == 1 ? 0x10 : 0);
  _cmd_loco_A2(0x07, current_light_F0);
}

void cmd_move_28(int speed28) {
  if (speed28 < 0) {
    _cmd_loco_A2(0x01, -speed28);
  } else {
    _cmd_loco_A2(0x02, speed28);
  }
}

void cmd_estop() {
  _cmd_loco_A2(0x06, 0);
}

void cmd_set_speed_mode(int speed_28_or_128) {
  cmd_buf[0] = 0x8D;
  cmd_buf[1] = LOCO_H;
  cmd_buf[2] = LOCO_L;
  cmd_buf[3] = speed_28_or_128 == 28 ? 2 : 3;
  Serial1.write(cmd_buf, 4);
  blink();

  // 8D replies with 2 bytes: ! (ok) or 1 (invalid addr) or 3 (invalid data) + speed mode 0..3
  if (Serial1.available()) {
    Serial1.read();
    blink();
    Serial1.read();
  }
  blink();
}

// -------------------------------
// State

#define STATE_START 0
#define STATE_MOVE_FORWARD 1  // move till past sensor A0
#define STATE_TOGGLE_TURNOUT 2
#define STATE_MOVE_REVERSE 3
#define STATE_PAUSE 4
#define STATE_PAUSE 4

#define SPEED_MODE 128
#define SPEED_FWD 20
#define SPEED_REV 15
#define DELAY_REV 7
#define DELAY_PAUSE 4

int state = 0;
int after_trigger_state = 0;
long after_trigger_delay = 0;
long delay_state_until = 0;
int turnout_state = 0;
int last_print_state = -1;

void print_msg(const char *s) {
  lcd.setCursor(0, 1);
  lcd.print(s);
}

void print_state() {
  if (state != last_print_state) {
    print_number(16, 2, state, 3);
    last_print_state = state;
  }
}

void loop_state() {
  // Estop if button is pressed when running
  if (state >= 10 && is_button_pressed()) {
    print_msg("ESTOP!");
    cmd_estop();
    block_till_button_released();
    is_sensor0_activated(); // clear sensor state
    delay_state_until = 0;
    state = 0;
    return;
  }

  if (delay_state_until != 0) {
    if (delay_state_until > millis()) {
      return;
    } else {
      delay_state_until = 0;
    }
  }
  
  switch(state) {
    case 0:
      print_msg("Must block sensor!");
      state = 1;
      break;
    case 1:
      if (is_sensor0_activated()) {
        state = 2;
      }
      break;
    case 2:
      // Start with button pressed and sensor activated
      //         01234567890123456789
      print_msg("Use btn to start   ");
      is_button_pressed(); // clear button state
      state = 3;
      break;
    case 3:
      // Wait for button and sensor activated
      if (!is_sensor0_activated()) {
        state = 0;
      } else if (is_button_pressed()) {
        state = 10;
        block_till_button_released();
        is_sensor0_activated(); // clear sensor state
      }
      break;

    // ---
    case 10:
      // Start moving forward
      cmd_set_speed_mode(SPEED_MODE);
      cmd_light(1);
      cmd_move_28(SPEED_FWD);
      print_msg("Forward            ");
      state = 11;
      break;
    case 11:
      if (!is_sensor0_activated()) {
        cmd_move_28(0);
        print_msg("Stop               ");
        state = 12;
      }
      break;
    case 12:
      // Toggle turnout to whatever other state
      turnout_state = !turnout_state;
      set_turnout(turnout_state);
      state = 13;
      break;
    case 13:
      // Starting to reverse. Must have a trigger on/off event first.
      cmd_light(1);
      cmd_move_28(-SPEED_REV);
      print_msg("Reverse            ");
      state = 100;
      after_trigger_state = 14;
      after_trigger_delay = DELAY_REV * 1000;
      break;
    case 14:
      cmd_move_28(0);
      print_msg("Stop & Wait        ");
      state = 15;
      delay_state_until = millis() + DELAY_PAUSE *1000;
      break;
    case 15:
      // Starting forward. Must have a trigger on/off event first.
      cmd_light(1);
      cmd_move_28(SPEED_FWD);
      print_msg("Forward            ");
      state = 100;
      after_trigger_state = 16;   // Loop to initial state 11
      after_trigger_delay = 2 * 1000;
      break;
    case 16:
      cmd_move_28(0);
      state = 11;   // Loop to initial state 11
      delay_state_until = millis() + DELAY_PAUSE *1000;
      break;

    // --- trigger: sensor is off, must of on then off again
    case 100:
      if (is_sensor0_activated()) {
        state = 101;
      }
      break;
    case 101:
      if (!is_sensor0_activated()) {
        state = after_trigger_state;
        if (after_trigger_delay != 0) {
          delay_state_until = millis() + after_trigger_delay;
          after_trigger_delay = 0;
        }
      }
      break;
    
  }
}

// -------------------------------


void loop() {
    loop_blink();
    loop_time();
    loop_ir();
    loop_button();
    loop_state();
    print_state();
}

