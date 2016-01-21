#include <LiquidCrystal.h>
#include <Wire.h>

#include "LiquidCrystal_I2C.h"

#include <stdlib.h>
#include <ctype.h>

#undef DEBUG
#define LED_PIN         13
#define LDR0_PIN        0


LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 lines display

// --------------------------------

void blink() {
      digitalWrite(LED_PIN, HIGH);
      delay(100 /*ms*/);
      digitalWrite(LED_PIN, LOW);
}

// --------------------------------

void setup_lcd() {
  Serial.println("LCD init");
  lcd.init();
  lcd.display();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("AutoShunt Demo");
}

void setup_ldr() {
  lcd.setCursor(0,2);
  lcd.print("Light:");
  pinMode(A0, INPUT);  // seems to be default and not really needed
}

void setup_time() {
  lcd.setCursor(0,3);
  lcd.print("Uptime:");
}

void setup_nce() {
  Serial1.begin(9600 /*SERIAL_8N1*/);
}

void setup() {
    setup_lcd();
    setup_ldr();
    setup_time();
    setup_nce();
}

// -------------------------------

void print_number(int col, int row, int value, int len) {
  String s(value);
  lcd.setCursor(col, row);
  for (int l = s.length(); l < len; l++) {
    lcd.print(" ");
  }
  lcd.print(s);  
}


void loop_time() {
long mil = millis();
  int sec = mil / 1000;
  int dec = (mil / 100) % 10;
  String s(sec);
  lcd.setCursor(20 - 2 - s.length(), 3);
  lcd.print(s);
  lcd.print('.');
  lcd.print(dec);
}


#define N_LDR_WIN 8
int ldr_win[N_LDR_WIN];
int ldr_win_idx = 0;
int cmd = 0;
long cmd_millis = 0;

void loop_ldr() {
  int a = analogRead(LDR0_PIN);
  print_number(6, 2, a, 4);

  ldr_win[ldr_win_idx++] = a;
  if (ldr_win_idx == N_LDR_WIN) { ldr_win_idx = 0; }

  int avg = 0;
  for (int i = 0; i < N_LDR_WIN; i++) { avg += ldr_win[i]; }
  avg /= N_LDR_WIN;
  
  print_number(11, 2, avg, 4);
  int inc = a - avg;
  print_number(16, 2, inc, 4);  
  if (inc > 100) { cmd = 1; }
  else if (inc < 100) { cmd = 2; }
}

#define LOCO 1538
int light = 0;
byte cmd_buf[10];
int pos = 0;

void process_cmd() {
  if (light != cmd) {
    long now_millis = millis();
    if (cmd_millis > now_millis) { return; }
    cmd_millis = now_millis + 2000; // next cmd in 5 seconds at least
    light = cmd;

    int LOCO_H = 0xC0 + (LOCO >> 8);
    int LOCO_L = LOCO & 0xFF;

    cmd_buf[0] = 0xA2;
    cmd_buf[1] = LOCO_H;
    cmd_buf[2] = LOCO_L;
    cmd_buf[3] = 0x07;
    cmd_buf[4] = light == 1 ? 0x10 : 0;
    Serial1.write(cmd_buf, 5);

    if (Serial1.available()) {
      char c = Serial1.read();
      lcd.setCursor(pos, 1);
      lcd.print(c);
      pos = (pos+1) % 20;
      lcd.setCursor(pos, 1);
      lcd.print(' ');
    }
  }
}

void loop() {
    delay(250);
    blink();
    loop_time();
    loop_ldr();
    process_cmd();
}

