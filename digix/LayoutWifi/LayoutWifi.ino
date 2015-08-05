#ifndef __TEST
#include "RalfDigiFi.h"
#endif  // !__TEST
#include <ctype.h>

#define LED_PIN     13
#define RELAY_PIN_1 90
#define TURNOUT_N    4
#define RELAY_N     (2 * TURNOUT_N)
#define RELAY_NORMAL( T) (2 * (T))
#define RELAY_REVERSE(T) ((2 * (T)) + 1)

#define SENSOR_PIN_1   107
#define SENSOR_N         1
#define AIU_N            1

DigiFi _wifi;

unsigned long _sensors_ms = 0;
char          _buf_1024[1024];
unsigned int  _sensors[AIU_N];
unsigned char _turnouts[TURNOUT_N];

const char *hex = "0123456789ABCDEF";

#define TURNOUT_NORMAL  'N'
#define TURNOUT_REVERSE 'R'

#define READ_BIT( I, N) (((I) &  (1<<N)) >> N)
#define SET_BIT(  I, N)  ((I) |  (1<<N))
#define CLEAR_BIT(I, N)  ((I) & ~(1<<N))

// --------------------------------

void blink();
void trip_relay(int);


void setup_pause() {
    // DigiX trick - since we are on serial over USB wait for character to be
    // entered in serial terminal
    // Delay start by up to 5 seconds unless a key is pressed.
    // This should give enough to reprogram a bad behaving program.
    for (int i = 0; i < 5 && !Serial.available(); i++) {
        Serial.println("Enter any key to begin");
        blink();
        delay(1000);
    }
}

void setup_wifi() {
    Serial.begin(9600); 
    _wifi.setDebug(true);
    _wifi.begin(9600);

    Serial.println("Starting");

    while (_wifi.ready() != 1) {
        Serial.println("Error connecting to network");
        delay(15000);
    }  

    Serial.println("Connected to wifi!");
    Serial.print("Server running at: ");
    String address = _wifi.server(8080); //sets up server and returns IP
    Serial.println(address); 
}

void setup_sensors() {
    int j = 0;
    for (int aiu = 0; aiu < AIU_N; aiu++) {    
        unsigned int s = 0;
        for (int i = 0; i < 14 && j < SENSOR_N; i++, j++) {
            if (digitalRead(SENSOR_PIN_1 + i) == HIGH) {
                s = SET_BIT(s, i);
            }
        }
        _sensors[aiu] = s;
    }
}

void setup_relays() {
    // initialize digital pin 90-97 as an output.
    for (int i = 0; i < RELAY_N; i++) {
        pinMode(RELAY_PIN_1 + i, OUTPUT);
        digitalWrite(RELAY_PIN_1 + i, LOW);
    }
    
    for (int i = 0; i < TURNOUT_N; i++) {
        _turnouts[i] = TURNOUT_NORMAL;
        trip_relay(RELAY_NORMAL(i));
    }
}

void setup() {
    setup_pause();
    setup_wifi();
    setup_sensors();
    setup_relays();
}

// -------------------------------

void blink() {
      digitalWrite(LED_PIN, HIGH);
      delay(100 /*ms*/);
      digitalWrite(LED_PIN, LOW);
}

void trip_relay(int index /* 0..RELAY_N-1 */) {
    digitalWrite(RELAY_PIN_1 + index, HIGH);
    delay(100);
    digitalWrite(RELAY_PIN_1 + index, LOW);
}

void process_wifi() {
    int n;
    while (_wifi.available() > 0 && (n = _wifi.readBytes(_buf_1024, 1023)) > 0) {
        if (n > 1023) { n = 1023; }
        char *buf = _buf_1024;
        buf[1023] = 0;
        Serial.print("recv:_");
        Serial.print(buf);
        Serial.println("_");
        if (n >= 2 && *(buf++) == '@') {
            blink();
            char cmd = *(buf++);

            if (cmd == 'I' && n == 2) {
                // Info command: @I\n
                Serial.println("Info Cmd");
                // Reply: @IT<00>S<00>\n
                *(buf++) = 'T';
                *(buf++) = '0' + TURNOUT_N / 10;
                *(buf++) = '0' + TURNOUT_N % 10;
                *(buf++) = 'S';
                *(buf++) = '0' + AIU_N / 10;
                *(buf++) = '0' + AIU_N % 10;
                *(buf++) = '\n';
                _wifi.write((const uint8_t*)_buf_1024, buf - _buf_1024);

            } else if (cmd == 'T' && n == 5) {
                // Turnout command: @T<00><N|R>\n
                int turnout = ((buf[0] - '0') << 8) + (buf[1] - '0');
                char direction = buf[2];
                if ((direction == TURNOUT_NORMAL || direction == TURNOUT_REVERSE)
                        && turnout > 0 && turnout <= TURNOUT_N) {
                    Serial.println("Accepted Turnout Cmd");
                    turnout--; // cmd index is 1-based but array & relays are 0-based
                    if (_turnouts[turnout] != direction) {
                        trip_relay(direction == TURNOUT_NORMAL ? RELAY_NORMAL(turnout) 
                                                               : RELAY_REVERSE(turnout));
                        _turnouts[turnout] = direction;
                        // Reply is the same command
                        _buf_1024[5] = '\n';
                        _wifi.write((const uint8_t*)_buf_1024, 6);
                    }
                } else {
                    Serial.println("Rejected Turnout Cmd");
                }
            }
        }
    }
}

void poll_sensors() {
    if (_sensors_ms > millis()) {
        return;
    }
    _sensors_ms = millis() + 500;   // twice per second
    blink();

    int j = 0;
    for (int aiu = 0; aiu < AIU_N; aiu++) {    
        unsigned int s = _sensors[aiu];
        for (int i = 0; i < 14 && j < SENSOR_N; i++, j++) {
            if (digitalRead(SENSOR_PIN_1 + i) == HIGH) {
                s = SET_BIT(s, i);
            } else {
                s = CLEAR_BIT(s, i);
            }
        }
        _sensors[aiu] = s;

        char *buf = _buf_1024;
        *(buf++) = '@';
        *(buf++) = 'S';
        *(buf++) = '0';
        *(buf++) = '1' + aiu;        
        *(buf++) = hex[(s >> 12) & 0x0F];
        *(buf++) = hex[(s >>  8) & 0x0F];
        *(buf++) = hex[(s >>  4) & 0x0F];
        *(buf++) = hex[(s >>  0) & 0x0F];
        *(buf++) = '\n';
        _wifi.write((const uint8_t*)_buf_1024, buf - _buf_1024);
    }    
}

void loop() {
    process_wifi();
    poll_sensors();
}
