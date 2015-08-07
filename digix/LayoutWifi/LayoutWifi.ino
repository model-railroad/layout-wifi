#include <stdlib.h>
#include <ctype.h>

#if defined(ARDUINO_ARCH_SAM)
#include "RalfDigiFi.h"
#endif

#define LED_PIN     13

// There are 2 relays (one normal, one reversed) per turnout.
// There are 2 cards of 8 relays, so we can have a max of 8 relays
// and we need 16 output pins: 90-105.
#define RELAY_PIN_1 90
#define TURNOUT_N    8
#define RELAY_N     (2 * TURNOUT_N)
#define RELAY_NORMAL( T) (2 * (T))
#define RELAY_REVERSE(T) ((2 * (T)) + 1)

// For sensors, we can use the 22-52 pins, which means up to 30 sensors.
// However the first 2 AIUs are "virtual" (they represent turnout states)
// and so only AIUs 3 and 4 are real sensors, for a total of 28 pins so
// from pin 22 up to pin 22+28-1=49
#define SENSOR_PIN_1   22
#define FIRST_REAL_AIU  3
#define AIU_N           4
#define SENSORS_PER_AIU 14
#define SENSOR_N       (AIU_N * SENSORS_PER_AIU)

DigiFi _wifi;

unsigned long _sensors_ms = 0;
char          _buf_1024[1024];
unsigned int  _last   [AIU_N];
unsigned int  _sensors[AIU_N];
char          _buf_itoa[12];

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
    int pin = SENSOR_PIN_1;
    for (int aiu = 0; aiu < AIU_N; aiu++) {    
        _last[aiu] = 0x0FFFF;
        unsigned int s = 0;
        for (int i = 0; i < SENSORS_PER_AIU && j < SENSOR_N; i++, j++) {
            if (aiu >= FIRST_REAL_AIU-1) {
                if (digitalRead(pin++) == HIGH) {
                    s = SET_BIT(s, i);
                }
            }
        }
        _sensors[aiu] = s;
    }
}

void setup_relays() {
    // initialize digital pin 90-105 as an output, set all turnouts in normal
    for (int t = 0, aiu = 0, i = 0; t < TURNOUT_N; t++, i++) {
        if (i == SENSORS_PER_AIU) {
            aiu++;
            i = 0;
        }
        _sensors[aiu] = CLEAR_BIT(_sensors[aiu], i);
        
        int n = RELAY_NORMAL(t);
        int r = RELAY_REVERSE(t);

        pinMode(RELAY_PIN_1 + n, OUTPUT);
        digitalWrite(RELAY_PIN_1 + n, LOW);

        pinMode(RELAY_PIN_1 + r, OUTPUT);
        digitalWrite(RELAY_PIN_1 + r, LOW);
        
        trip_relay(n);
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
        // n counts the number of input characters, including the \n
        if (n > 1023) { n = 1023; }
        char *buf = _buf_1024;
        buf[n] = 0;
        Serial.print("recv:_");
        snprintf(_buf_itoa, 12, "%d", n);
        Serial.print(_buf_itoa);
        Serial.print("_");
        Serial.print(buf);
        Serial.println("_");
        if (n >= 3 && *(buf++) == '@') {
            blink();
            char cmd = *(buf++);

            if (cmd == 'I' && n == 3) {
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

            } else if (cmd == 'T' && n == 6) {
                // Turnout command: @T<00><N|R>\n
                int turnout = ((buf[0] - '0') << 8) + (buf[1] - '0');
                char direction = buf[2];
                if ((direction == TURNOUT_NORMAL || direction == TURNOUT_REVERSE)
                        && turnout > 0 && turnout <= TURNOUT_N) {
                    Serial.println("Accepted Turnout Cmd");
                    turnout--; // cmd index is 1-based but array & relays are 0-based

                    int aiu = turnout / SENSORS_PER_AIU;
                    int index = turnout % SENSORS_PER_AIU;
                    unsigned int state = (_sensors[aiu] >> index) & 1;
                    unsigned int desired  = 0;
                    if (direction == TURNOUT_REVERSE) {
                        desired = 1;
                    }
            
                    if (state != desired) {
                        if (direction == TURNOUT_NORMAL) {
                            trip_relay(RELAY_NORMAL(turnout));
                            _sensors[aiu] = CLEAR_BIT(_sensors[aiu], index);
                        } else {
                            trip_relay(RELAY_REVERSE(turnout));
                            _sensors[aiu] = SET_BIT(_sensors[aiu], index);
                        }
                    }
                    // Reply is the same command
                    _buf_1024[5] = '\n';
                    _wifi.write((const uint8_t*)_buf_1024, 6);
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
        int pin = SENSOR_PIN_1;
        for (int i = 0; i < SENSORS_PER_AIU && j < SENSOR_N; i++, j++) {
            if (aiu >= FIRST_REAL_AIU-1) {
                if (digitalRead(pin++) == HIGH) {
                    s = SET_BIT(s, i);
                } else {
                    s = CLEAR_BIT(s, i);
                }
            }
        }
        _sensors[aiu] = s;
        if (s != _last[aiu]) {
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
            _last[aiu] = s;
        }
    }    
}

void loop() {
    process_wifi();
    poll_sensors();
}
