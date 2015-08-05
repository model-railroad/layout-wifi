#include <stdio.h>
#include <string.h>

// -----------------------------------------------
#define __TEST

static int _failures = 0;

void _expect(int line, 
        long expected, const char *exp_str, 
        long actual, const char *act_str, 
        const char *msg) {
    if (expected != actual) {
        printf("[%4d] Expected %s [%d], Actual %s [%d]%s%s\n", 
            line, 
            exp_str, expected, 
            act_str, actual, 
            msg == NULL ? "" : ": ",
            msg == NULL ? "" : msg);
        _failures++;
    }
}

void _expect(int line, 
        const char *expected, const char *exp_str, 
        const char *actual,   const char *act_str, 
        const char *msg) {
    if (strcmp(expected, actual) != 0) {
        printf("[%4d] Expected %s [%s], Actual %s [%s]%s%s\n", 
            line, 
            exp_str, expected, 
            act_str, actual, 
            msg == NULL ? "" : ": ",
            msg == NULL ? "" : msg);
        _failures++;
    }
}

#define EXPECT(a, b) _expect(__LINE__, a, # a, b, # b, NULL)
#define EXPECT_MSG(a, b, msg) _expect(__LINE__, a, # a, b, # b, msg)

long _next_millis = 1000;
char _digital_writes[1024];
unsigned long _digital_read = 0;

// -----------

class String {
public:
};

class DigiFi {
public:
    void println(const char *s) {}
    void begin(int aBaud = 9600, bool en = false) {}
    void setDebug(bool debugStateVar) {}
    bool ready() { return true; }
    String server(uint16_t port) { return String(); }

    int available( void ) {
        // return first value then zero. This stops the process_wifi() loop.
        int a = _available;
        _available = 0;
        return a;
    }

    int readBytes(char *buf, size_t size) {
        EXPECT(_expected_readBytes_len, size);
        memcpy(buf, _readBytes_data, _readBytes_reply);
        return _readBytes_reply; 
    }

    size_t write(const uint8_t *buf, size_t size) {
        _write_len = size;
        memcpy(_write_data, buf, size);
        _write_data[size] = 0;
        return size;
    }
    
    size_t _available;
    size_t _readBytes_reply;
    size_t _expected_readBytes_len;
    char   _readBytes_data[1024];
    char   _write_data[1024];
    size_t _write_len;
};

// -----------
class _MockSerial {
public:
    static void print(const char *s) {}
    static void println(const char *s) {}
    static void println(String &s) {}

    static void begin(int aBaud = 9600) {}
    static size_t available() { return 0; }
};

_MockSerial Serial;

// -----------------------------------------------

#define LOW  0
#define HIGH 1
#define INPUT 0
#define INPUT_HIGH 1
#define OUTPUT 2

long millis() { return _next_millis; }
void delay(size_t ms) {}
void pinMode(int pin, int mode) {}
int digitalRead(int);
void digitalWrite(int, int);

// -----------------------------------------------

#include "LayoutWifi.ino"

// -----------------------------------------------

int digitalRead(int pin) { 
    pin -= SENSOR_PIN_1;
    if (pin < 0) {
        EXPECT_MSG(0, pin, "Invalid pin for digitalRead");
    }
    return ((_digital_read >> pin) & 0x01) ? HIGH : LOW;
}

void digitalWrite(int pin, int value) { 
    int n = strlen(_digital_writes);
    sprintf(_digital_writes + n, "%c%03d,",
        value == HIGH ? 'H' : (value == LOW ? 'L' : '?'),
        pin);
}

void _reset() {
    _next_millis = 1000;
    _digital_read = 0;
   *_digital_writes = 0;
    _wifi._available = 0;
    _wifi._expected_readBytes_len = 0;
    _wifi._readBytes_reply = 0;
   *_wifi._readBytes_data = 0;
    _wifi._write_len = 0;
   *_wifi._write_data = 0;
}

// -----------------------------------------------

void test_blink() {
    printf("- TEST: blink\n");
    _reset();
    blink();
    EXPECT("H013,L013,", _digital_writes);
}

void test_trip_relay() {
    printf("- TEST: trip_relay\n");
    _reset();
    trip_relay(0);
    trip_relay(RELAY_N - 1);
    EXPECT("H090,L090,H097,L097,", _digital_writes);
}

void test_receive_info() {
    printf("- TEST: receive info\n");
    _reset();
    _wifi._expected_readBytes_len = 1023;
    strcpy(_wifi._readBytes_data, "@I\n");
    _wifi._available = 2;
    _wifi._readBytes_reply = 2;

    process_wifi();

    EXPECT(9, _wifi._write_len);
    EXPECT("@IT04S01\n", _wifi._write_data);
    EXPECT("H013,L013,", _digital_writes); // blink    
}

void test_receive_turnout_normal() {
    printf("- TEST: receive turnout normal\n");
    _reset();
    _wifi._expected_readBytes_len = 1023;
    strcpy(_wifi._readBytes_data, "@T01N\n");
    _wifi._available = 5;
    _wifi._readBytes_reply = 5;

    process_wifi();

    EXPECT(6, _wifi._write_len);
    EXPECT("@T01N\n", _wifi._write_data);
    EXPECT("H013,L013,H090,L090,", _digital_writes); // blink + trip_relay
}

void test_receive_turnout_reverse() {
    printf("- TEST: receive turnout reverse\n");
    _reset();
    _wifi._expected_readBytes_len = 1023;
    strcpy(_wifi._readBytes_data, "@T01R\n");
    _wifi._available = 5;
    _wifi._readBytes_reply = 5;

    process_wifi();

    EXPECT(6, _wifi._write_len);
    EXPECT("@T01R\n", _wifi._write_data);
    EXPECT("H013,L013,H091,L091,", _digital_writes); // blink + trip_relay
}

void test_poll_sensors() {
    printf("- TEST: poll sensors\n");

    _reset();
    _next_millis = 1000;
    _digital_read = 0x0000;
    poll_sensors();

    EXPECT(1500, _sensors_ms);
    EXPECT(9, _wifi._write_len);
    EXPECT("@S010000\n", _wifi._write_data);
    EXPECT("H013,L013,", _digital_writes); // blink

    // not past the 1500 ms mark yet, no action
    _reset();
    _next_millis = 1300;
    _digital_read = 0x0001;
    poll_sensors();

    EXPECT(1500, _sensors_ms);
    EXPECT(0, _wifi._write_len);
    EXPECT("", _wifi._write_data);
    EXPECT("", _digital_writes);

    _reset();
    _next_millis = 1800;
    _digital_read = 0x0001;
    poll_sensors();

    EXPECT(2300, _sensors_ms);
    EXPECT(9, _wifi._write_len);
    EXPECT("@S010001\n", _wifi._write_data);
    EXPECT("H013,L013,", _digital_writes); // blink
}

int main(int argc, char **argv) {
    printf("- Start\n");
    EXPECT_MSG(1, argc, "No program arguments accepted.");

    test_blink();
    test_trip_relay();
    test_receive_info();
    test_receive_turnout_normal();
    test_receive_turnout_reverse();
    test_poll_sensors();
    
    printf("- End. %d failures%s\n",
        _failures,
        _failures == 0 ? " => SUCCESS." : "");
    return 0;
}
