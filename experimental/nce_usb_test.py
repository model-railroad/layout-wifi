#!/usr/bin/python
# Program to test communicating directly with NCE USB over COM port
# Requires: PySerial: pip install pyserial (http://pyserial.readthedocs.org/)

import serial
import time

# COM8 if using Python Windows, /dev/com8 or /dev/ttyS7 if using Cygwin, /dev/ttyS2 on Linux
COM = "/dev/ttyS7"

LOCO = 4014
LOCO_H = 0xC0 + (LOCO >> 8)
LOCO_L = LOCO & 0xFF

def test0(ser):
    # NCE USB 0x80 should always reply with a single !, 0x8C with 3 bytes ! \n\r
    N = 10000
    good = 0
    for i in range(0, N):
        n = ser.write(bytearray([0x80]))
        if n == 1:
            v = ser.read()
            if v == '!':
                good += 1
    print "0x80 stats: ", good, "good out of", N

    
def test_check_version(ser):
    # NCE USB 0xAA return C/S software version
    n = ser.write(bytearray([0xAA]))
    print "0xAA Write reply", n
    time.sleep(0.01)
    v = ser.read(size=3)  # 3 bytes
    print "0xAA Read reply", repr(v)
    time.sleep(0.01)


def test_send_dummies(ser):
    # NCE USB 0x80 should always reply with a single !, 0x8C with 3 bytes ! \n\r

    n = ser.write(bytearray([0x80]))
    print "0x80 Write reply", n
    time.sleep(0.01)
    v = ser.read()  # 1 byte
    print "0x80 Read reply", repr(v)
    time.sleep(0.01)

    n = ser.write(bytearray([0x8C]))
    print "0x8C Write reply", n
    time.sleep(0.01)
    v = ser.read(size=3)
    print "0x8C Read reply", repr(v)
    time.sleep(0.01)


def test_headlight_F4_on_off(ser):
    # NCE USB 0xA2 <adr H/L> 07 10|00 (4=F0/light on/off), reply 1 byte status
    for state in [0x10, 0x00]:
        n = ser.write(bytearray([0xA2, LOCO_H, LOCO_L, 0x07, state]))
        print "0xA2 light Write reply", n
        time.sleep(0.01)
        v = ser.read()  # 1 byte
        print "0xA2 light Read reply", repr(v)
        time.sleep(2)


def test_move_forward_stop(ser):
    # NCE USB 0xA2 <adr H/L> 04 0..7F (forward 128) and 06 00 (estop forward), reply 1 byte status
    for speed in [0x02, 0x00]:
        n = ser.write(bytearray([0xA2, LOCO_H, LOCO_L, 0x04, speed]))
        print "0xA2 fwd Write reply", n
        time.sleep(0.01)
        v = ser.read()  # 1 byte
        print "0xA2 fwd Read reply", repr(v)
        time.sleep(2)

    n = ser.write(bytearray([0xA2, LOCO_H, LOCO_L, 0x06, 0x00]))
    print "0xA2 estop Write reply", n
    time.sleep(0.01)
    v = ser.read()  # 1 byte
    print "0xA2 estop Read reply", repr(v)
    time.sleep(2)


def main():
    print COM, "open port"
    ser = serial.Serial(port=COM, baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, timeout=None)
    try:
        test_check_version(ser)
        test_send_dummies(ser)
        test_headlight_F4_on_off(ser)
        test_move_forward_stop(ser)
    finally:
        ser.close()
        print COM, "closed port"
    

if __name__ == "__main__":
    main()

    