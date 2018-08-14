#!/usr/bin/python
# vim: set ts=4 sw=4:
#
# Script that decodes & prints the NCE Cab Bus data to understand the protocol.
#
# For details, see: http://ralf.alfray.com/trains/nce_cab_bus.html
#
# --------------------------------------------------
# Usage:
# - Get an USB RS485 adapter that works on Linux.
# - Get a phone cord. Cut. Connect red/green to A/B. Connect other side to an NCE UTP.
#   (if you get bytes mostly in the 0xF0 range, reverse the A/B connection).
# - Run script. It listens and prints cab traffic in hex. You need the PDF below to understand it.
# --------------------------------------------------
#
# NCE Cab Bus Protocol:
# https://ncedcc.zendesk.com/hc/en-us/article_attachments/200182749/Cab_bus_protocol.pdf
#
# CS:
# - All bytes with 0x80 / but 7 on
# - Ping has bit 6 = 0, data bit 6 = 1 (0x40)
# - 0x80: ping on broadcast address / send data to all cabs (e.g. fast clock)
# - 0x81..BF: ping on address 1..63A
# - 0xC0+   : data byte
#
# Example fast clock sent by CS:
# 80 C1 (print right line 1, 8 chars) E0=sp F0=0 F4=4 FA=: F0=0 F0=0 D0=lf CD=cr
#
# Replies from bus devices:
# - ProCab is a type 'a' with 2 bytes:
#     First ping: 7E (refresh) 7F (no speed)
#     CS sends D2: 61 (type 'a')
#     Repeat: 7D [no key] 7F [no speed]
# - NCE USB: type 'c' 5 bytes: 42 19 03 04 50
#       4E 1B 00 00 55 : last byte = checksum (xor)
#       04 19 04 13 0A : 0419=engine 537 [op A2] 04=fwd-128 13=speed (7E max)
#       04 19 06 00 1B : 0419=engine 537 [op A2] 06=estop fwd
#   Engine address:
#       4F 00-7F = short address
#       XX 00-7F = long address: MSB<<7 + LSB (LSB bit 7 is always 0)
#   Throwing a turnout:
#       50 30 03 00 63 : 30=turnout 48 [op AD] 03=normal/close/on
#       50 30 04 00 64 : 30=turnout 48 [op AD] 04=reverse/throw/off

# 83 <-- 19 03 01[04] --> D9 C0 C0 C0 ??


import curses
import getopt
import serial
import sys

VERBOSE = False
DEV_TTY = "/dev/usb/ftdi"

CS_MASK   = 0x80    # Bit 7 is 1 for any command from CS
PING_MASK = 0x40    # Bit 6 is 0 for a command ping from CS
CAB_MASK  = 0x3F    # 0 is broadcast, cab addresses are 1..63

CAB_STR = (". " * (CAB_MASK + 1)).strip().split(" ")
CURRENT_CAB = 0
CURSES = True
LINES = {}
CAB_DATA = {}  # Map id ==> Cab object

class Cab(object):
    def __init__(self, cabId):
        self.cabId = cabId
        self.pingId = CS_MASK & cabId
        self.dvData = []    # data from device to CS
        self.csData = []    # data from CS to device
        self.newDvData = True
        self.newCsData = True
        self.nextDv = 0
        self.nextCs = 0

    def ping(self):
        self.newDvData = True
        self.newCsData = True

    def addDvByte(self, b):
        self.newDvData = self._addByte(self.dvData, self.newDvData, b)

    def addCsByte(self, b):
        self.newCsData = self._addByte(self.csData, self.newCsData, b)

    def _addByte(self, data, newData, b):
        n = len(data)
        if n == 0 or newData:
            buf = [ b ]
            data.append(buf)
        else:
            data[n-1].append(b)
        return False

    def printStart(self, scrn, maxChars):
        self._printBuf(scrn, 0, maxChars / 5, self.dvData)

    def printLast(self, scrn, maxChars, data):
        n = maxChars / 5
        start = len(data) - 1
        if start < 0:
            return
        # remove all the last 7D 7F pairs from a 2-byte type a cab
        while start > 1 and data[start] == [0x7D, 0x7F] and data[start] == data[start-1]:
            start -= 1;
        ni = len(data[start])
        while start > 0:
            ni2 = len(data[start - 1])
            if ni + ni2 > n:
                break
            ni += ni2
            start -= 1
        self._printBuf(scrn, start, n, data)

    def _printBuf(self, scrn, start, n, data):
        ni = 0
        dn = len(data)
        while start < dn:
            buf = data[start]
            start += 1
            bn = len(buf) - 1
            bi = 0
            while bi <= bn:
                b = buf[bi]
                # _debug(scrn, (n, ni, dn, start, bn, bi, self.data))
                self._printByte(scrn, b)
                scrn.addstr(bi == bn and '|' or ' ')
                bi += 1
                ni += 1
                if ni >= n:
                    return

    def _printByte(self, scrn, b):
        scrn.addstr("%02X " % b)
        b = b & 0x7F
        if b & 0x20 != 0: b = b ^ 0x40
        scrn.addstr("%c" % ((b <= 32 or b > 0x7e) and ' ' or chr(b)))

    def printNoCurses(self):
        self.nextCs = self._printNoCurses("CS to %02x: " % self.cabId, self.nextCs, self.csData)

        # if the last printed DV was a 7D/7F, skip the following ones
        if self.nextDv > 0 and self.dvData[self.nextDv-1] == [0x7D, 0x7F]:
            while self.nextDv < len(self.dvData) and self.dvData[self.nextDv] == [0x7D, 0x7F]:
                self.nextDv += 1
        self.nextDv = self._printNoCurses("%02x to CS: " % self.cabId, self.nextDv, self.dvData)

    def _printNoCurses(self, header, next, data):
        n = len(data)
        head = True
        while n > next:
            buf = data[next]
            next += 1
            if head and len(buf) > 0:
                print header,
                head = False
            for b in buf:
                b2 = b & 0x7F
                if b2 & 0x20 != 0: b2 = b2 ^ 0x40
                print "%02X" % b, "%c" % ((b2 <= 32 or b2 > 0x7e) and ' ' or chr(b2)),
        if not head:
            print
        return next


def _debug(scrn, data):
    scrn.addstr("%s=%s" % (type(data), str(data)))

def _fatal(data):
    curses.nocbreak()
    curses.echo()
    curses.endwin()
    print type(data), str(data)

def open_serial():
    return serial.Serial(DEV_TTY,
                         baudrate=9600,
                         bytesize=serial.EIGHTBITS,
                         parity=serial.PARITY_NONE,
                         stopbits=serial.STOPBITS_TWO)

def read_byte(s):
    b = s.read(1)
    if isinstance(b, str):
        b = ord(b[0])
    return b

def print_current(scrn):
    scrn.addstr(0, 0, "Current: %02d" % CURRENT_CAB)
    scrn.move(1, 2)
    for c in CAB_STR:
        scrn.addstr(c)
    scrn.move(1, 0)
    scrn.refresh()

def print_cab(scrn, cab):
    s = scrn.getmaxyx()
    n = max(0, s[1] - 4 - 8)
    line = 3 + LINES[cab.cabId] * 4
    scrn.addstr(line    , 0, "%02d " % cab.cabId)
    scrn.addstr(line    , 3,   "Start: ") ; cab.printStart(scrn, n)
    scrn.addstr(line + 1, 3,   " Live: ") ; cab.printLast(scrn, n, cab.dvData)
    scrn.addstr(line + 2, 1, "from CS: ") ; cab.printLast(scrn, n, cab.csData)
    scrn.refresh()

def process_serial(s, scrn):
    global CURRENT_CAB
    global CAB_DATA
    global LINES

    bytescr = None
    datascr = None
    if CURSES:
        h, w = scrn.getmaxyx()
        bytes_h = 8
        bytescr = curses.newwin(bytes_h, w, h - bytes_h, 0)
        bytescr.idlok(1)
        bytescr.scrollok(1)
        bytescr.addstr(0, 0, "Raw ")
        bytescr.refresh()
        sepscr  = curses.newwin(1, w, h - bytes_h - 1, 0)
        datascr = curses.newwin(h - bytes_h - 1, w, 0, 0)
        sepscr.hline(0, 0, '_', w)
        sepscr.refresh()

    while True:
        b = read_byte(s)
        if b & CS_MASK != 0:        # byte from CS
            if b & PING_MASK == 0:  # ping from CS
                if CURSES: bytescr.addstr("%02X " % b)
                CAB_STR[CURRENT_CAB] = CURRENT_CAB in CAB_DATA and "!" or "."
                newCab = b & CAB_MASK
                if not CURSES and (newCab != CURRENT_CAB) and (CURRENT_CAB in CAB_DATA):
                    CAB_DATA[CURRENT_CAB].printNoCurses()
                CURRENT_CAB = newCab 
                CAB_STR[CURRENT_CAB] = "*"
                if CURRENT_CAB in CAB_DATA:
                  CAB_DATA[CURRENT_CAB].ping()
                if CURSES: print_current(datascr)
            else:
                if CURSES: bytescr.addstr("%02X " % b, curses.A_UNDERLINE)
                if CURRENT_CAB in CAB_DATA:
                    CAB_DATA[CURRENT_CAB].addCsByte(b)
        elif CURRENT_CAB != 0:      # byte from device to CS
            if not CURRENT_CAB in CAB_DATA:
                CAB_DATA[CURRENT_CAB] = Cab(CURRENT_CAB)
                if not CURRENT_CAB in LINES:
                    LINES[CURRENT_CAB] = len(LINES)
            CAB_DATA[CURRENT_CAB].addDvByte(b)
            if CURSES: print_cab(datascr, CAB_DATA[CURRENT_CAB])
        if CURSES:
            if b & (CS_MASK | PING_MASK) == 0:
                bytescr.addstr("%02X " % b, curses.A_REVERSE)
            bytescr.refresh()


def usage():
    print """Decodes NCE Cab Bus using an RS485-to-USB adapter.

Syntax :
   [-v | --verbose] [-d | --device /dev/ttyUSBn] [-h | --help] [-p|--print]

Mode:
    - Default is a curses display.
    - Print dump bytes in simple text mode.

Default device is %s.
""" % (DEV_TTY)
    sys.exit(2)

def main(argv):
    show_help = False
    try:
        opts, args = getopt.getopt(argv, "hvd:p", [ "help", "verbose", "device=", "print" ])

        if len(args) > 0:
            print "Unknown parameters:", args
            print
            show_help = True

        for opt, arg in opts:
            if opt in [ "-v", "--verbose" ]:
                global VERBOSE
                VERBOSE = True
            elif opt in [ "-d", "--device" ]:
                global DEV_TTY
                DEV_TTY = arg
            elif opt in [ "-h", "--help" ]:
                show_help = True
            elif opt in [ "-p", "--print" ]:
                global CURSES
                CURSES = False
    except getopt.GetoptError:
        show_help = True

    if show_help:
        usage()

    s = None
    stdscr = None
    try:
        s = open_serial()
        if CURSES: stdscr = curses.initscr()
        process_serial(s, stdscr)
    finally:
        if CURSES:
            curses.nocbreak()
            if stdscr != None:
                stdscr.keypad(0)
            curses.echo()
            curses.endwin()
        if s != None:
            s.close()

if __name__ == "__main__":
    main(sys.argv[1:])
