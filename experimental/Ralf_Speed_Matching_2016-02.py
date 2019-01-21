# Script to speedmatch all steps of a locomotive and decoder after installation.
# Based on one of many JRMI jython examples and heavily modified.
#
# (c) Ralf 2016-02. License: "Do as you want, at your risk and perils".
#
# Mode of operation:
# - This is expected to run on a loop with 3 sensors.
# - There are 3 sensors on the loop:
#   - Start / Stop sensors delineate the section of the track used to measure speed.
#   - Reset sensor is used to know when to start adjusting the speed before reaching the timed segment.
# - Warmup: Start by doing 5 loops to warm up the motor.
# - Speed / Interval / Rety: Speed mode is either 28 or 128; Interval is the number of steps to skip,
#     and Retry is the number of times to retry timing the segment (and average results) for a measure.
#     For example 28/14/3 would check speed steps 1, 14 and 28 and measure 3 times for each step.
# - Measure Mode: For each speed step, measure the time to go through the timed segment. 
#     Sometimes sensors can be flaky so if a measured time is twice or more than the previous run it is ignored.
#     The output is given in a text field, which can then be plotted in a Google Spreadsheet.
# - "Set Linear" Mode: Reprograms the decoder's speed table using the given values for the requested speed steps.
#     This uses a basic dichotomy: take a measure, adjust the speed table CV, and iterate till it converges
#     towards the desired time.
#
# A typical usage of this is to match engines running in MU:
# - Get one engine as a reference and measure its speed at various steps.
# - Plot the measured times to make sure they are consistent (sensors are not that precise sometimes).
# - Now change the engine on the loop to one to be re-programmed, keep the same values in the UI and
#   use the 'set linear' mode so that it tries to adjust the speed table to match the previous engine.
#

import java
import javax.swing
from javax.swing import *

class RalfTest(jmri.jmrit.automat.AbstractAutomaton) :
    
    # define how long to wait between settings (seconds)
    delay = 2
    
    # init() is called exactly once at the beginning to do
    # any necessary configuration.
    def init(self):
        self._continue = True
        self.afterStopDelay = 0
        self.status.text = "Getting throttle"
        number = int(self.fieldAddress.text)
        self.loco_number = number
        isLong = (number > 100)
        self.throttle = self.getThrottle(number, isLong)
        if self.throttle == None:
            print "Couldn't assign throttle!"
        self.throttle.setSpeedStepMode(self.throttle.SpeedStepMode128)
        print "Throttle", self.throttle.getSpeedStepMode(), " inc", self.throttle.getSpeedIncrement()
        self.startSensor = sensors.provideSensor(self.fieldSensorStart.text)
        self.stopSensor  = sensors.provideSensor(self.fieldSensorStop.text)
        self.resetSensor  = sensors.provideSensor(self.fieldSensorReset.text)

        self._cv8 = self._readCV(8)
        self._isLokSound = self._cv8 == 151
        print "LokSound mode:", self._isLokSound
        
    def _light(self, state):
        self.throttle.setF0(state)

    def _speed(self, v28):
        self.throttle.setIsForward(v28 >= 0)
        absv28 = v28
        if absv28 < 0:
            absv28 = -absv28
        self.throttle.setSpeedSetting(absv28 / 28.)
        self.fieldSpeed.text = str(v28)
        print "Speed set to", self.throttle.getSpeedSetting()

    def _toot(self):
        self.throttle.setF8(False)  # sound
        self.waitMsec(1000)
        for i in range(0, 2):
            self.throttle.setF2(True)
            self.waitMsec(100)
            self.throttle.setF2(False)
            self.waitMsec(1000)
        self.throttle.setF8(True)  # mute

    def _waitStart(self):
        self.waitSensorActive(self.startSensor)
        self.status.text = "Start point reached"

    def _waitStop(self):
        self.waitSensorActive(self.stopSensor)
        self.status.text = "Stop point reached"

    def _waitReset(self):
        self.waitSensorActive(self.resetSensor)
        self.status.text = "Reset point reached"

    def _nowTS(self):
        return java.lang.System.currentTimeMillis()

    def _readCV(self, cv):
        sp = self.throttle.getSpeedSetting()
        if sp > 0:
            self.throttle.setSpeedStepMode(0)
            self.waiMsec(int(1000 + 2000. * sp))
        value = self.readServiceModeCV(cv)
        print "READ CV %s = %s" % (cv, value)
        return value

    def _writeCV(self, cv, value):
        sp = self.throttle.getSpeedSetting()
        if sp > 0:
            self.throttle.setSpeedStepMode(0)
            self.waiMsec(int(1000 + 2000. * sp))
        if self.writeServiceModeCV(cv, value):
            print "WRITE CV %s = %s" % (cv, value)
            self.waitMsec(500)
        else:
            print "FAILED to write CV %s = %s" % (cv, value)

    # handle() will only execute once here, to run a single test
    def handle(self):
        if self._mode == "measure":
            return self.handleMeasure()
        elif self._mode == "linear":
            return self.handleLinear()
    
    def handleMeasure(self):
        if self.fieldWarmup.selected:
            self._warmup()

        speed = 28
        self._light(True)
        self.throttle.setF8(True)  # mute
        self.textArea.text = ""
        last_elapsed = 0
        interval = max(1, int(self.fieldInterval.text))
        retry = max(1, int(self.fieldRetry.text))
        self.fieldMeasures.text = ""
        try:
            # Ignore the first loop
            self._speed(speed)
            self._waitReset()
            print "Ignoring first loop"

            while speed > 0 and self._continue:
                measured = 0
                r = 0
                while r < retry:
                    r += 1
                    self.status.text = "Measure speed %d, #%d of %d" % (speed, r, retry)
                    print self.status.text
                    self._speed(speed)

                    # Wait for start signal
                    self._waitStart()
                    now = self._nowTS()

                    # Wait for end signal
                    self._waitStop()
                    elapsed = self._nowTS() - now

                    # If too big by a somewhat variable factor, ignore it
                    factor = 2.5
                    if r == 0:
                        if speed <= 2:
                            factor = 4
                        else:
                            factor = 3
                    if last_elapsed > 0 and elapsed > last_elapsed * factor:
                        print speed, "ignored", elapsed, ">", factor, "*", last_elapsed
                        r -= 1
                        continue
                    elif last_elapsed > 0:
                        print speed, "loop time:", elapsed, float(elapsed) / float(last_elapsed), "%"
                    last_elapsed = elapsed
                    measured += elapsed

                    # Continue max speed till next loop
                    self._speed(28)
                    self._waitReset()

                elapsed = int(measured / retry)
                self.fieldMeasures.text += "%d: %d, " % (speed, elapsed)
                result = "%d, %d\n" % (speed, elapsed)
                self.textArea.text += result
                print result

                # Prepare for next loop; always go speed 1 last
                last_speed = speed
                speed -= interval
                print "Next speed:", speed
                if speed <= 0 and last_speed > 1:
                    speed = 1
            self._toot()
        finally:
            self._speed(0)
            self._light(False)
            return False

    def handleLinear(self):
        if self.fieldWarmup.selected:
            self._warmup()

        retry = max(1, int(self.fieldRetry.text))
        measures = self._parseMeasures(self.fieldMeasures.text)
        print "Parsed measure targets:", measures
        # We always expect at least targets for 1 and 28
        time28 = measures[28]
        time1 = measures[1]

        if not self._continue:
            return False

        self.textArea.text = ""
        vmin = 0
        vmax = 255
        cv67 = 67
        vdelta = vmax - vmin
        tdelta = time28 - time1

        # Match speeds 28 then 1 and then everything in between
        speeds = [28, 1]
        speeds += range(27, 1, -1)
        
        #if self._isLokSound:
        #    speeds = []
        #if not fast:
        #    speeds += range(27, 1, -1)
        #else:
        #    speeds += range(2, 28)

        speedMap = {}

        for speed in speeds:
            if not self._continue:
                break
            cv = cv67 + speed - 1
            if self._isLokSound:
                if speed == 1:
                    cv = 2  # CV Vstart
                elif speed == 28:
                    cv = 5  # CV Vhigh

            # TODO if not target in measures map, interpolate from closest values
            if speed in measures:
                target = measures[speed]
            else:
                target = int(time1 + (float(speed - 1) / 27. * tdelta))
            print "Match speed", speed, "target", target

            self._adjustSpeed(speed, cv, target, speedMap, retry, time1)

        if self._continue:
            self._toot()
        return False
    
    def _parseMeasures(self, text):
        """
        Expected string format:
        - "n1,...,n28" with consecutive numbers for speeds 1..28.
        - "i1:n1,...,i28:n28" with indexed "speed : time".
        - mix of the above.
        - Always coma separated.
        """
        m = {}
        index = 1
        for entry in text.split(","):
            entry = entry.strip()
            fields = entry.split(":")
            if len(fields) == 2:
                index = int(fields[0].strip())
            m[index] = int(fields[-1].strip())
        return m
        

    def _adjustSpeed(self, speed, cv, target, speedMap, retry, max_tm):
        last_v = 0
        v = 128
        d = v >> 1
        ddir = 0
        vmin = 0
        if self._isLokSound:
            vmin = 1

        same_retry = 0
        while same_retry < retry and self._continue:
            self._writeCV(cv, v)

            if speedMap is None:
                tm = self._measureSpeed(speed)
            else:
                prev = speedMap.get(v)
                if prev is None:
                    while True:
                        tm = self._measureSpeed(speed)
                        speedMap[v] = (tm, 1)
                        if tm > 6 * max_tm:
                            print "Map CV", v, "| Ignore first", tm, ">", 6 * max_tm
                        else:
                            break
                    print "Map CV", v, "| Init to", repr(speedMap[v])
                elif prev[1] >= retry:
                    tm = prev[0]
                    print "Map CV", v, "| Reuse", repr(prev)
                else:
                    last = prev[0]
                    loop = 0
                    while loop < retry:
                        loop += 1
                        tm = self._measureSpeed(speed)
                        if tm > 2 * last:
                            # Maybe invalid. Retry once more.
                            print "Map CV", v, "| Ignore retry", tm, ">", 2 * last
                            continue
                        break
                    tm = int((last + tm) / 2)
                    speedMap[v] = (tm, prev[1] + 1)
                    print "Map CV", v, "| Update to", repr(speedMap[v])
            
            print "Adjust speed", speed, "cv", cv, "value", v, "delta", d, "time", tm, "target", target
           
            last_v = v
            last_dir = ddir
            if tm > target:
                v = min(255, v + d)
                ddir = 1
            elif tm < target:
                v = max(vmin, v - d)
                ddir = -1
            else:
                break
            if last_dir != 0 and last_dir != ddir:
                d = d >> 1
            if v == last_v:
                same_retry += 1
            else:
                same_retry = 0
            print "Next v", v, ", retry", same_retry, "| d", d, ", ddir", ddir

        result = "%d, %d\n" % (speed, tm)
        self.textArea.text += result
        print result
       
    def _warmup(self, laps=4):
        try:
            self._light(True)
            self.throttle.setF8(True)  # mute
            self._speed(28)
            delay = 0
            lap = 0
            while lap < laps:
                lap += 1
                txt = "Warmup Loop %d" % lap
                print txt
                self.status.text = txt
                if not self._continue:
                    break
                # Wait for start signal
                self._waitStart()
                now = self._nowTS()

                # Wait for end signal
                self._waitStop()
                d = self._nowTS() - now
                print "Warmup measure:", d
                if delay == 0:
                    delay = d
                else:
                    delay = min(delay, d)
                print "After stop delay", delay
            self.afterStopDelay = delay
        finally:
            self._speed(0)
            self._light(False)


    def _measureSpeed(self, speed):
        if not self._continue:
            return 0
        elapsed = 0
        try:
            self._light(True)
            self.throttle.setF8(True)  # mute

            self._speed(speed)
            self._waitStart()
            now = self._nowTS()

            # Wait for end signal
            self._waitStop()
            elapsed = self._nowTS() - now
            print speed, "=>", elapsed

            self._speed(28)
            # self.waitMsec(self.afterStopDelay)
            self._waitReset()

        finally:
            self._speed(0)
            self._light(False)
            return elapsed

    # define what button does when clicked and attach that routine to the button
    def _onStartMeasure(self, event):
        self._continue = True
        self._mode = "measure"
        self.start()

    def _onStartLinear(self, event):
        self._continue = True
        self._mode = "linear"
        self.start()

    def _onStop(self, event):
        self._continue = False
        self.stop() # TODO use a different flag
        
    # routine to show the panel, starting the whole process     
    def setup(self):
        # create a frame to hold the button, set up for nice layout
        f = JFrame("Ralf Speed Test")       # argument is the frames title
        f.contentPane.setLayout(BoxLayout(f.contentPane, BoxLayout.Y_AXIS))

        # Fields for setup/configuration
        panelAddress = JPanel()
        panelAddress.add(JLabel("Address/Sensors"))

        self.fieldAddress = JTextField("5000", 5)
        self.fieldSensorStart = JTextField("TS37", 4)
        self.fieldSensorStop = JTextField("TS32", 4)
        self.fieldSensorReset = JTextField("TS36", 4)

        panelAddress.add(self.fieldAddress)
        panelAddress.add(self.fieldSensorStart)
        panelAddress.add(self.fieldSensorStop)
        panelAddress.add(self.fieldSensorReset)
        f.contentPane.add(panelAddress)

        panelServer = JPanel()
        panelServer.add(JLabel("Sensors Server"))

        self.fieldServer = JTextField("localhost:8090", 15)

        panelServer.add(self.fieldServer)
        f.contentPane.add(panelServer)

        # Fields for flags
        panelFlags = JPanel()
        panelFlags.add(JLabel("Flags"))
        
        self.fieldWarmup = JCheckBox("Warmup")
        self.fieldWarmup.selected = True
        
        panelFlags.add(self.fieldWarmup)
        f.contentPane.add(panelFlags)
        
        # Fields for current speed and results
        panelSpeed = JPanel()
        panelSpeed.add(JLabel("Speed/Interval/Retry"))

        self.fieldSpeed = JTextField(3)
        self.fieldInterval  = JTextField("1", 3)
        self.fieldRetry = JTextField("1", 3)

        panelSpeed.add(self.fieldSpeed)
        panelSpeed.add(self.fieldInterval)
        panelSpeed.add(self.fieldRetry)
        f.contentPane.add(panelSpeed)

        # Fields for measured times 1/28
        panelMeasures = JPanel()
        panelMeasures.add(JLabel("Measures 1->28"))

        self.fieldMeasures  = JTextArea(
                editable=True, wrapStyleWord=True, lineWrap=True, size=(200,300))
        self.fieldMeasures.text = "1:52300, 28:4200"

        panelMeasures.add(self.fieldMeasures)
        f.contentPane.add(panelMeasures)
    
        # Buttons
        panelButtons = JPanel()

        self.measureButton = JButton("Measure")
        self.measureButton.actionPerformed = self._onStartMeasure

        self.linearButton = JButton("Set Linear")
        self.linearButton.actionPerformed = self._onStartLinear

        self.stopButton = JButton("Stop")
        self.stopButton.actionPerformed = self._onStop

        panelButtons.add(self.measureButton)
        panelButtons.add(self.linearButton)
        panelButtons.add(self.stopButton)
        f.contentPane.add(panelButtons)

        # Result text area
        panelArea = JPanel()
        self.textArea = JTextArea(
                editable=True, wrapStyleWord=True, lineWrap=True, size=(300,300))
        self.textArea.text = "Results\n"
        panelArea.add(self.textArea)
        f.contentPane.add(panelArea)
        
        # Put contents in frame and display
        self.status = JLabel("Enter address & click start")
        panelStatus = JPanel()
        panelStatus.add(self.status)
        f.contentPane.add(panelStatus)
        f.pack()
        f.show()

# create one of these
a = RalfTest()

# set the name, as a example of configuring it
a.setName("Ralf test script")

# set the time between settings
#a.delay = 3

# and show the initial panel
a.setup()
