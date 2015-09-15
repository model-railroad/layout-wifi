# Translate & LayoutWifi #

This repository contains 3 utilities that I use to manage my H0 model-train DCC layout at home:

* **LayoutWifi** runs on a DigiX (an Arduino clone) to control physical turnouts and sensors.
* **Translate** is a service that interfaces LayoutWifi to both RocRail or JMRI. 
* **Translate** also performs train block detection by analyzing nigh-vision IP cameras.

The 3 pieces are designed to work together.


### Example ###

Let's detail a concrete example. My home layout is DCC. I use an NCE booster and an NCE USB interface.
This allows software like JMRI or RocRail to provide basic throttle management.

To really automate a train layout, we need 2 more important pieces:

* A way to control turnouts so that the automating software can change trains' routes.
* A way to report block occupancy to the automating software so that it can know where trains are.

In my case, the setup works as follows:

* A linux server runs RocRail or JMRI. This is the automating software.
* The DigiX arduino controls relays that switch the turnouts -- this is the LayoutWifi software.
* The linux server also runs Translate, which offers the following services:
    * On one side, it connects to the LayoutWifi software on the DigiX to send turnouts commands and receive turnout feedback information.
    * On the other side, it connect to one or more IP cameras and analyze the images to provide block occupancy sensors.
    * Finally it interfaces with RocRail via an SRCP protocol or with JMRI via an NCE binary protocol.

~~~~
  DigiX                         +---[SRCP]---> RocRail ---+
LayoutWifi <----> Translate <---|                           |---> NCE USB ---> NCE Booster / DCC
    |                 ^         +---[NCE]----> JMRI ------+
    v                 |
 turnouts          ip-cams
~~~~


Here's a [video of what it looks like in action](https://youtu.be/ycCJjT7WUgk):

[![LayoutWifi and Translate in action](http://img.youtube.com/vi/ycCJjT7WUgk/0.jpg)](https://youtu.be/ycCJjT7WUgk)


### Rationale ###

Why this thing?

First to be clear: you need none of this thing.
You can just buy DCC-ready turnouts these days. 
You can control Tortoise slow-motion machines or twin-coil turnouts with NCE Switch-It or Snap-It DCC interfaces.
You can also do block control detection using a variety of sensors (magnets) with AIU modules or block detection using BD20 modules. And that's just NCE. DigiTrax and LocoNet provide a variety of DCC-friendly alternatives.

However this is a *DIY makers* project. 
I didn't want to use existing solutions and instead I like exploring new software or different ways.

The only thing I've never seen before is block detection using cheap night-vision cameras. 
There's probably a reason for this.

Speaking of rationale, what's with the SRCP vs NCE binary protocols?

I needed a way to interface my server with both RocRail and JMRI.
After looking at both, I choose to implement an existing protocol to talk to RocRail and JMRI instead of writing an extension to these 2 softwares. They both support a large variety of custom closed protocols. After looking at a variety, I settled down on SRCP -- it's a fairly simple protocol and it's well documented. The RocRail implementation is good and solid. There's also a microSRCP protocol that seemed adequate for use on the arduino.

However it turned out that JMRI doesn't support SRCP correctly. JMRI 3.9's SRCP implementation doesn't follow the spec at all. Instead I reverted to the NCE binary protocol which is used by their boosters in serial mode or by the NCE USB interface that I have, since I know that one pretty well and it's properly implemented in JMRI's code.

As for microSRCP, I unfortunately couldn't use it since the DigiX wifi interface works as a serial port, not using a socket concept, so there can't be 2 connections at the same time for the INFO vs COMMAND channels. Instead I made up a mini character protocol and the Translate server does the interface.


## LayoutWifi ##

LayoutWifi runs on [http://digistump.com/products/50](DigiX), an arduino clone that has wifi support, sdcard support and a [tons of i/o pins](http://digistump.com/wiki/digix/tutorials/pinout).


To run the unit tests:
~~~~
$ cd digix/LayoutWifi
$ ./_test.sh
~~~~

Note that _test.sh is currently designed for Cygwin's GCC.


To flash the software on a DigiX:

* Follow the wiki instructions to [setup the Arduino IDE with the DigiX board](http://digistump.com/wiki/digix/tutorials/software).
* Load [LayoutWifi.ino](https://bitbucket.org/ralfoide/layout/src/HEAD/digix/LayoutWifi/LayoutWifi.ino) in the IDE.
* Build and deploy to your DigiX.


Features of the software, which can be changed via parameters in
[LayoutWifi.ino](https://bitbucket.org/ralfoide/layout/src/HEAD/digix/LayoutWifi/LayoutWifi.ino):

* By default, up to 8 turnouts can be controlled.
* Turnouts are controlled via pins 90-105. 
    * Even pins put turnouts in normal state, odd pins in reverse state.
* The software was designed to emulate NCE AIU cards to report sensor information.
    * Each card can have up to 14 sensor inputs.
* Up to 4 AIUs are supported.
    * AIUs 1 and 2 represent turnout states. Both RocRail and JMRI can be configured to query turnout feedback information via these sensors.
    * AIUs 3 and 4 represent real sensors, polling from pins 22-49.
* Wifi port is 8080.

The DigiX wifi seems to have its own set of idiosynchrasies which severly influenced (a.k.a. "limited") the design.
There is no proper socket support with anything resembling a client connection and disconnection, and
experimental trials show it won't accept connections from more than one client at a time.
Since the functionality is so limited, it should be relatively trivial to port this to any other arduino
with an ethernet or wifi shield if desired.


The protocol to communicate with the Translate server is quite simple:

* Each command is one text line (ASCII), starting with `@` and terminated by `\n`.
* Commands sent by Translate and received by LayoutWifi on the DigiX:
    * `@I\n` requests a status. Reply: `@T08S04\n` indicating the board supports 8 turnouts and 4 AIU cards.
    * `@T01N\n` or `@T01R\n` indicates to set turnout 1 into either Normal or Reverse directions.
        * Turnout numbers must be greater than zero and equal to the number of turnouts reported by `@I\n`.
        * The reply is exactly the command received, for confirmation that is was properly understood.
        * The corresponding relay is only set if the turnout is in the opposite direction.
* Commands sent by LayoutWifi from the DigiX to the Translate server:
    * `@S01ABCD\n` represents an AIU 14-bit state.
        * The first 2 digits (`01` in the example) indicate the AIU card number. The value is 1-based.
        * The second 4 hexa diits (`ABCD` in the example) indicate the sensors bit state in big endian.
          Since AIU cards have 14 bits each, the max value is thus 0x3FFFF.
* There is no command to retrieve sensor state. Instead the arduino polls sensors every 10 milliseconds
  are reports sensor status if anything as changed.
* Turnout states are also reported as sensor states on AIUs 1 and 2.
    * The current configuration supports 8 turnouts, so obvioulsy only AIU #1 on bits 1-8 is currently used.
    * Sensors bits are set to 0 for a turnout in normal direction and 1 for a turnout in reverse direction.


## Translate ##

Translate a simple server written in Go. It runs a variety of services on the following default ports:

* An SRCP server on port 4303, to communicate with RocRail.
* An NCE binary server on port 8080, to communicate with JMRI.
* A web server on port 8089, to get realtime status information.
* A LayoutWifi client, which connects to the DigiX on port 8080.
* An optional LayoutWifi simulator, which emulates a DigiX if you don't have one.
* One or more IP camera clients that connect to the IP cameras configured in the config file.


### Building and deploying ###

First install Go 1.4 or 1.5 following the [golang.org instructions](https://golang.org/doc/install).
You can build from source or just use any of the OS-specific packages.

Make sure `$GOROOT` is properly defined in your environment.
If that's your first time using Go, test your installation by following the hello world steps in the
[install](https://golang.org/doc/install#testing) document.

Once you have a working Go environment you can build & test the software:

~~~~
$ cd translate
$ ./_test.sh
$ ./_build.sh
$ bin/translate --simulate
~~~~

The shell scripts set `$GOPATH` to `src/translate` for you. 
The only env var needed is a properly set `$GOROOT`.
The output is `bin/translate`.

Running with the `--simulate` flag creates a local LayoutWifi/DigiX server and connects to it.
This allows you to quickly test without having a DigiX around.
If all you want is the camera sensor portion of the software, this also allows you to run
Translate without needing an arduino.


### Command-line flags and .translaterc ###

To get the available command-line flags:

~~~~
$ bin/translate --help
~~~~

All the command-line flags can also be placed in a `~/.translaterc` file
using a simple per-line `key=value` syntax where key is the name of the
command-flag without the initial dashes.


### Translate Camera Sensors ###

Let's start with the interesting part.

In this first version, cameras are managed by creating a `~/.translaterc` file.

Here's an example of what one would look like.i
Take the following snippet, save it in ~/.translaterc and _then_ start `bin/translate --simulate`:

~~~~
lw-client-port=mydigix.local:8080

cam-server = :8088
cam-urls = http://user:pass@dlink1.local/mjpeg.cgi,http://user:pass@dlink2.local/mjpeg.cgi

cam-sensors = 1,32:61,308,82,287 1,33:519,313,537,337 1,34:38,101,62,91 1,35:566,282,576,312 1,36:34,178,59,159 1,37:622,304,631,329 1,38:589,437,589,466 0,29:385,454,360,451 0,39:177,423,150,420 0,40:52,403,29,398 0,41:579,20,599,23 0,42:120,458,100,455 0,43:81,33,101,27 0,44:192,17,211,20 0,45:545,62,565,64 0,48:379,415,402,418 1,49:499,372,499,401 0,50:378,393,400,388 1,51:445,412,445,441 0,52:128,51,147,54 0,53:539,116,562,116

cam-offsets = 0:0,0 1:0,0
~~~~

Note that you can provide these parameters as command-line arguments. It's just easier to put them in the rc file.

Let's go over these parameters:

* `lw-client-port` is the host:port to connect to the DigiX. 
  Simply ommit the line if you're not having a real DigiX and use the `--simulate` option instead.
* `cam-server` is the port where the Translate server web page can be accessed. The default is 8088, so once
  Translate run you can view the status page at [http://localhost:8088](http://localhost:8088).
* `cam-urls` indicates the access URLs for your IP cameras.
    * This is a **comma-separated** list.
    * The order: camera 0 is the first one in the list, camera 1 the next one, etc.
    * See below for more information on which cameras are supported.
* `cam-sensors` is a **space-separated** list of sensors.i
    * The syntax for each sensor is "camera number - sensor number : x1,y1,x2,y2".
    * Camera number is a 0-based value matching the cameras from `cam-urls`.
    * Sensor number is the sensor id that must be reported back to RocRail or JMRI.
      See below for sensor numbers.
    * Coordinates indicate the start-end points of a segment to analyze in the image.
* `cam-offsets` is a space-separated lists of pixel offsets to add to the X/Y values in `cam-sensors`.
  This is convenient to adjust the points analyzed on the images in case the visual alignements gets off.


**Supported IP Cameras and URLs:**

First of all, IP cameras are accessed over an HTTP MJPEG stream.
There is currently no support for JPEG snapshot mode and there's no H264 support.
There is no support for USB-connected webcams.

I use night-vision IP cameras. This allows me to operate the layout in very low light or even in the dark.
Modern IP cams have an automatic IR filter feature.
If possible you want to deactivate that as their video is often different in IR mode vs day mode.

If you can't disable the automatic IR filter and force night mode in the camera settings, simply tape
over the light detector.

I've personnally used 3 kind of cameras:

* Foscam FI8910W. These are Pan & Tilt with a strong infrared vision and a large field view.
  In theory, the pan & tilt feature seems attractive but even though you can set memory points so
  that the camera pan & tilts to the same place, it's not that precise and you'll have alignment issues.
  The larger field of view unfortunately makes the corners unusuable due to barrel distortion.
  I don't recommend it for this application.
* Make sure to avoid the recent H264 "Plug'n'Play" Foscam cameras. Their software is retrograde and
  actually does not work at all anymore on Linux unless you use their custom Windows or Mac software.
* Edimax IC-3116W and D-Link DCS-932L are pretty good choices. They have descent IR illumination.
  They do 640x480 in MJPEG.
  The D-Link is the cheaper one but quality is really just the same as with the other ones.
  In the D-Link you can force night-mode via the settings.

For Foscam and Edimax, the default multipart Go parser works well to split the MJPEG stream.
This doesn't work with the D-Link and I coded a
[simplified MJPEG stream parser](https://bitbucket.org/ralfoide/layout/src/651f96d615e9622f35832c4990a09a84832fe7aa/translate/src/translate/cam_sensor.go?at=master&fileviewer=file-view-default#cam_sensor.go-390) which is triggered by detecting the word `dlink` in the camera host name.
Adding a configuration setting is left as an exercise for the reader.

To choose which camera to use, think of your lower denominator use case.
If you're only going to use the layout in daylight or with plenty of illumination, you could use even
cheaper cameras without night-vision support. However if you want to operate with dimmed lights,
you might be better off using IR night-vision enabled IP cameras.

No matter what you use, the camera analysis is done on a black and white image stream.


**Sensor numbers:**

* Since Translate and LayoutWifi emulate NCE AIU cards which have 14-bits each, numbers must match.
* Sensors 1-14 are reserved in LayoutWifi for turnouts.
* If you exclusively use RocRail, you can use any sensor numbers from 15 and up.
  That's because the communication with RocRail is done via SRCP which is a hardware-agnostic protocol.
* If you want to use sensors with JMRI, since I use the NCE binary protocol you need to comply with
  how an NCE AIU card would be addressed in JMRI. 

As explained in the [JMRI NCE](http://jmri.sourceforge.net/help/en/html/hardware/nce/NCE.shtml) page, this is:
~~~~
Sensor id = (AIU address - 1) * 16 + (AIU pin number) -1
~~~~
Concretely:

* Sensors  0-13 = AIU #1
* Sensors 16-29 = AIU #2
* Sensors 32-45 = AIU #3
* Sensors 48-61 = AIU #4

And in this case with JMRI you can't access sensor numbers 14, 15, 30, 31, 46, 47, etc.
That's because NCE AIU cards have 14-bits and thus the 2 higher bits are not addressable.
However if you use RocRail, you _do not have to care_ and you can use them.

Note that the sensor numbers do not have to be consecutive.
Internally for Translate, they don't matter, they are just labels.
It's only RocRail or JMRI which interpret them in a specific way when you define your sensor tables.


**Sensor Coordinates:**

This is where it's visible this is a version 1.
In an ideal application, I'd want to configure my sensors via a web page interactively.
That's not the case here.

The current sensor detection works by taking 2 points on the image and drawing a line between both:

* 12 points are sampled over that line. 
* A 3/4th moving window average is computed.
* The analysis wants to see 3 zones: 1 dark zone, 1 white zone, 1 dark zone.
* There must be at least 3 points with a dark or gray color value below the average,
  followed by at least 3 points above average and finally at least 3 points below average.

In this first experimental revision, I physically place little pieces of white tape on the track.
The pieces cover about 3 ties in length (on a H0 track).
I tried various materials and settle down on a white tape that I found appeared very white and
reflexive on my infrared night-vision cameras. The second choice I found that reflected IR best
was surprinsingly black craft paper so you may want to experiment with what you have around versus
the aesthetic look you want to achieve.

To figure the sensor segments, what I do is:

* Run `bin/translate --simulate`.
* Connect to the status page at [http://localhost:8088](http://localhost:8088).
* Save each camera image locally, open with a paint application and figure the coordinates I want to use for each sensor.
* Since each piece of white tape covers about 3 ties, I try to pick a segment that starts 3 ties before the tape and
  finished 3 ties after.
* Edit `~/.translaterc` with that information.
* Once I'm done, I restart `bin/translate`.

Tip: in the [status page](http://localhost:8088) you can use the "Reload Config" link to force Translate
to re-read the ~/.translaterc file.
This however is _only safe to do if you do **not change** the number of sensors_.








 


