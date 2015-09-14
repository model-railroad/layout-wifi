# Translate & LayoutWifi #

This repository contains 3 utilities that I use to manage my H0 model-train DCC layout at home:

* **LayoutWifi** runs on a DigiX (an Arduino clone) to control physical turnouts and sensors.
* **Translate** is a service that interfaces LayoutWifi to both RocRail or JMRI. 
* **Translate** also performs train block detection by analyzing nigh-vision webcams.

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
    * On the other side, it connect to one or more webcams and analyze the images to provide block occupancy sensors.
    * Finally it interfaces with RocRail via an SRCP protocol or with JMRI via an NCE binary protocol.

~~~~
  DigiX                                 +-------[SRCP]---> RocRail ---+
LayoutWifi   <---->   Translate     <---|                             |---> NCE USB ---> NCE Booster / DCC
    |                     ^             +-------[NCE]----> JMRI ------+
    v                     |
 turnouts              webcams
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