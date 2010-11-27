Aiko: Arduino Framework
=======================

A small modular, event-driven framework for structuring Arduino
sketches, such that individual device drivers can be easily
componentized and combined into a single application.

Aiko allows you to write event-driven code:

    #include <AikoEvents.h>
    using namespace Aiko;

    int ledPin = 13;

    void setup() {
      pinMode(ledPin, OUTPUT);
      Events.addHandler(blink, 1000);  // Every 1000ms
    }

    void loop() {
      Events.loop();
    }

    void blink() {
      static boolean on = HIGH;
      digitalWrite(ledPin, on);
      on = !on;
    }

Writing individual device drivers as event-driven functions makes it
simpler to create device specific modules that can be shared with others
and easier to then combine and integrate them with less code changes.


Installation
============

Get the samotage fork of the Aiko codeline:
https://github.com/samotage/Aiko

You will need some libraries for Arduino to compile correctly,

Change into your Arduino libraries folder and use git to clone the project.

On a Mac this looks like:

    cd /Applications/Arduino.app/Contents/Resources/Java/libraries

    into this directory copy the whole Aiko directory as a library.
    this will include necessary resources for the SEGmeter.
   
On Linux, this will directory will be wherever you install your Arduino
software.

Other dependencies
==================

There are some other things that the SEGmeter code needs to work fully, these are 
separate Arduino libraries that need to be instlled into the library directory above.

They are:

NewSoftSerial
PString
OneWire

Whilst it's possible to search for these resources in the intertubes, 
they have been packaged up here for your benifit.

So, go to here:
https://github.com/samotage/Aiko/tree/master/other_arduino_libraries/

And add each of the above directories into the above mentioned Arduino Library directory.



Modules
=======

- **Callback** - Easy to use function and method callbacks.
- **Events** - Schedule regular callbacks so you can easily deal with
  a bunch of devices connected to your Arduino.
- **SExpression** - Parse simple SExpressions. Think of this as the
  Arduino equivalent of JSON.
- **Timing** - Accurate timing, including better replacements for the
  standard Arduino timing functions.

See the corresponding files in the docs directory for more info on each module.


Community
=========

Report bugs on our [GitHub bug tracker](http://github.com/geekscape/Aiko/issues).

Written by Andy Gelme, Jonathan Oxer, and Pete Yandell.

Copyright (C) 2009 by Geekscape Pty. Ltd.
Copyright (C) 2009 by Jonathan Oxer.
Copyright (C) 2009 by Pete Yandell.
Released under the GPLv3 license (dual-licensed).
