# R2D2_SHADOW_variant
My personal variant of the S.H.A.D.O.W. variant of the Padawan control system for Astromech or R2D2 droids

As of 10 Feb 2020, this is still a Work In Progress.  The Nano and Leonardo sketches are 99% complete, and the ADK sketch (e.g. the "tri-
processor" sketch) is approximately 80-90% complete.  I am just down to final testing and installations.

This is the collection of the schematic and arduino sketches for my R2DPool Astromech.  This unit will not be an "offical" or "approved"
astromech, but it is mine.

My primary processor uses an Arduino Mega ADK with an Arduino Leonardo + Adafruit Music Maker Shield in the body.  Then I have an
Arduino Nano controlling my dome.  My FLD and RLD (Front and Rear Data displays) are Adafruit 870 & 3152 mini 0.8" x 0.8" LED matrices.
The Nano controls the matrices and the ADK controls the Nano via a serial feed.
