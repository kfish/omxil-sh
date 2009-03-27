omxil-sh
========

omxil-sh: A collection of OpenMAX IL components for SH-Mobile,
using the Bellagio OpenMAX IL project framework.

Copyright (C) 2009 Renesas Technology Corp.

Installation
------------

This source archive uses the GNU autotools to build. Ensure that the standard GNU
build environment is available, including autoconf, automake, pkg-config.

To build:

    autoreconf -vif
    ./configure
    make
    make install

Then register the resulting OpenMAX IL components on the SH-Mobile host:

    $ omxil-bellagio-register

Testing
-------

To test the audio components, use the omxsh-decode-audio tool. This is installed
when you do 'make install'.

Playback to an ALSA sound device:

    $ omxsh-decode-audio testfile.mp3

    $ omxsh-decode-audio testfile.aac

Decode to a file:

    $ omxsh-decode-audio -o /tmp/testfile.raw testfile.mp3

    $ omxsh-decode-audio -o /tmp/testfile.raw testfile.aac

When decoding audio to a file, the output will be written in the following raw PCM format:

    Sample Size    : 16-bit (2 bytes)
    Sample Encoding: signed (2's complement)
    Channels       : 2
    Sample Rate    : 44100

It can be played back with a command such as the following (using 'play' on a host
with sox installed):

    $  play -s -2 -c2 -r44100 testfile.raw

License
-------

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston MA  02110-1301 USA

See the file COPYING for details.
