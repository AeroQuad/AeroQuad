#!/usr/bin/python

import serial
import os
import sys
import time
from struct import pack

def get_maple_device_path(file_prefix):
    """Try to find the device file for the Maple on OS X; assuming
    that it looks like /dev/<file_prefix>*.  If there are multiple
    possibilities, ask the user what to do.  If the user chooses not
    to say, returns None."""
    possible_paths = [os.path.join('/dev', x) for x in os.listdir('/dev') \
                          if x.startswith(file_prefix)]
    if len(possible_paths) == 0:
        return None
    elif len(possible_paths) == 1:
        return possible_paths[0]
    else:
        print 'Found multiple candidates for the Maple device:'
        for (i,p) in enumerate(possible_paths):
            print '\t%d. %s' % (i+1, p)

        prompt = 'Enter a number to select one, or q to quit: '
        while True:
            resp = raw_input(prompt).strip().lower()
            if resp == 'q': return None

            try:
                i = int(resp, 10)
            except ValueError:
                pass
            else:
                if 0 <= i-1 < len(possible_paths):
                    return possible_paths[i-1]

            prompt = 'Please enter a number from the list, or q to quit: '

os_sysname = os.uname()[0]
if os_sysname == 'Linux':
    maple_path = get_maple_device_path('ttyACM')
    # fall back on /dev/maple if that doesn't work
    if maple_path is None:
        maple_path = '/dev/maple'
        print 'Could not find Maple serial port; defaulting to /dev/maple.'
elif os_sysname == 'Darwin':
    maple_path = get_maple_device_path('tty.usbmodem')
else:
    # TODO [mbolivar] what to do for windows, BSD, whatever?
    maple_path = '/dev/maple'

if maple_path is None:
    print 'Could not find the Maple serial port for reset.', \
        'Perhaps this is your first upload, or the board is already', \
        'in bootloader mode.'
    print
    print "If your sketch doesn't upload, try putting your Maple", \
        'into bootloader mode manually by pressing the RESET button', \
        'then letting it go and quickly pressing button BUT', \
        '(hold for several seconds).'
    sys.exit()

print 'Using %s as Maple serial port' % maple_path

try:
    ser = serial.Serial(maple_path, baudrate=115200, xonxoff=1)
    ser.open()

    # try to toggle DTR/RTS (old scheme)
    ser.setRTS(0)
    time.sleep(0.01)
    ser.setDTR(0)
    time.sleep(0.01)
    ser.setDTR(1)
    time.sleep(0.01)
    ser.setDTR(0)

    # try magic number
    ser.setRTS(1)
    time.sleep(0.01)
    ser.setDTR(1)
    time.sleep(0.01)
    ser.setDTR(0)
    time.sleep(0.01)
    ser.write("1EAF")

    # ok we're done here
    ser.close()

except:
    print 'Failed to open serial port %s for reset.' % maple_path
    sys.exit()

