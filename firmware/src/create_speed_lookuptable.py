#!/usr/bin/env python

""" Generate the stepper delay lookup table for Marlin firmware. """

import argparse

__author__ = "Ben Gamari <bgamari@gmail.com>"
__copyright__ = "Copyright 2012, Ben Gamari"
__license__ = "GPL"

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('-f', '--cpu-freq', type=int, default=16, help='CPU clockrate in MHz (default=16)')
parser.add_argument('-d', '--divider', type=int, default=8, help='Timer/counter pre-scale divider (default=8)')
args = parser.parse_args()

cpu_freq = args.cpu_freq * 1000000
timer_freq = cpu_freq / args.divider

print "#ifndef SPEED_LOOKUPTABLE_H"
print "#define SPEED_LOOKUPTABLE_H"
print
print '#include "Marlin.h"'
print

print "const uint16_t speed_lookuptable_fast[256][2] PROGMEM = {"

# for i in range(256):
#   print "/*%i / %i  = %i*/" % (timer_freq, ((i*256)+(args.cpu_freq*2)), timer_freq / ((i*256)+(args.cpu_freq*2)))

lookup_slow = []
lookup_fast = []

a = [ timer_freq / ((i*256)+(args.cpu_freq*2)) for i in range(256) ]
b = [ a[i] - a[i+1] for i in range(255) ]
b.append(b[-1])
for i in range(32):
    print "  ",
    for j in range(8):
        print "{%d, %d}," % (a[8*i+j], b[8*i+j]),
        lookup_fast.append((a[8*i+j], b[8*i+j]))
    print 
print "};"
print

print "const uint16_t speed_lookuptable_slow[256][2] PROGMEM = {"
a = [ timer_freq / ((i*8)+(args.cpu_freq*2)) for i in range(256) ]
b = [ a[i] - a[i+1] for i in range(255) ]
b.append(b[-1])
for i in range(32):
    print "  ",
    for j in range(8):
        print "{%d, %d}," % (a[8*i+j], b[8*i+j]),
        lookup_slow.append((a[8*i+j], b[8*i+j]))
    print 
print "};"
print

print "#endif"


for j in range(1, 65536):
  i = j
  if i < args.cpu_freq*2:
    i = args.cpu_freq*2
  i = i - args.cpu_freq*2
  if i >= 8*256:
    index = i >> 8
    gain = lookup_fast[index][1]
    timer_off = ((i & 0xff) * gain) >> 16
    # print timer_off
    timer = lookup_fast[index][0] - timer_off
    # if (float(timer_freq)/float(j) - float(timer) > 1.0):
    print ">%i/%i = %f - (%i) = %f" % (timer_freq, j, float(timer_freq)/float(j), timer, float(timer_freq)/float(j) - float(timer))
  else:
    index = (i >> 1 & 0xffc) / 4
    # print "index: %i" % (index)
    timer = lookup_slow[index][0]
    # print (lookup_slow[index][1] * (i & 0x7))>>3
    timer -= (lookup_slow[index][1] * (i & 0x7))>>3
    # if (float(timer_freq)/float(j) - float(timer) > 1.0):
    print " %i/%i = %f - (%i) = %f" % (timer_freq, j, float(timer_freq)/float(j), timer, float(timer_freq)/float(j) - float(timer))
  