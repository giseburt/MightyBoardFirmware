#!/usr/bin/python
#
# generates a table of inverse feedrates, to be used in acceleration algorithm

# usage python GenerateFeedrateTable.py 

import sys
import struct
import numpy as np
import matplotlib.pyplot as plt


t1 = np.arange(65536);
t2 = np.arange(65536);
t3 = np.zeros(1024);
for n in range(1, 65536):
	t2[n] = 1000000.0 / n;


print "static uint32_t ifeedrate_table[] PROGMEM = {",

k = 0;
idx = 0;
for n in range(0,32):
	data = 0;
	print "%s, " % (data) ,
	k = k+1;
	t3[idx] = data;
	idx = idx+1;

for n in range(32,256 + 128):
	data = 1000000 / n;
	print "%s, " % (data) ,
	if t1[k] != n:
		print "fail : %d %d" % (t1[k], n)
		sys.exit("FAIL");
	k = k+1;
	t3[idx] = data;
	idx = idx+1;

for n in range(96,256):
	data = 1000000 / (n*4);
	print "%s, " % (data) ,
	if (t1[k] >> 2) != n:
		print "fail : %d %d" % (t1[k], n)
		sys.exit("FAIL");
	k = k+4;
	t3[idx] = data;
	idx = idx+1;

for n in range(64,256):
	data = 1000000 / (n*16);
	print "%s, " % (data) ,
	if (t1[k] >> 4) != n:
		print "fail : %d %d" % (t1[k], n)
		sys.exit("FAIL");
	k = k+16;
	t3[idx] = data;
	idx = idx+1;

for n in range(16,256):
	data = 1000000 / (n*256);
	print "%s, " % (data) ,
	if (t1[k] >> 8) != n:
		print "fail : %d %d" % (t1[k], n)
		sys.exit("FAIL");
	k = k+256;
	t3[idx] = data;
	idx = idx+1;
	
print "};"

array1idx = 0;
array2idx = 256 + 128 - 96;
array3idx = array2idx + 256 - 64;
array4idx = array3idx + 256 - 16;

for n in range(0,256+128):
	if n >= 32:
		if(1000000/n != t3[array1idx + n]):
			print "fail : %d %d" % (n, t3[array1idx + n] )
			sys.exit("FAIL");
for n in range(256+128,1024,4):
	if(1000000/n != t3[array2idx + (n>>2)]):
		print "fail : %d %d %d %d" % (n, t3[array2idx + (n>>2)], array2idx + (n>>2), 1000000/n)
		sys.exit("FAIL");

for n in range(1024,4096,16):
	if(1000000/n != t3[array3idx + (n>>4)]):
		print "fail : %d %d %d %d" % (n, t3[array3idx + (n>>4)], array3idx + (n>>3), 1000000/n)
		sys.exit("FAIL");

for n in range(4096,65536,256):
	if(1000000/n != t3[array4idx + (n>>8)]):
		print "fail : %d %d %d %d" % (n, t3[array4idx + (n>>8)], array4idx + (n>>8), 1000000/n)
		sys.exit("FAIL");

diff = np.zeros(65536);
for n in t1:
	if n >= 4096:
		diff[n] = 1000000/n - t3[array4idx + (n >> 8)];
	elif n >= 1024:
		diff[n] = 1000000/n - t3[array3idx + (n >> 4)];	
	elif n >= 256+128:
		diff[n] = 1000000/n - t3[array2idx + (n >> 2)];
	elif n >= 32:
		diff[n] = 1000000/n - t3[array1idx + n];

plt.plot(diff[200:5000])
plt.show()

print "slow %d" % (array1idx)
print "mid1 %d" % (array2idx)
print "mid2 %d" % (array3idx)
print "fast %d" % (array4idx)
	 
	
	



