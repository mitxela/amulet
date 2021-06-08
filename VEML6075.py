#!/bin/python3

import time, serial, sys

s = serial.Serial(port='/dev/ttyUSB0',baudrate=115200,timeout=0.01)

#[0x20 0x00 0x40]
#[0x20 0x07 [ 0x21 rr]
#[0x20 0x09 [ 0x21 rr]

def buscmd(cmd, lines=1):
  global s
  s.write(cmd.encode() + b'\n')
  r = b''
  line = s.readline()
  while line[-1:] !=b'>':
    r += line
    line = s.readline()
  #print(r.decode())
  return r

def write_i2c(addr, reg, value):
  buscmd("[ 0x%02x 0x%02x 0x%02x 0x00 ]" % (addr, reg, value))

def read_i2c(addr, reg, num=1):
  r = buscmd("[ 0x%02x 0x%02x [ 0x%02x r:%d ]" % (addr, reg, addr+1, num))
  return r.decode()

def read_reg(reg):
  a = read_i2c(addr, reg, 2)
  a = a.splitlines()[6][6:].split('  ACK ')
  lsb, msb = [int(i,0) for i in a]
  return ((msb<<8)+lsb)


buscmd('#') # reset
buscmd('m') # mode
buscmd('4') # i2c
buscmd('3') # speed 400khz
buscmd('W') # power on

addr = 0x20

HD=1 if len(sys.argv)>1 and int(sys.argv[1])>0 else 0

if HD:
  write_i2c(addr, 0x00, 0x18) # power on, int time 100ms
else:
  write_i2c(addr, 0x00, 0x10) # power on, int time 100ms

time.sleep(0.5)

csv=0
if (csv):
  print("Time,D,uva,uvb,uvcomp1,uvcomp2,dummy,uva_calc,uvb_calc,uvi")

def measure():
  uva = read_reg(0x07)
  uvb = read_reg(0x09)
  uvcomp1 = read_reg(0x0A)
  uvcomp2 = read_reg(0x0B)
  
  dummy = read_reg(0x08)
  
  # revision 2018
  c_a = 2.22
  c_b = 1.33
  c_c = 2.95
  c_d = 1.74
  c_resp_uva = 0.001461
  c_resp_uvb = 0.002591

  # revision 2015
  #c_a = 3.33
  #c_b = 2.5
  #c_c = 3.66
  #c_d = 2.75
  #c_resp_uva = 0.0011
  #c_resp_uvb = 0.00125

  
  uva_calc = uva - c_a*(uvcomp1-dummy) - c_b*(uvcomp2-dummy)
  uvb_calc = uvb - c_c*(uvcomp1-dummy) - c_d*(uvcomp2-dummy)
  
#  uva_calc = uva - c_a*uvcomp1 - c_b*uvcomp2
#  uvb_calc = uvb - c_c*uvcomp1 - c_d*uvcomp2
  
  uvi = ( uva_calc*c_resp_uva + uvb_calc*c_resp_uvb )/2
 
  if (csv):
    print(time.strftime('%H:%M:%S'),",HD" if HD else ",SD",
          f",{uva},{uvb},{uvcomp1},{uvcomp2},{dummy}",
          f",{uva_calc:.1f},{uvb_calc:.1f},{uvi:.1f}")
  else:
    print("\n",time.strftime('%H:%M:%S'), "\t", "HD" if HD else "SD", "\t100ms")
    print(f"raw: {uva},\t{uvb},\t{uvcomp1},\t{uvcomp2},\t{dummy}")
    print(f"UVA: {uva_calc:.1f},\tUVB:{uvb_calc:.1f},\tUVI: {uvi:.1f}")

    r_raw = float(uva)/float(uvb) if uvb>0 else 0
    r_comp = uva_calc/uvb_calc if uvb_calc>0 else 0
    print(f"Ratios: {r_raw}\t{r_comp}")

while (1):
  try:
    measure()
    time.sleep(0.5)
  except KeyboardInterrupt:
    exit()

