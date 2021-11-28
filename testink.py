#!/usr/bin/env python3


import mtp02_ioctl

f=open("/dev/mtp02.0","wb")

settings=mtp02_ioctl.get_settings(f)

full_line = bytes([0xff])*48

for i in range(250,0,-1):
    settings['burn_time'] = i
    mtp02_ioctl.set_settings(f,settings)
    f.write(full_line)
    f.flush()