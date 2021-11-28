#!/usr/bin/env python3

from PIL import Image, ImageOps
import sys

img = Image.open(sys.argv[1])

basewidth = 384

wpercent = (basewidth/float(img.size[0]))
if wpercent < 1:
	hsize = int((float(img.size[1])*float(wpercent)))
	img = img.resize((basewidth,hsize), Image.ANTIALIAS)

ki = img.convert("L")  # Invert: Only works on 'L' images
ki = ImageOps.invert(ki) # Bits are sent with 0 = white, 1 = black in ESC/POS
ki = ki.convert("1") # Pure black and white

width, height = ki.size
result = Image.new(ki.mode, (384, height), 0)
result.paste(ki, (0, 0))

#result.save("a.png")

#print(bytes(result.getdata()))

open("/dev/mtp02.0","wb").write(bytes(result.tobytes()))