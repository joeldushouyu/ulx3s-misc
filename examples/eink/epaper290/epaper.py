import waveshare_epaper290bw296x128
from time import sleep_ms
from random import randint
import framebuf

epd=waveshare_epaper290bw296x128.epaper290(dc=26,din=25,cs=16,clk=17,rst=15,busy=5)
# initialize the frame buffer
fb_size = (epd.width*epd.height)//8
frame = bytearray(fb_size)
fb=framebuf.FrameBuffer(frame, epd.width, epd.height, framebuf.MONO_HLSB)

def clear():
  for i in range(fb_size):
    frame[i]=0xFF

def chequers():
  for x in range(epd.width):
    for y in range(epd.height):
      epd.set_pixel(frame, x,y, ((x//5)^(y//5))&1)

def lines():
  i=0
  while i<epd.width:
    epd.draw_line(frame, 0,0, i,epd.height-10, 0)
    i+=8

def randlines(n):
  for i in range(n):
    fb.line(randint(0,epd.width),randint(10,epd.height), randint(0,epd.width),randint(10,epd.height), 0)
    i+=8

def run():
  #clear()
  #chequers()
  #epd.init()
  #epd.display_frame(frame)
  #epd.sleep()

  #sleep_ms(10000)

  #clear()
  #randlines(100)

  fb.fill(0xFF)
  fb.text("Micropython!", 0,0, 0)
  fb.hline(0,10, 96, 0)
  #fb.line(0,10, 96,106, 0)
  randlines(100)

  epd.init()
  epd.display_frame(frame)
  epd.sleep()

run()