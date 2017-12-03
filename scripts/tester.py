#!/usr/bin/env python
from OSC import OSCServer
from time import sleep

server = OSCServer( ("localhost", 3131) )
server.timeout = 0
run = True

def user_callback(path, tags, args, source):
   print path, args



server.addMsgHandler( "/poses", user_callback )

# user script that's called by the game engine every frame
def each_frame():
    # clear timed_out flag
    server.timed_out = False
    # handle all pending requests then return
    while not server.timed_out:
        server.handle_request()

while True:
    server.timed_out = False
    # handle all pending requests then return
    while not server.timed_out:
        server.handle_request()

