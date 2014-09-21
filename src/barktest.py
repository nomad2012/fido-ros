#!/usr/bin/env python
import time
import subprocess

def play_sound(filename):
    pipe = subprocess.Popen(['mpg123', filename])

while True:
    play_sound('/home/dennis/Downloads/bark03.mp3')
    time.sleep(2.0)