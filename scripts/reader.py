from __future__ import print_function
import numpy as np
import cv2
import glob
import time
import sys
from processor import FrameProcessor, Timer

# usage: python reader.py glob/to/frame/files*.pgm

def main():
    fp = FrameProcessor()
    files = sorted(sys.argv[1:])

    for f in files[:-10]:
        im = cv2.imread(f, cv2.CV_8UC1)
        out = fp.process(im)
        if out is not None:
	        cv2.imshow("out", out)
	        cv2.waitKey(1)

   

if __name__ == "__main__":
    main()
