from __future__ import print_function
import numpy as np
import cv2
import glob
import time
import sys
from processor import FrameProcessor, Timer

def main():
    fp = FrameProcessor()
    files = sorted(sys.argv[1:])

    for f in files[:-10]:
        im = cv2.imread(f, cv2.CV_8UC1)
        # big = cv2.resize(im, (0, 0), fx=2, fy=2)
        fp.process(im)

    


if __name__ == "__main__":
    main()
