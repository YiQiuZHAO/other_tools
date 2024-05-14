from PIL import Image
from PIL import ImageSequence
from glob import glob
import cv2
import os.path as op
import numpy as np

path = '..'
files = glob('/'.join([path,'*.gif']))


for file in files:
    name = op.splitext(op.split(file)[1])[0]
    img = Image.open(file)
    x,y = img.size[:2]
    frame_size = (x,y)
    fps = 15
    # Initialize video writer object
    output = cv2.VideoWriter('/'.join([path,name+'.avi']), cv2.VideoWriter_fourcc('M','J','P','G'), fps, frame_size)
    for frame in ImageSequence.Iterator(img):
        output.write(cv2.cvtColor(np.asarray(frame),cv2.COLOR_RGB2BGR))
        # cv2.waitKey(1000/fps)
    output.release()