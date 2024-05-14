import cv2
from glob import glob

# files = glob('C:/Users/zhaoy/OneDrive/code/ultralytics-main/V8/runs/segment/track/*.jpg')
files = glob('C:/Users/zhaoy/OneDrive/code/ultralytics-main/V8/runs/segment/predict2/*.jpg')

frame_width = 960
frame_height = 540
frame_size = (frame_width,frame_height)
fps = 30
# Initialize video writer object
output = cv2.VideoWriter('output_video_from_file.avi', cv2.VideoWriter_fourcc('M','J','P','G'), fps, frame_size)
for file in files:
    # vid_capture.read() methods returns a tuple, first element is a bool 
    # and the second is frame

    img = cv2.imread(file)

    output.write(img)
output.release()

