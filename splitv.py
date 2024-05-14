import cv2

v = 'IMG_2949.MP4'
savepath = './save'
cap = cv2.VideoCapture(v)

framenum = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
name_len = len(str(framenum))
i = 0
while True:
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(savepath + '/' + '0' * (name_len - len(str(i))) + str(i) + '.jpg', frame)
    i += 1
    if i == framenum:
        break
cap.release()
