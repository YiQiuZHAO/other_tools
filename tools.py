import cv2
import numpy as np


def img_show(imgs):
    if type(imgs) == list:
        for i, img in enumerate(imgs):
            cv2.namedWindow(str(i), cv2.WINDOW_NORMAL)
            img = (img * (255 / img.max())).astype('uint8')
            cv2.imshow(str(i), img)
    elif type(imgs) == np.ndarray:
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        img = (imgs * (255 / imgs.max())).astype('uint8')
        cv2.imshow('frame', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
