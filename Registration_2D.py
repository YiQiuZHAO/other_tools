import cv2
import numpy as np


def imgshow(imgs, name=[], key=0, des=True):
    if type(imgs) == list:
        for i, img in enumerate(imgs):
            n = name[i] if len(name) == len(imgs) else str(i)
            cv2.namedWindow(n, cv2.WINDOW_NORMAL)
            cv2.imshow(n, img)
            if key != 0:
                cv2.waitKey(key)
        cv2.waitKey(key)
    else:
        n = name if len(name) == len(imgs) else '1'
        cv2.namedWindow(n, cv2.WINDOW_NORMAL)
        cv2.imshow(n, imgs)
        cv2.waitKey(key)
    if des:
        cv2.destroyAllWindows()


img1 = cv2.imread('out46.jpg')
img2 = cv2.imread('out47.jpg')

# mtx1 = np.asarray(
#     [9.842439e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00, 9.808141e+02, 2.331966e+02, 0.000000e+00, 0.000000e+00,
#      1.000000e+00]).reshape((3, 3))
# mtx2 = np.asarray(
#     [9.037596e+02, 0.000000e+00, 6.957519e+02, 0.000000e+00, 9.019653e+02, 2.242509e+02, 0.000000e+00, 0.000000e+00,
#      1.000000e+00]).reshape((3, 3))
# dist1 = np.asarray([-3.728755e-01, 2.037299e-01, 2.219027e-03, 1.383707e-03, -7.233722e-02])
# dist2 = np.asarray([-3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02])
# img1 = cv2.undistort(img1, mtx1, dist1)
# img2 = cv2.undistort(img2, mtx2, dist2)

gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

sift = cv2.SIFT_create()
(kps1, features1) = sift.detectAndCompute(gray1, None)
(kps2, features2) = sift.detectAndCompute(gray2, None)

bf = cv2.BFMatcher()
matches = bf.knnMatch(features1, features2, 2)

good_matches = []
for m, n in matches:
    if m.distance < 0.75 * n.distance:
        good_matches.append([m])
ptsA = np.int32([kps1[m[0].queryIdx].pt for m in good_matches]).reshape(-1, 2)
ptsB = np.int32([kps2[m[0].trainIdx].pt for m in good_matches]).reshape(-1, 2)

H, status = cv2.findHomography(ptsA, ptsB, cv2.RHO, 4)
imgOut = cv2.warpPerspective(img2, H, (img1.shape[1], img1.shape[0]), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)

rate = 0.5
overlapping = cv2.addWeighted(cv2.cvtColor(gray1, cv2.COLOR_GRAY2BGR), rate, imgOut, 1 - rate, 0)

imgshow([img1, img2, imgOut, overlapping])
