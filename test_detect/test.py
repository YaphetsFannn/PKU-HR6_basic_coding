import cv2
import numpy as np

fim = np.loadtxt('rgb.txt')
im = np.zeros((480, 640, 3), np.uint8)
row, col, _ = im.shape

for i in range(row):
    for j in range(col):
        im[i, j, 0] = fim[i*3*col+j*3]
        im[i, j, 1] = fim[i*3*col+j*3+1]
        im[i, j, 2] = fim[i*3*col+j*3+2]

cv2.imshow('imshow', im)
cv2.waitKey(0)
cv2.destroyAllWindows()




