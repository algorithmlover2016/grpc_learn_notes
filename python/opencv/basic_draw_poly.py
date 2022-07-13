# -*-coding:utf-8 -*-

# refer to https://segmentfault.com/a/1190000015585635 and
# cv2.line(), cv2.circle() , cv2.rectangle(), cv2.ellipse(), cv2.putText()
import cv2
import numpy as np
import pdb
pdb.set_trace()

# https://blog.csdn.net/qq_33757398/article/details/96174768
# 以灰度方式读取图像
img = cv2.imread('./data/img.png', cv2.IMREAD_GRAYSCALE)
mask = img.copy()
 
# 二值化，100为阈值，小于100的变为255，大于100的变为0
# 也可以根据自己的要求，改变参数：
# cv2.THRESH_BINARY
# cv2.THRESH_BINARY_INV
# cv2.THRESH_TRUNC
# cv2.THRESH_TOZERO_INV
# cv2.THRESH_TOZERO
_, binaryzation = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY_INV)
 
# 找到所有的轮廓
contours, _ = cv2.findContours(binaryzation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
print(f"len: {len(contours)}")
 
area = []
 
# 找到最大的轮廓
for k in range(len(contours)):
	area.append(cv2.contourArea(contours[k]))
max_idx = np.argmax(np.array(area))
 
# 填充最大的轮廓
for k in range(len(contours)):
    t_val = max(0, 255 - k * 10)
    # mask = cv2.drawContours(mask, contours, max_idx, 255, cv2.FILLED)
    print(f"index: {k}, t_val: {t_val}")
    mask = cv2.drawContours(mask, contours, k, (t_val, t_val, t_val), cv2.FILLED)
 
# 保存填充后的图像
cv2.imwrite('./data/masked.png', mask)