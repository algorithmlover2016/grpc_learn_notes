# [cv2.draw](https://segmentfault.com/a/1190000015585635)<br>
## [draw line]<br>
```py
# refer to https://machinelearningknowledge.ai/quick-guide-for-drawing-lines-in-opencv-python-using-cv2-line-with-examples/#Line_in_OpenCV_Python_cv2line
# cv2.line(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

# Draw a diagonal blue line with thickness of 5 px
cv2.line(img,(0,0),(511,511),(255,0,0),5)
cv2.imshow('line',img)
cv2.waitKey()
```
## [draw rectangle]<br>
```py
# cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

cv2.rectangle(img,(384,0),(510,128),(0,255,0),3)

cv2.imshow('line',img)
cv2.waitKey()
```

## [draw circle]<br>
```py
# cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

cv2.rectangle(img,(384,0),(510,128),(0,255,0),3)
cv2.circle(img,(447,63), 63, (0,0,255), -1)


cv2.imshow('line',img)
cv2.waitKey()
```

## [ellipse]<br>
```py
# cv2.ellipse(img, center, axes, angle, startAngle, endAngle, color[, thickness[, lineType[, shift]]])
import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

cv2.ellipse(img,(256,256),(100,50),0,0,180,255,-1)

cv2.imshow('line',img)
cv2.waitKey()
```

## [polylines](https://www.geeksforgeeks.org/python-opencv-cv2-polylines-method/)<br>
```py
# cv2.polylines(img, pts, isClosed, color[, thickness[, lineType[, shift]]])
import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
pts = pts.reshape((-1,1,2))
cv2.polylines(img,[pts],True,(0,255,255))

cv2.imshow('line',img)
cv2.waitKey()
```

## [put text]<br>
```py
# cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(img,'OpenCV',(10,500), font, 4,(255,255,255),2,cv2.LINE_AA)

cv2.imshow('show',img)
cv2.waitKey()
```
