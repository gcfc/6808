import cv2

cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
    print(frame) # cv2.COLOR_BGR
else:
    rval = False

while rval:
    cv2.imshow("preview", frame)
    rval, frame = vc.read()
    key = cv2.waitKey(1)
    if key == 27: # exit on ESC
        break

    # TODO: DETECT JOINTS
    
    # TODO: MAP TO 3D COORDINATES

cv2.destroyWindow("preview")