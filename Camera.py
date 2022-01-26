
import cv2
import numpy as np
from time import time


def open_cap():
    cap = cv2.VideoCapture(0) # Define Webcam 0
    ap = cv2.VideoCapture('http://192.168.6.97:6001/?action=stream')
    # Setting Camera Resolution
    cap.set(3, 640)
    cap.set(4, 480)
    return cap


def cap_frame(cap): # Capture frame
    ret, frame_ori = cap.read()
    while not ret:
        ret, frame_ori = cap.read()
    return frame_ori


def find_temp(gray, tmp_name, min_thresh, y_t, rx_t, ry_t, div='i'):
    template = cv2.imread(tmp_name)
    template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    # run template matching, get minimum val
    res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    match_locations = np.where(res > min_thresh)

    # draw template match boxes
    w, h = template.shape[::-1]
    rets = []
    for (x, y) in zip(match_locations[1], match_locations[0]):
        ok = False
        if len(rets) == 0 and ((y < y_t and div == 'i') or
                               (y > y_t and div == 's')):
            rets.append((x, y))
        else:
            ok = False
            for i in rets:
                rx = abs(i[0] - x)
                ry = abs(i[1] - y)
                if (rx > rx_t and ry > ry_t) and ((y < y_t and div == 'i') or
                                                  (y > y_t and div == 's')):
                    ok = True
                else:
                    ok = False
            if ok:
                rets.append((x, y))
    return rets, w, h


def coord2protocol(coords,cx,cy): #converter pixel in (mm)
    c = []
    if len(coords) > 0:
        for coord in coords:
            if coord is not None:
                x = 50-(100*coord[0])/cx #converter x
                y = 50-(100*coord[1])/cy
                c.append(str(x) + ',' + str(y) + ',20')
        return c
    return '-1 -1 -1'

def process_temp(frame, box, color):
    """
    Process template box in image trying to find the centroid
    of a circular region of a given color
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    x, y, w, h = box
    total_area = w * h
    h_box = hsv[y:y + h, x:x + w, 0].copy()
    v_box = hsv[y:y + h, x:x + w, 2].copy()
    if color == 'v':
        # lower mask (0-10)
        lower_red = np.array([0])
        upper_red = np.array([10])
        mask0 = cv2.inRange(h_box, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170])
        upper_red = np.array([180])
        mask1 = cv2.inRange(h_box, lower_red, upper_red)
        # join my masks
        mask = mask0 + mask1

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    #    print color
    if color == 's':
        _, mask = cv2.threshold(v_box, 160, 210, cv2.THRESH_BINARY_INV)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    if color == 'p':
        _, mask = cv2.threshold(v_box, 87, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    im2, contours, hierarchy = cv2.findContours(mask, 1, 2)
    centers = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area >= 0.25 * total_area:
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centers.append([cx + x, cy + y])
    if len(centers) > 0:
        return centers[0]


def detect_grid(frame_ori):
    cv2.imwrite("frame.png", frame_ori)
    hsv = cv2.cvtColor(frame_ori, cv2.COLOR_BGR2HSV) #COLOR_BGR2GRAY
    mask0 = cv2.inRange(hsv, (0, 5, 0), (255, 25, 255)) #white mask (0,5,0) (255,25,255)
    kernel = np.ones((5, 5), np.uint8)
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_OPEN, kernel)
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_CLOSE, kernel)
    # find the contours in the mask
    _, cnts, _ = cv2.findContours(
        mask0.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    areas = list(map(cv2.contourArea, cnts))
    dict_area = dict(zip(areas, cnts))
    cnt_area = sorted(dict_area.keys(), reverse=True)
    c = dict_area[cnt_area[0]]

    rect = cv2.minAreaRect(c)
    center = tuple([int(x) for x in rect[0]])
    shape = tuple([int(x) for x in rect[1]])

    angle = rect[2]
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(frame_ori, [box], 0, (0, 0, 255), 2) #
    frame_ori = cv2.circle(frame_ori, center, 3, (0, 0, 255), -1)
    #    print "center"
    #    print center
    h, w = mask0.shape
    cx = (w)/2 #320
    cy = (h)/2 #240
    tx, ty = (cx - center[0], cy - center[1])

    Mt = np.float32([[1, 0, tx], [0, 1, ty]])
    dst = cv2.warpAffine(frame_ori, Mt, (w, h))
    dst = cv2.transpose(dst)
    cv2.imwrite("dst2.png", dst)

    x0 = cx - shape[0] / 2
    x1 = cx + shape[0] / 2 + 1
    y0 = cy - shape[1] / 2
    y1 = cy + shape[1] / 2 + 1

    print('dst', dst)
    print('x', x0, x1)
    print('y', y0, y1)

    cropped = dst[x0:x1, y0:y1]

    return cropped, shape[0], shape[1]

def detect_pieces(cap):
    silver = '-1 -1 -1'
    black = '-1 -1 -1'
    frame_ori = cap_frame(cap)
    # frame_ori = cv2.imread('frame.png',1)   #####para teste 
    frame, cx, cy = detect_grid(frame_ori)
    print ('(cx, cy) = ({}, {})'.format(cx,cy))
    cv2.imwrite("frame2.png", frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rets_s, w_s, h_s = find_temp(
        gray, 'ts.png', min_thresh=0.75,
        y_t=cy, rx_t=15, ry_t=15, div='i'
    )
    boxes_s = [(c[0], c[1], w_s, h_s) for c in rets_s]
    centers_s = [process_temp(frame, box, 's') for box in boxes_s]
    print ('Centers_s: {}'.format(centers_s))

    rets_p, w_p, h_p = find_temp(
        gray, 'tp.png', min_thresh=0.75,
        y_t=cy, rx_t=15, ry_t=15, div='i'
    )
    boxes_p = [(c[0], c[1], w_p, h_p) for c in rets_p]
    centers_p = [process_temp(frame, box, 'p') for box in boxes_p]
    print ('Centers_p: {}'.format(centers_p))
    frame = draw_detections(frame, rets_s, centers_s, w_s, h_s)
    silver = coord2protocol(centers_s,cx,cy)
    frame = draw_detections(frame, rets_p, centers_p, w_p, h_p)
    black = coord2protocol(centers_p,cx,cy) #centers coords in pixels
    print ('black in mm: {}'.format(black))
    print ('silver in mm: {}'.format(silver))
    return frame, silver, black


def draw_detections(frame, rets, centers, w, h):
    for b, c in zip(rets, centers):
        if b is not None:
            frame = cv2.rectangle(
                frame, (b[0], b[1]), (b[0] + w, b[1] + h), #+1 extra
                (0, 255, 0), 3
            )
            frame = cv2.circle(
                frame, (c[0], c[1]), np.floor((h + w) / 8).astype(int),
                (0, 255, 0), 3
            )
        if c is not None:
            frame = cv2.circle(
                frame, (c[0], c[1]), 2,
                (0, 255, 0), 3
            )
    return frame

print ("Starting")
cap = open_cap()
color, silver, black = detect_pieces(cap)

file = open("posxyz.txt", "w")
for piece in black:
    y = float(piece.split(',')[0])/100
    x = (float(piece.split(',')[1])/100)*-1    
    z = float(piece.split(',')[2])/100
    piece = str(x)+', '+str(y)+', '+str(z)
    file.write(piece+", b\n")
for piece in silver:
    y = float(piece.split(',')[0])/100
    x = (float(piece.split(',')[1])/100)*-1    
    z = float(piece.split(',')[2])/100
    piece = str(x)+', '+str(y)+', '+str(z)    
    file.write(piece+", s\n")
file.close()

#    cv2.imshow('detections', color)
cv2.imwrite('detections.png', color)
cap.release()
print ("finish")

#Red = Hue(0...9) AND Hue(151..180)
#Orange = Hue(10...15)
#Yellow = Hue(16..45)
#Green = Hue(46..100)
#Blue = Hue(101..150)
