import math
import cv2
import numpy as np
# from networktables import NetworkTables as nt

# nt.initialize(server="10.14.77.2")
# table = nt.getTable("spindexer")

cap = cv2.VideoCapture('conevid.mp4')

while True:
    ret, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([9, 196, 99])
    upper_yellow = np.array([200, 255, 255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    frame = cv2.bitwise_and(frame, frame, mask=mask)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContourArea = 0
    largestContour = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > largestContourArea:
            largestContourArea = area
            largestContour = contour
            
    if largestContourArea != 0:

        perimeter = cv2.arcLength(largestContour, True)
        approx = cv2.approxPolyDP(largestContour, 0.1 * perimeter, True)

        if len(approx) == 3:
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)

            x1 = approx[0][0][0]
            x2 = approx[1][0][0]
            x3 = approx[2][0][0]

            y1 = approx[0][0][1]
            y2 = approx[1][0][1]
            y3 = approx[2][0][1]

            d1 = math.sqrt(math.pow(x1 - x2, 2) + pow(y1 - y2, 2))
            d2 = math.sqrt(math.pow(x2 - x3, 2) + pow(y2 - y3, 2))
            d3 = math.sqrt(math.pow(x3 - x1, 2) + pow(y3 - y1, 2))

            sides = [d1, d2, d3]

            shortest_side = min(sides)

            tipX = 0
            tipY = 0

            if shortest_side == d1:
                tipX, tipY = x3, y3
            elif shortest_side == d2:
                tipX, tipY = x1, y1
            elif shortest_side == d3:
                tipX, tipY = x2, y2


            cv2.circle(frame, (tipX, tipY), radius=10,
                    color=(0, 0, 255), thickness=-1)
            cv2.putText(frame, "Tip ({}, {})".format(tipX, tipY), (tipX,
                        tipY), cv2.FONT_ITALIC, 2, color=(255, 0, 0), thickness=4)

            table.putNumber("tip-x", tipX)
        else:
            table.putNumber("tip-x", -1477)
            print("No tip found")
    else:
        table.putNumber("tip-x", -1477)


    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
