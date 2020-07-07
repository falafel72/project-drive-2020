# import the necessary packages
from __future__ import print_function
import cv2


waypoints = []
vis_waypoints = []
# origin of the car

image = cv2.imread(
    "./berlin.png"
)
origin = (232,image.shape[1]-530)

def click_and_crop(event, x, y, flags, param):
    # grab references to the global variables
    global waypoints
    # if the left mouse button was clicked, record the starting (x, y) coordinate
    if event == cv2.EVENT_LBUTTONDOWN:
        waypoints.append([x, image.shape[1]-y])
        vis_waypoints.append((x,y))

clone = image.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)
# keep looping until the 'q' key is pressed

while True:
    global image
    # display the image and wait for a keypress
    cv2.imshow("image", image)
    key = cv2.waitKey(1) & 0xFF
    # if the 'c' key is pressed, break from the loop
    if key == ord("c"):
        break
    # if pressed r, removed the last input
    if key == ord("r"):
        if (len(waypoints) > 0):
            waypoints.pop()
            vis_waypoints.pop()
            image = cv2.imread("./berlin.png")
            print("\r")
    cv2.circle(image,origin,radius=3,color=(0,225,0),thickness=3)
    for coords in vis_waypoints:
        cv2.circle(image, coords, radius=2, color=(225, 0, 0), thickness=2)
    #print("Current waypoints" + str(waypoints),end="\r")

print("")
print("Final waypoints are: " + str(waypoints))

cv2.destroyAllWindows()
