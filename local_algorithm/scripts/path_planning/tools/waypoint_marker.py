# import the necessary packages
import cv2

waypoints = []


def click_and_crop(event, x, y, flags, param):
    # grab references to the global variables
    global waypoints
    # if the left mouse button was clicked, record the starting (x, y) coordinate
    if event == cv2.EVENT_LBUTTONDOWN:
        waypoints.append((x, y))


image = cv2.imread(
    "/home/ycyao/Projects/pdRosRepo/project-drive-2020/local_algorithm/scripts/path_planning/tools/berlin.png"
)
clone = image.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)
# keep looping until the 'q' key is pressed
while True:
    # display the image and wait for a keypress
    cv2.imshow("image", image)
    key = cv2.waitKey(1) & 0xFF
    # if the 'c' key is pressed, break from the loop
    if key == ord("c"):
        break
    for coords in waypoints:
        cv2.circle(image, coords, radius=2, color=(225, 0, 0), thickness=2)


print(waypoints)

cv2.destroyAllWindows()
