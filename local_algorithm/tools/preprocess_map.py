import cv2
import numpy as np
map_img = cv2.imread('../scripts/path_planning/tools/berlin.png')
map_img = map_img[:,:,0]
map_img = cv2.blur(map_img, (15,15))
threshold, map_img = cv2.threshold(map_img, 254, 255, cv2.THRESH_BINARY)
np.savetxt('berlin_blurred.txt', map_img)
cv2.imwrite('berlin_blurred.png', map_img)
