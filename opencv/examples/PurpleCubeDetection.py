import cv2
import numpy as np
folderpath = 'C:\\Users\\driver\\Desktop\\aryan\\612-2024-Preseason'
cap = cv2.VideoCapture(0)
Known_distance = 40
Known_width = 25
# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

def polygons(tempp, output):
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
    # draw in blue the contours that were founded
        cv2.drawContours(output, contours, -1, 255, 3)

    # find the biggest countour (c) by the area
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)

    # draw the biggest contour (c) in green
        cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)

# show the images
    cv2.imshow("Result", np.hstack([tempp, output]))
    return w

ref_image = cv2.imread(folderpath+'\\opencv\\images\\box_image.jpg')
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image): 

	# finding the focal length 
	focal_length = (width_in_rf_image * measured_distance) / real_width 
	return focal_length 

def Distance_finder(Focal_Length, real_face_width, face_width_in_frame): 

	distance = (real_face_width * Focal_Length)/face_width_in_frame 

	# return the distance 
	return distance 
hsv = cv2.cvtColor(ref_image, cv2.COLOR_BGR2HSV) 
lower_blue = np.array([110,50,50]) 
upper_blue = np.array([130,255,255]) 
  
    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame. 
mask = cv2.inRange(hsv, lower_blue, upper_blue) 
 
    # The bitwise and of the frame and mask is done so  
    # that only the blue coloured objects are highlighted  
    # and stored in res 
res = cv2.bitwise_and(ref_image,ref_image, mask= mask) 
ret,thresh = cv2.threshold(mask, 40, 255, 0)
ref_image_w = polygons(ref_image, res)
Focal_length_found = Focal_Length_Finder( Known_distance, Known_width, ref_image_w)
print(Focal_length_found)
while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    lower_blue = np.array([110,50,50]) 
    upper_blue = np.array([130,255,255]) 
  
    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame. 
    mask = cv2.inRange(hsv, lower_blue, upper_blue) 
  
    # The bitwise and of the frame and mask is done so  
    # that only the blue coloured objects are highlighted  
    # and stored in res 
    res = cv2.bitwise_and(frame,frame, mask= mask) 
    ret,thresh = cv2.threshold(mask, 40, 255, 0)
    res_polygons = polygons(frame, res)
    if res_polygons != 0:
        Distance = Distance_finder( Focal_length_found, Known_width, res_polygons) 
        print(Distance) 
    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()