import cv2
import numpy as np
import pdb
import sys

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

cone_template_path = './test_images_stata_line/line1.png'

def image_print(img):
        """
        Helper function to print out images, for debugging. Pass them in as a list.
        Press any key to continue
        """
        cv2.imshow("image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def crop_image(img, top_left_inclusive, bottom_right_exclusive):
    r_min, c_min = top_left_inclusive
    r_max, c_max = bottom_right_exclusive
    return img[r_min:r_max, c_min:c_max]

def filter_contours(contours):
    if len(contours) == 0:
        return None
    
    return [contour for contour in contours if cv2.contourArea(contour) > 1000] # WILL NEED TO CHANGE

def get_largest_contour(contours):
    if len(contours) == 0:
        return None
    largest_area = cv2.contourArea(contours[0])
    largest_contour = contours[0]
    x, y, w, h = cv2.boundingRect(contours[0])
    #print("# contours", len(contours))
    for contour in contours[1:]:
        new_area = cv2.contourArea(contour)
        if new_area > largest_area:
            x, y, w, h = cv2.boundingRect(largest_contour)
	    #print("w, h", w, h)
	    if w/h < 0.85 and w/h > 0.65:
	    	largest_area = new_area
            	largest_contour = contour
    #print("largest_w_h", w, h)
    return largest_contour
    #return max(contours, key=lambda x: cv2.contourArea(x))

def cd_color_segmentation(img, template=None):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template_file_path; Not required, but can optionally be used to automate setting hue filter values.
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px                                                                    (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
        """

    # Create mask for orange cone. HSV threshods
    light_orange = (100, 100, 100) #(60, 170, 125) #(70, 180, 150)
    dark_orange = (180, 255, 255) #(170, 255, 255) #(150, 255, 255)
    kernel = np.ones((7, 7), np.uint8)

    filtered_img = cv2.dilate(cv2.erode(img, kernel, iterations=1), kernel, iterations=1)
    
    #image_print(filtered_img)
    hsv_img = cv2.cvtColor(filtered_img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_img, light_orange, dark_orange)
    #image_print(mask)
    #print('mask.shape', mask.shape)
    cropped_mask = crop_image(mask, (mask.shape[0]*2//3, 0), (mask.shape[0], mask.shape[1]))
    #print('cropped mask shape', cropped_mask.shape)
    # Find remaning contours, correspond to orange objects
    
    contours = cv2.findContours(cropped_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
   # print('num contours', len(contours))
    #print(contours)
    largest_contour = get_largest_contour(contours)
    #print(largest_contour)

    # Draw boxes around the contours
    x, y, w, h = cv2.boundingRect(largest_contour)
    bounding_box = ((x, y+mask.shape[0]*2//3), (x+w, y+h+mask.shape[0]*2//3))
    #print(bounding_box)
    return bounding_box
                                                           
if __name__ == '__main__':
    cone_image = cv2.imread(cone_template_path)
    #image_print(cone_image)
    boundary_box = cd_color_segmentation(cone_image)
    cv2.rectangle(cone_image, boundary_box[0], boundary_box[1], (0, 255, 0),2)
    image_print(cone_image)
