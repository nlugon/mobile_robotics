# edge detection in an image basic of the basic
# import the necessary packages
import numpy as np
import argparse
import cv2
import matplotlib.pyplot as plt
# construct the argument parse and parse the arguments


def apply_edge_detection(image, method):
    # load the image and convert it to grayscale
    image = cv2.imread(args["image"])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # show the original image
    cv2.imshow("Original", image)
    # apply edge detection
    edged = cv2.Canny(gray, 10, 250)
    # show the output edge detection image
    cv2.imshow("Edge without blur", edged)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return edged


def apply_edge_detection_with_blur(image,):
    # load the image and convert it to grayscale
    image = cv2.imread(args["image"])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # show the original image
    cv2.imshow("Original", image)
    # apply edge detection
    if method == 'blur':
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 30, 150)
    elif method == 'median':
        blurred = cv2.medianBlur(gray, 5)
        edged = cv2.Canny(blurred, 30, 150)
    elif method == 'bilateral':
        blurred = cv2.bilateralFilter(gray, 9, 75, 75)
        edged = cv2.Canny(blurred, 30, 150)
    else:
        print('Method not supported')
    

    cv2.imshow("Edge with blur", edged)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return edged


if __name__ == "__main__":
    parse = argparse.ArgumentParser()
    parse.add_argument("-i", "--image", required = True, help = "Path to the image")
    parse.add_argument("-m", "--method", default='blur', help = "Method to apply edge detection")
    args = vars(parse.parse_args())
    apply_edge_detection(args)
    apply_edge_detection_with_blur(args)
        
