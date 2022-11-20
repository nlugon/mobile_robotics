# edge detection in an image basic of the basic
# import the necessary packages
import numpy as np
import argparse
import cv2
import matplotlib.pyplot as plt
# construct the argument parse and parse the arguments


def apply_edge_detection(args):
    # load the image and convert it to grayscale
    image = cv2.imread(args.image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fig = plt.figure()
    # show the original image and the edge detected image
    #cv2.imshow("Original", image)
    # apply edge detection
    edged = cv2.Canny(gray, 10, 250)
    # show the output edge detection image with plt and fig name
    ax1 = fig.add_subplot(121)
    ax1.imshow(image)
    ax1.set_title('Original Image')
    ax2 = fig.add_subplot(122)
    ax2.imshow(edged)
    ax2.set_title('Edge Image')
    plt.show()

    #cv2.imshow("Edge without blur", edged)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return edged


def apply_edge_detection_with_blur(args):
    # load the image and convert it to grayscale
    image = cv2.imread(args.image)
    #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    method = args.method
    # show the original image
    #cv2.imshow("Original", image)
    # apply edge detection
    if method == 'blur':
        #blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        # highlight green pixels in the blurred image
        #mask = cv2.inRange(blurred, (0, 200, 0), (255, 255, 255))
        #plt.imshow(mask)
    elif method == 'median':
        blurred = cv2.medianBlur(image, 5)
    elif method == 'bilateral':
        blurred = cv2.bilateralFilter(image, 9, 75, 75)
    else:
        print('Method not supported')
    
    # show the output edge detection image in plt
    # apply green color mask and bitwise-and on the image
    # Applying different edge detection algorithms

    filtered_img_hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
    mask1 = cv2.inRange(filtered_img_hsv, (0, 200, 0), (255, 255, 255))
    mask2 = cv2.inRange(filtered_img_hsv, (0, 0, 0), (255, 255, 255))
    # remove black pixels
    #mask = cv2.bitwise_and(mask1, mask2)
    output = cv2.bitwise_and(filtered_img_hsv, filtered_img_hsv, mask=mask1)
    output = cv2.bitwise_and(output, output, mask=mask2)
    # show the output edge detection image with plt and fig name
    # apply green color mask and bitwise-and on the image
 


    fig = plt.figure()
    ax1 = fig.add_subplot(121)
    ax1.imshow(image)
    ax1.set_title('Original Image')
    ax2 = fig.add_subplot(122)
    ax2.imshow(output)
    ax2.set_title('Edge Image')
    plt.show()
    fig.savefig('output/mask.png')

    # trying sobel on the blurred image black and white
    gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    sobelx = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=5)
    sobely = cv2.Sobel(gray,cv2.CV_64F,0,1,ksize=5)
    sobel = cv2.sqrt(cv2.addWeighted(cv2.pow(sobelx, 2.0), 1.0, cv2.pow(sobely, 2.0), 1.0, 0.0))

    plt.figure(figsize=(7,4))
    plt.subplot(2,2,1),plt.imshow(output,cmap = 'gray')
    plt.title('masked'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,2),plt.imshow(sobelx,cmap = 'gray')
    plt.title('SobelX'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,3),plt.imshow(sobely,cmap = 'gray')
    plt.title('SobelY'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,4),plt.imshow(sobel,cmap = 'gray')
    plt.title('Sobel'), plt.xticks([]), plt.yticks([])
    plt.show()
    plt.savefig('output/edge_detection_sobel.png')
    #fig.savefig(f'../output/Edge_Image_with_{method}_method.png')
    # draw the contours on the image
    # show the output cnts image in plt
  
    # get x_ y coordinates of the contours
    # get the bounding box of the contour
    # draw the bounding box on the image
    
    x_y = []

    ret ,thresh = cv2.threshold(gray,127,255,0)
    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        x,y,w,h = cv2.boundingRect(c)
        x_y.append([x,y])
        cv2.rectangle(gray,(x,y),(x+w,y+h),(36,255,12),thickness=15)
    fig_tresh = plt.figure()
    ax_contour = fig_tresh.add_subplot(111)
    ax_contour.imshow(gray)
    ax_contour.set_title('Contours')
    fig_tresh.savefig('output/contours.png')
    return x_y


if __name__ == "__main__":
    parse = argparse.ArgumentParser()
    parse.add_argument("-i", "--image", default= "src/sample_image.png", help = "Path to the image")
    parse.add_argument("-m", "--method", default='bilateral', help = "Method to apply edge detection")
    args = parse.parse_args()
    #apply_edge_detection(args)
    apply_edge_detection_with_blur(args)
        
