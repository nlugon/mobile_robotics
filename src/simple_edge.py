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
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    method = args.method
    # show the original image
    #cv2.imshow("Original", image)
    # apply edge detection
    if method == 'blur':
        #blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # highlight green pixels in the blurred image
        #mask = cv2.inRange(blurred, (0, 200, 0), (255, 255, 255))
        #plt.imshow(mask)
        edged = cv2.Canny(blurred, 30, 150)
    elif method == 'median':
        blurred = cv2.medianBlur(gray, 5)
        edged = cv2.Canny(blurred, 30, 150)
    elif method == 'bilateral':
        blurred = cv2.bilateralFilter(gray, 9, 75, 75)
        edged = cv2.Canny(blurred, 30, 150)
    else:
        print('Method not supported')
    
    # show the output edge detection image in plt
    fig = plt.figure()
    ax1 = fig.add_subplot(121)
    ax1.imshow(image)
    ax1.set_title('Original Image')
    ax2 = fig.add_subplot(122)
    ax2.imshow(edged)
    ax2.set_title(f'Edge Image with {method} method')
    plt.show()
    fig.savefig(f'../output/Edge_Image_with_{method}_method.png')
    # draw the contours on the image
    # show the output cnts image in plt
  
    edges = np.argwhere(edged)
    #edges = np.fliplr(edges) # store them in x,y coordinates instead of row,col indices
    print (edges)
    # scatter plot edges on the image
    fig2 = plt.figure()
    ax = fig2.add_subplot(111)
    ax.imshow(image)
    ax.scatter(edges[:,1], edges[:,0], color='red', s=0.5, marker='.')
    plt.title("original image with edges")
    plt.show()
    fig2.savefig(f'../output/original_image_with_edges_{method}_method.png')



    return edged


if __name__ == "__main__":
    parse = argparse.ArgumentParser()
    parse.add_argument("-i", "--image", required = True, help = "Path to the image")
    parse.add_argument("-m", "--method", default='blur', help = "Method to apply edge detection")
    args = parse.parse_args()
    apply_edge_detection(args)
    apply_edge_detection_with_blur(args)
        
