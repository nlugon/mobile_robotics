import cv2
from matplotlib import pyplot as plt
import numpy as np

import torch
import torchvision
import kornia as K


def imshow(input: torch.Tensor, figtitle, edge):
    out = torchvision.utils.make_grid(input, nrow=2, padding=5)
    out_np: np.ndarray = K.utils.tensor_to_image(out)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(out_np)
    ax.set_title(figtitle)
    save_path = "./output/" + figtitle.replace(' ', '_') + '.png'
    fig.savefig(save_path)
    plt.show()
    



if __name__ == '__main__':
    img_bgr: np.ndarray = cv2.imread('src\sample_image.png', cv2.IMREAD_COLOR)
    print(img_bgr.shape)
    x_bgr: torch.Tensor = K.utils.image_to_tensor(img_bgr)  # CxHxWx
    x_bgr = x_bgr[None, ...].float() / 255.

    x_rgb: torch.Tensor = K.color.bgr_to_rgb(x_bgr)
    x_gray = K.color.rgb_to_grayscale(x_rgb)

    #imshow(x_gray, 'Original Image', None)
    x_sobel: torch.Tensor = K.filters.sobel(x_gray)
    # keep only dense edges
    #x_sobel = torch.where(x_sobel > 0.1, x_sobel, torch.zeros_like(x_sobel))
    edges_sobel = np.argwhere(x_sobel[0, 0].numpy())
    imshow(1. - x_sobel, 'Sobel Edge Image',edges_sobel)

    x_laplacian: torch.Tensor = K.filters.canny(x_rgb)[0]
    # keep only dense edges
    #x_laplacian = torch.where(x_laplacian > 0.1, x_laplacian, torch.zeros_like(x_laplacian))
    edges_canny = np.argwhere(x_laplacian[0, 0].numpy())
    edges_canny = np.fliplr(edges_canny)
    print(edges_canny)
    imshow(1. - x_laplacian.clamp(0., 1.), 'Canny Edge Image', edges_canny)
    

