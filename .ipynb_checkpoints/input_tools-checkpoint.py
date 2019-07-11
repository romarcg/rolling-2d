from scipy.io import loadmat

import matplotlib.pyplot as plt
import matplotlib.image as im
import numpy as np
import cv2 as cv

plt.matplotlib.rcParams['figure.figsize'] = [12, 6]

def convert_mat_to_contours(mat_file, cylinders_indx=[1,2]):
    profile = loadmat(mat_file)
    outlet = profile['Components'][0][0][0]
    cylinders_imgs = []
    cylinders_contours = []
    
    for ind in cylinders_indx:
        tresh, img = cv.threshold(profile['Components'][0][ind][0],0,255,cv.THRESH_BINARY)
        cylinders_imgs.append(img)
        im2, contours, hierarchy = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cylinders_contours.append(contours)
    return outlet, cylinders_imgs, cylinders_contours