from scipy.io import loadmat

import matplotlib.pyplot as plt
import matplotlib.image as im
import numpy as np
import cv2 as cv

#plt.matplotlib.rcParams['figure.figsize'] = [12, 6]

def convert_mat_to_contours(mat_file, cylinders_indx=[1,2]):
    print ("> Loading profiles from mat file")
    profile = loadmat(mat_file)
    print ("> mat file profiles (from dict Components): ", profile['Components'].shape)
    # order is outlet, Over, Right, Under and Left
    outlet = profile['Components'][0][0][0]
    cylinders_imgs = []
    cylinders_contours = []


    for ind in cylinders_indx:
        print("Profile in ", ind, " has size ", profile['Components'][0][ind][0].shape)
        tresh, img = cv.threshold(profile['Components'][0][ind][0],0,255,cv.THRESH_BINARY)
        #cv.imshow('profile 0',img)
        #cv.waitKey(0)
        cylinders_imgs.append(img)
        contours, hierarchy = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #print( "hierarchy: ", contours )
        cylinders_contours.append(contours)
    return outlet, cylinders_imgs, cylinders_contours

def show_images_from_mat(mat_file):
    print ("> Loading profiles from mat file")
    profile = loadmat(mat_file)
    print ("> mat file profiles (from dict Components): ", profile['Components'].shape)
    for key,val in profile.items():
        print (key, " => ", len(val))
    outlet = profile['Components'][0][0][0]
    #cylinders_imgs = []
    #cylinders_contours = []


    for ind in range(profile['Components'].shape[1]):
        print("Profile in ", ind, " has size ", profile['Components'][0][ind][0].shape)
        tresh, img = cv.threshold(profile['Components'][0][ind][0],0,255,cv.THRESH_BINARY)
        cv.imwrite('matprofile_'+str(ind)+'.png', img)
        #cv.imshow('profile 0',img)
        #cv.waitKey(0)
        #cylinders_imgs.append(img)
        #contours, hierarchy = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #print( "hierarchy: ", contours )
        #cylinders_contours.append(contours)
    #return outlet, cylinders_imgs, cylinders_contours
