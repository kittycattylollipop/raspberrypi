import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches as patches

# compute the ratio of RGB input
# output channels: R/G, R/B, G/B
def rgbRatioImage(image):
    r, g, b = 0, 1, 2
    ratioImg = np.zeros_like(image)
    ratioImg[:, :, 0] = image[:, :, r] / (image[:, :, g] + 0.1)
    ratioImg[:, :, 1] = image[:, :, r] / (image[:, :, b] + 0.1)
    ratioImg[:, :, 2] = image[:, :, g] / (image[:, :, b] + 0.1)

    return ratioImg

def rgbNormImage(img, show_stats=False):
    sumimg = img.sum(axis=2)
    norm_rgb = img / np.dstack((sumimg,sumimg,sumimg))
    #compute stats 
    if show_stats:
        name = ['normR', 'normG', 'normB']
        print(norm_rgb.shape)
        print("    norm_rgb max: ", norm_rgb.max(axis=0).max(axis=0))
        print("    norm_rgb min: ", norm_rgb.min(axis=0).min(axis=0))
        plt.rcParams['figure.figsize'] = [12,6]
        for i in range(norm_rgb.shape[2]):            
            plt.subplot('1'+str(norm_rgb.shape[2])+str(i+1))
            hist = plt.hist(norm_rgb[:,:,i].flat,bins=100)
            plt.title(name[i])
        plt.show()
    return norm_rgb


def img_mask(img, th):
    fimg = np.float32(img)
    mask = np.ones(img.shape[0:-1], dtype=bool)
    for i in range(img.shape[2]):
        if th[i]>0:
            mask = mask & (fimg[:,:,i] > th[i])
        elif th[i]<0:
            mask = mask & (fimg[:,:,i] < np.abs(th[i]))
    return mask


def img_in_range(img, low, high):
    mask = img_mask(img, low) + img_mask(img, -high)
    return mask



def visMask(img, mask, figsize=[24,12]):
    plt.rcParams['figure.figsize'] = figsize
    plt.subplot(131),plt.imshow(img)
    plt.subplot(132),plt.imshow(mask,'gray')
    plt.subplot(133), plt.imshow(img * np.dstack((mask, mask, mask)))  
    plt.show()


#input binary mask (0,1), or boolean mask (True/False)
def maskLabeling(mask, sizeTh):
    # https://stackoverflow.com/questions/35854197/how-to-use-opencvs-connected-components-with-stats-in-python
    # stats[label,COLUMN] include: left, top, width, height, area
    # centroid include: x, y
    num_labels, labels_im, stats, centroids = cv2.connectedComponentsWithStats(np.uint8(mask)*255)    
    labCount = 0
    connStats = np.empty((0, len(stats[0, :])))
    connCent = np.empty((0, len(centroids[0, :])))

    for idx in range(1, num_labels):
        if stats[idx, 4] < sizeTh:
            labels_im[labels_im == idx] = 0
        else: 
            labCount += 1
            labels_im[labels_im == idx] = labCount
            connStats = np.vstack((connStats,stats[idx,:]))
            connCent = np.vstack((connCent, centroids[idx,:]))            
            
    return labCount, labels_im, connStats, connCent


#############################################
#  Visualization
##############################################
#show image map with different color 
def imshow_components(labels):
    # Map component labels to hue val
    label_hue = np.uint8(179*labels/np.max(labels))
    blank_ch = 255*np.ones_like(label_hue)
    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

    # cvt to BGR for display
    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2RGB)

    # set bg label to black
    labeled_img[label_hue==0] = 0

    plt.subplot(111),plt.imshow(labeled_img)

def plotRectangle(rectXYWH, ax, colorCh='r', lineW=1):
    for row in rectXYWH:
        ax.add_patch(patches.Rectangle((row[0],row[1]), row[2], row[3], linewidth = lineW, edgecolor=colorCh,fill=False))
