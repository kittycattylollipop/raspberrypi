import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches as patches

#compute patch stats
def patch_stats(orgpatch, figsize=[12,6]):
    
    plt.rcParams['figure.figsize'] = figsize
    
    patch = orgpatch.reshape(-1,3)
    print(patch.shape)
    print("    patch max: ", patch.max(axis=0))
    print("    patch min: ", patch.min(axis=0))
    r,g,b=0,1,2
    ratio_r_g = patch[:,r]/(patch[:,g]+0.1)
    print("    r/g max: ", ratio_r_g.max())
    print("    r/g min: ", ratio_r_g.min())
    plt.subplot(131,xscale='log'), plt.hist(ratio_r_g,bins=100), plt.title("r/g ratio")  
    ratio_r_b = patch[:,r]/(patch[:,b]+0.1)
    print("    r/b max: ", ratio_r_b.max())
    print("    r/b min: ", ratio_r_b.min())
    plt.subplot(132,xscale='log'), plt.hist(ratio_r_b,bins=100),  plt.title("r/b ratio")
    ratio_g_b = patch[:,g]/(patch[:,b]+0.01)
    print("    g/b max: ", ratio_g_b.max())
    print("    g/b min: ", ratio_g_b.min())
    plt.subplot(133,xscale='log'), plt.hist(ratio_g_b,bins=100), plt.title("g/b ratio")
    plt.show()
    
    ratio_r_g.resize(orgpatch.shape[0:2])
    ratio_r_b.resize(orgpatch.shape[0:2])
    ratio_g_b.resize(orgpatch.shape[0:2])
    return ratio_r_g, ratio_r_b, ratio_g_b


def RGB_Ratio_Mask(img, ratioImg, rgbTh, ratioTh, viz=False, figsize=[24,12]):
    fimg = np.float64(img)
    mask = np.ones(img.shape[0:2], dtype=bool)
    for i in range(img.shape[2]):
        if rgbTh[i]>0:
            mask = mask & (fimg[:,:,i] > rgbTh[i])
        elif rgbTh[i]<0: 
            mask = mask & (fimg[:,:,i] < np.abs(rgbTh[i]))
        
        if ratioTh[i]>0:
            mask = mask & (ratioImg[:,:,i] > ratioTh[i])
        elif ratioTh[i]<0: 
            mask = mask & (ratioImg[:,:,i] < np.abs(ratioTh[i]))                        
    
    if viz:
        plt.rcParams['figure.figsize'] = figsize
        plt.subplot(131),plt.imshow(img)
        plt.subplot(132),plt.imshow(mask,'gray')
        plt.subplot(133), plt.imshow(img * np.dstack((mask, mask, mask)))
        
    return mask


def normRGB(img, show_stats=False):
    sumimg = img.sum(axis=2)+0.01
    norm_rgb = img / np.dstack((sumimg,sumimg,sumimg))
    name=['normR', 'normG', 'normB']
    #compute stats 
    if show_stats:
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
    #num_labels, labels_im = cv2.connectedComponents(np.uint8(mask)*255)
    num_labels, labels_im, stats, centroids = cv2.connectedComponentsWithStats(np.uint8(mask)*255)    
    labCount = 1
    connStats = stats[0,:] #the background stats
    connCent = centroids[0,:]
    for idx in range(1,num_labels):
        if stats[idx,4] < sizeTh:
            labels_im[labels_im==idx] = 0
        else: 
            labCount += 1
            labels_im[labels_im==idx] = labCount
            connStats = np.vstack((connStats,stats[idx,:]))
            connCent = np.vstack((connCent, centroids[idx,:]))            
            
    return labCount, labels_im, connStats, connCent


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
    
def createRectangles(rectXYWH):
    rects = []
    for row in rectXYWH:
        rects.append(patches.Rectangle((row[0],row[1]), row[2], row[3], linewidth = 1, edgecolor='r',fill=False))
    return rects


def plotRectangle(rectXYWH, ax, colorCh='r', lineW=1):
    for row in rectXYWH:
        ax.add_patch(patches.Rectangle((row[0],row[1]), row[2], row[3], linewidth = lineW, edgecolor=colorCh,fill=False))
    

                    