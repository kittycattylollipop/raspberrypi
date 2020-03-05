import cv2
import numpy as np
from matplotlib import pyplot as plt

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


def RGB_Ratio_Mask(img, ratioImg, rgbTh, ratioTh, viz=False, figsize=[12,6]):
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

#input binary mask (0,1), or boolean mask (True/False)
def maskLabeling(mask, sizeTh):
    num_labels, labels_im = cv2.connectedComponents(np.uint8(mask)*255)
    labCount = 0
    for idx in range(1,num_labels):
        if (labels_im==idx).sum() < sizeTh:
            labels_im[labels_im==idx] = 0
        else: 
            labCount += 1
            labels_im[labels_im==idx] = labCount
            
    return labCount, labels_im


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
    