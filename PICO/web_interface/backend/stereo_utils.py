
# utilities .py
import copy
import math
import numpy as np
import cv2

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    plt = None

try:
    import scipy
    import scipy.optimize
except ImportError:
    scipy = None

try:
    import torch
    import torchvision
    import torchvision.transforms.functional as tvtf
    from torchvision.models.detection import MaskRCNN_ResNet50_FPN_Weights, MaskRCNN_ResNet50_FPN_V2_Weights
    HAS_TORCH = True
except ImportError:
    torch = None
    HAS_TORCH = False
    print("Warning: Torch not found. Stereo processing might be limited.")


COLOURS = [] # Default if matplotlib missing
if plt:
    try:
        COLOURS = [
            tuple(int(colour_hex.strip('#')[i:i+2], 16) for i in (0, 2, 4))
            for colour_hex in plt.rcParams['axes.prop_cycle'].by_key()['color']
        ]
    except:
        pass

if not COLOURS:
    COLOURS = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255)]

def preprocess_image(image):
    if not HAS_TORCH: return None
    image = tvtf.to_tensor(image)
    image = image.unsqueeze(dim=0)
    return image

def get_detections(maskrcnn, imgs, score_threshold=0.4): #person, dog, elephan, zebra, giraffe, toilet
    ''' Runs maskrcnn over all frames in vid, storing the detections '''
    if not HAS_TORCH: return [], [], [], []
    
    det = []
    lbls = []
    scores = []
    masks = []
    
    for img in imgs:
        with torch.no_grad():
            result = maskrcnn(preprocess_image(img))[0]
    
        mask = result["scores"] > score_threshold

        boxes = result["boxes"][mask].detach().cpu().numpy()
        if (len(boxes)) > 10:
            boxes = boxes[:10]
        det.append(boxes)
        lbls.append(result["labels"][mask].detach().cpu().numpy())
        scores.append(result["scores"][mask].detach().cpu().numpy())
        masks.append(result["masks"][mask]) 
        
    return det, lbls, scores, masks

def draw_detections(img, det, colours=COLOURS, obj_order = None):
    for i, (tlx, tly, brx, bry) in enumerate(det):
        if obj_order is not None:
            i = obj_order[i]
        i %= len(colours)
        tlx, tly, brx, bry = int(tlx), int(tly), int(brx), int(bry)
        cv2.rectangle(img, (tlx, tly), (brx, bry), color=colours[i], thickness=2)

# ... (omitting remaining plotting helpers that use matplotlib extensively if we only need streaming)
# ... But I will include the cost functions used for tracking

def tlbr_to_center1(boxes):
    points = []
    for tlx, tly, brx, bry in boxes:
        cx = (tlx+brx)/2
        cy = (tly+bry)/2
        points.append([cx, cy])
    return points

def tlbr_to_corner(boxes):
    points = []
    for tlx, tly, brx, bry in boxes:
        cx = (tlx+tlx)/2
        cy = (tly+tly)/2
        points.append((cx, cy))
    return points

def tlbr_to_corner_br(boxes):
    points = []
    for tlx, tly, brx, bry in boxes:
        cx = (brx+brx)/2
        cy = (bry+bry)/2
        points.append((cx, cy))
    return points

def tlbr_to_area(boxes):
    areas = []
    for tlx, tly, brx, bry in boxes:
        cx = (brx-tlx)
        cy = (bry-tly)
        areas.append(abs(cx*cy))
    return areas

def get_horiz_dist_centre(boxes):
    pnts1 = np.array(tlbr_to_center1(boxes[0]))[:,0]
    pnts2 = np.array(tlbr_to_center1(boxes[1]))[:,0]
    return pnts1[:,None] - pnts2[None]

def get_vertic_dist_centre(boxes):
    pnts1 = np.array(tlbr_to_center1(boxes[0]))[:,1]
    pnts2 = np.array(tlbr_to_center1(boxes[1]))[:,1]
    return pnts1[:,None] - pnts2[None]

def get_area_diffs(boxes):
    pnts1 = np.array(tlbr_to_area(boxes[0]))
    pnts2 = np.array(tlbr_to_area(boxes[1]))
    return abs(pnts1[:,None] - pnts2[None])

def get_cost(boxes, lbls = None, sz1 = 400):
    if scipy is None: return np.zeros((1,1))
    
    alpha = sz1; beta  = 10; gamma = 5
    
    #vertical_dist, scale by gamma since can't move up or down
    vert_dist = gamma*abs(get_vertic_dist_centre(boxes))
    
    #horizonatl distance.
    horiz_dist = get_horiz_dist_centre(boxes)
    
    #increase cost if object has moved from right to left.
    horiz_dist[horiz_dist<0] = beta*abs(horiz_dist[horiz_dist<0])
    
    #area of box
    area_diffs = get_area_diffs(boxes)/alpha
    
    cost = np.array([vert_dist,horiz_dist,area_diffs])
    
    cost=cost.sum(axis=0)
    
    #add penalty term for different object classes
    if lbls is not None:
        for i in range(cost.shape[0]):
            for j in range(cost.shape[1]):
                if (lbls[0][i]!=lbls[1][j]):
                    cost[i,j]+=50500
    return cost
    
def get_tracks(cost):
    if scipy is None: return [], []
    return scipy.optimize.linear_sum_assignment(cost)
