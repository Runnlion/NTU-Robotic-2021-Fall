import numpy as np
import cv2 as cv


def object_filtering(img):
    """Binarize the RGB image

    @Params:
        img: RGB image

    @Returns:
        binary image
    """
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 200, 255, cv.THRESH_BINARY)

    # Noise removal
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel, iterations=1)
    return thresh


def calc_area_desciptors(cnt):
    """Calculate the centroid and the principal angle

    @Params:
        cnt: contour of single image

    @Returns:
        centroid 
        principal angle
    """
    m = cv.moments(cnt)
    centroid = np.array([m['m10']/m['m00'], m['m01']/m['m00']])
    pa = np.arctan2(2*m['mu11'], m['mu02']-m['mu20']) / 2 * 180 / np.pi
    return centroid, pa


def find_largest_object(img, mask, min_criteria, show=False, line_length=100):
    """Find the largest object outside the masked region
    @Params:
        img: RGB raw image
        mask: the mask to exclude the searching

    @Results:
        poses: list of position and angle pair
        [[pos, pa], [pos, pa]...]
        pa: principal angle
        cnt: the largest contour (list of points)
    """
    thresh = object_filtering(img)

    # retval, labels, stats, centroids = cv.connectedComponentsWithStats(
    #    mask, connectivity=4)
    # largest_idx = np.argmax(stats[4])  # index 4 for area
    #obj = (labels == largest_idx).astype(np.uint8)

    # Mask out the base
    thresh = cv.bitwise_and(thresh, cv.bitwise_not(mask))

    poses, cnts = get_marker_pos(thresh, min_criteria)

    centroid, pa = poses[0]
    if show:
        pt2 = centroid + \
            np.array([np.cos(pa), np.sin(pa)], dtype=int) * line_length
        cv.arrowedLine(img, centroid, pt2, (0, 255, 255), 2)
    return centroid, pa, cnts[0]


def get_marker_pos(thresh, min_criteria=100, show=False):
    """Find the centroid and principal angle of the objects in the 
    given image. The result will be sorted with the area (from larger
    one to smaller one)

    @Params:
        thresh: binarized image

    @Results:
        poses: list of position and angle pair
        [[pos, pa], [pos, pa]...]
    """
    # cnts = cv.findContours(thresh, cv.RETR_EXTERNAL,
    #                        cv.CHAIN_APPROX_SIMPLE)
    _, cnts, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Sort the result with area size
    cnts = sorted(cnts, key=cv.contourArea)

    pose = []
    for cnt in cnts[:6]:
        if cv.contourArea(cnt) > min_criteria:
            centroid, pa = calc_area_desciptors(cnt)
            pose.append([centroid, pa])
            if show:
                cv.circle(thresh, centroid, 5, (0, 0, 255), 2)
    if show:
        cv.drawContours(thresh, cnts, -1, (255, 0, 0), -1)
        cv.imshow('thresh', thresh)
        cv.waitKey(0)
        cv.destroyAllWindows()
    return np.array(pose), cnts
