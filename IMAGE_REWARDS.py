# imports
import numpy as np
import cv2

""" CONSTANTS """
# constant for color range detection of the ball
BALL_COLOR = (120, 100, 100)
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
# resizing the frame so that we can process it faster
DOWNSIZE = 128
TOLERANCE = 20

# for reward function
WEIGHT = 0.25
SCALING = 10
NEGATIVE_REWARD = -50


def compute_center_and_size(image_frame):
    """ Function to extract info from an image
    Takes the image_frame array and sends it into the opencv algorithm.
    Returns the center and size of any COLOR_BALL (defined in the constant) that it
    detects in the image.
    :param image_frame:
    :return: Return is in the format tuple ((x, y), radius)
    Error conditions:
        if no ball is detected, then return (None, 0)
    """

    # preprocessing
    # we won't downsize, since the image is modifyable already
    # image_frame = imutils.resize(image_frame, width=DOWNSIZE)
    dims = image_frame.shape
    hsv = cv2.cvtColor(image_frame, cv2.COLOR_BGR2HSV)

    # mask
    color = np.array(BALL_COLOR)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    # don't need the erosion/dilation b/c sim images are perfect
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)

    # find contours in mask and initialize current center of ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    radius = 0

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        center = (x, y)

        # idk what this next line does...
        # M = cv2.moments(c)
        # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    return center, radius, dims[0:2]


def compute_reward(image_frame, initial_radius):
    """ simple reward computation function
    :param image_frame: one image frame rgb matrix to get reward function for
    :param initial_radius: the initial radius of the first frame
    :return:
    """
    (center, radius, dims) = compute_center_and_size(image_frame)
    img_center = np.array(dims)/2

    if radius is 0 or center is None:
        return NEGATIVE_REWARD

    # super simple reward function = radius - weight * (abs(xdiff) + abs(ydiff))
    delta_rad = abs(radius - initial_radius)
    radius = radius - delta_rad

    diff = img_center - np.array(center)
    diff = abs(diff)
    reward = SCALING * (radius + WEIGHT*(sum(diff)))

    return reward
