import cv2 as cv
import cv2
import numpy as np
from time import time
import logging
import math
import matplotlib as plt

class LaneFollower:
    def get_steering_angle(self, frame, lane_lines):

        height, width, _ = frame.shape
        
        x_offset, y_offset = 0, int(height / 2)  # Default offsets

        if len(lane_lines) == 2:
            # Extract lane line coordinates
            left_x1, left_y1, left_x2, left_y2 = lane_lines[0][0]
            right_x1, right_y1, right_x2, right_y2 = lane_lines[1][0]

            # Calculate slopes in radians
            slope_l = math.atan2(left_y2 - left_y1, left_x2 - left_x1)
            slope_r = math.atan2(right_y2 - right_y1, right_x2 - right_x1)
            slope_ldeg = int(slope_l * 180.0 / math.pi)
            steering_angle_left = slope_ldeg
            slope_rdeg = int(slope_r * 180.0 / math.pi)
            steering_angle_right = slope_rdeg
            # Determine offset based on slopes
            if left_x2>right_x2:
                if abs(steering_angle_left)<= abs(steering_angle_right):
                    x_offset = left_x2 - left_x1
                    y_offset = int(height/2)
                elif abs(steering_angle_right) > abs(steering_angle_right):
                    x_offset = right_x2 -right_x1
                    y_offset = int(height/2)
            else:
                mid = int(width/2)
                x_offset = (left_x2 + right_x2) / 2 - mid
                y_offset = int(height / 2)

        elif len(lane_lines) == 1:
            # Single lane line detected
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)

        elif len(lane_lines) == 0:
            # No lane lines detected
            x_offset = 0
            y_offset = int(height / 2)


        # Calculate the steering angle
        alfa = 0.6
        angle_to_mid_radian = alfa * getattr(self, 'angle', 0) + (1 - alfa) * math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        # Final steering angle
        steering_angle = angle_to_mid_deg + 90
        self.angle = angle_to_mid_radian  # Save angle for smoothing

        return steering_angle


    def display_heading_line(self, frame, steering_angle, line_color=(0, 255,0), line_width=5):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 1.75)
        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image

def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    `initial_img` should be the image before any processing.
    The result image is computed as follows:
    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv.addWeighted(initial_img, α, img, β, λ)


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv.HoughLinesP(cropped_edges, rho, angle, min_threshold,np.array([]), minLineLength=20,maxLineGap=200)  #maxLineGap=15

    return line_segments


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # Avoid division by zero
    if slope == 0:
        logging.error("Slope is zero, cannot compute x coordinates.")
        return [[0, y1, 0, y2]]

    # Bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def average_slope_intercept(frame, line_segments):
    """
    Combine line segments into one or two lane lines.
    """
    lane_lines = []
    if line_segments is None or len(line_segments) == 0:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('Skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    # Compute the averages only if the lists are not empty
    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)
        lane_lines.append(make_points(frame, left_fit_average))
    else:
        logging.info("No left lane line detected.")

    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(make_points(frame, right_fit_average))
    else:
        logging.info("No right lane line detected.")

    logging.debug('Lane lines: %s' % lane_lines)
    return lane_lines


def display_lines(frame, lines, line_color=(0, 255, 255), line_width=20):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def region_of_interest(edges):
    #height, width = edges.shape
    height = edges.shape[0]
    width = edges.shape[1]
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)
    """polygon = np.array([[
        (0, height ),
        (width, height ),
        (width, 130),
        (0, 130),
    ]], np.int32)"""
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv.bitwise_and(edges, mask)
    return cropped_edges
def normal(x):
    max_angle = 180  # Max range for steering
    mid_angle = 90  # Straight angle
    normalized_steering = (x - mid_angle) / (max_angle - mid_angle)
    return normalized_steering


# define a range of black color in HSV
lower_black = np.array([0, 0, 0])
upper_black = np.array([227, 100, 70])

# Rectangular Kernel
rectKernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))


if __name__ == '__main__':
    print('Started')
    print("Beginning Transmitting to channel: Happy_Robots")
    now = time()
    lane_follower = LaneFollower()

    # commencing subtraction
    cap = cv.VideoCapture("bosch_test.mp4")
    while cap.isOpened():
        try:
            _, frame = cap.read()

            # apply some gaussian blur to the image
            kernel_size = (3, 3)
            gauss_image = cv.GaussianBlur(frame, kernel_size, 0)
            # gauss_image =  cv.bilateralFilter(frame, 9, 75, 75)

            # here we convert to the HSV colorspace
            hsv_image = cv.cvtColor(gauss_image, cv.COLOR_BGR2HSV)

            # apply color threshold to the HSV image to get only black colors
            thres_1 = cv.inRange(hsv_image, lower_black, upper_black)

            # dilate the threshold image
            thresh = cv.dilate(thres_1, rectKernel, iterations=1)
            
            # apply canny edge detection
            low_threshold = 200
            high_threshold = 400
            canny_edges = cv.Canny(gauss_image, low_threshold, high_threshold)
            # get a region of interest
            roi_image = region_of_interest(canny_edges)

            line_segments = detect_line_segments(roi_image)
            lane_lines = average_slope_intercept(frame, line_segments)
            # overlay the line image on the main frame
            line_image = display_lines(frame, lane_lines)
            steering = lane_follower.get_steering_angle(frame, lane_lines)
            normal_values = normal(steering)
            heading_image1 =lane_follower.display_heading_line( frame, steering) 
            

            # display both the current frame and the fg masks
            cv.imshow('Frame', frame)
            cv.imshow('New Image', roi_image)
            cv.imshow('Line Image', line_image)
            cv.imshow('Heading Image', heading_image1)
            print('Real_Steering',normal_values )
            #print('Number of Lane Lines',len(lane_lines))
        

            keyboard = cv.waitKey(30)
            if keyboard == ord('q') or keyboard == 27:
                break
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error during processing: {e}")
            break

    # cleanup
    cap.release()
    cv.destroyAllWindows()

    print('Stopped')