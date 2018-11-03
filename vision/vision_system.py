import fiducial.tracker
import yolo.tracker
import cv2
import numpy as np

capture = None
context = None
socket = None
width = None
height = None

#find_buoy output gives the x and y directionality relative to the camera axis. Should be used as a nudge system.
def find_buoy()
    try:
        return yolo.tracker.find_object(capture)
    except Exception as e:
        raise Exception("Error encountered while attempting to find buoy visually: {}".format(e))

def buoy_nudge_directions(find_buoy_results)
    if find_buoy_results[0] == "n":
        return ("n", "n")
    try:
        directions = []
        x_midpoint = np.floor(find_buoy_results[1])
        y_midpoint = np.floor(find_buoy_results[2])
        borderbox_width = np.floor(find_buoy_results[3])
        if x_midpoint > (width/2):
            directions.append(1 * (borderbox_width/width))
        else
            directions.append(-1 * (borderbox_width/width))
        if y_midpoint > (height/2)
            directions.append(-1 * (borderbox_width/height))
        else
            directions.append(1 * (borderbox_width/height))
        return directions
    except Exception as e:
        raise Exception("Error in finding directionality of buoy: {}".format(e))

#find_markers output gives the x, y, and z direction vectors that translate from camera to buoy markers.
def find_markers()
    try:
        return fiducial.tracker.find_object(capture)
    except Exception as e:
        raise Exception("Error encountered while attempting to find markers visually: {}".format(e))

try:
    capture = cv2.VideoCapture(0)
    width = np.floor(capture.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
    height = np.floor(capture.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
except Exception as e:
     raise Exception("VideoCapture failed: {}".format(e))

try:
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")
except Exception as e:
    raise Exception("ZMQ binding failure: {}".format(e))
