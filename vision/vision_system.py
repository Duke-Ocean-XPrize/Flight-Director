import fiducial.tracker
import yolo.tracker
import cv2
import sys
import zmq
import numpy as np

capture = None
context = None
socket = None
width = None
height = None

#find_buoy output gives the x and y directionality relative to the camera axis. Should be used as a nudge system.
def find_buoy()
    try:
        yolo_ret = yolo.tracker.find_object(capture) #(system_id, x_midpoint, y_midpoint, borderbox_width)
        direction_tuple = buoy_nudge_directions(yolo_ret) #(x_dir, y_dir)
        return (yolo_ret[0], direction_tuple[0], direction_tuple[1], yolo_ret[3])
    except Exception as e:
        raise Exception("Error encountered while attempting to find buoy visually: {}".format(e))

def buoy_nudge_directions(find_buoy_results) #Takes the midpoint found by the vision and finds the directionality associated with it.
    if find_buoy_results[0] == "n":
        return ("n", "n")
    try:
        directions = []
        x_midpoint = np.floor(find_buoy_results[1])
        y_midpoint = np.floor(find_buoy_results[2])
        borderbox_width = np.floor(find_buoy_results[3])
        if x_midpoint > (width/2):
            directions.append(1)
        else
            directions.append(-1)
        if y_midpoint > (height/2)
            directions.append(-1)
        else
            directions.append(1)
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

def run()
    try:
        while True:
            buoy_res = find_buoy()
            socket.send("{}/{}/{}/{}".format(buoy_res[0], buoy_res[1], buoy_res[2], buoy_res[3]))
            if buoy_res[3] > 120:
                break
        x_mov_avg = []
        y_mov_avg = []
        z_mov_avg = []
        while True:
            marker_res = find_markers()
            x_mov_avg.insert(0, marker_res[1])
            y_mov_avg.insert(0, marker_res[2])
            z_mov_avg.insert(0, marker_res[3])
            if len(x_mov_avg) > 100:
                x_mov_avg.pop()
                y_mov_avg.pop()
                z_mov_avg.pop()
            socket.send("{}/{}/{}/{}".format(marker_res[0], str(sum(x_mov_avg)/len(x_mov_avg))[:6], str(sum(y_mov_avg)/len(y_mov_avg))[:6], str(sum(z_mov_avg)/len(z_mov_avg))[:6]))
            if marker_res[3] < 0.75:
                break
            sys.exit()
    except Exception as e:
        raise Exception("Vision_System movement protocol failed: {}".format(e))
