import cv2
from darkflow.net.build import TFNet
import numpy as np
import time
import timeit

system_id = "1"

options = {
    'model': './yolo/cfg/tiny-yolo-voc-1c.cfg',
    'load': 1375,
    'threshold': 0.1,
}

tfnet = TFNet(options)
colors = [tuple(255 * np.random.rand(3)) for _ in range(10)]

def find_object(capture):
    ret, frame = capture.read()
    if ret:
        results = tfnet.return_predict(frame)
        midpointX = None
        midpointY = None
        borderbox_width = None
        for color, result in zip(colors, results):
            tl = (result['topleft']['x'], result['topleft']['y'])

            midpointX = (result['topleft']['x'] + result['bottomright']['x'])/2
            midpointY = (result['topleft']['y'] + result['bottomright']['y'])/2

            br = (result['bottomright']['x'], result['bottomright']['y'])

            borderbox_width = round(result['bottomright']['x']) - round(result['topleft']['x'])

            label = result['label']
            confidence = result['confidence']

        return (system_id, midpointX, midpointY, borderbox_width)
    else
        return ("n","n","n","n")
