from tensorflow.lite.python.interpreter import Interpreter
import os
import cv2
import numpy as np
import importlib.util

table = None

MODEL_NAME = 'sw-uc-model'
GRAPH_NAME = 'detect.tflite'
LABELMAP_NAME = 'labelmap.txt'
min_conf_threshold = 0.5
imW, imH = 1280, 720

CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)
PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

pkg = importlib.util.find_spec('tflite_runtime')

with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

interpreter = Interpreter(model_path=PATH_TO_CKPT)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

outname = output_details[0]['name']
boxes_idx, classes_idx, scores_idx = 1, 3, 0

def detection_to_json_entry(object_name, center_x, score):
    return {
        'name': object_name,
        'center_x': center_x,
        'confidence': score,
    }
    



def update(frame, net):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))
    input_data = np.expand_dims(frame_resized, axis=0)

    input_data = (np.float32(input_data) - input_mean) / input_std

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[
        0]
    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[
        0]
    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[
        0]
    
    nt_json = {
        'cones': []
    }
    
    for i in range(len(scores)):
        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

            ymin = int(max(1, (boxes[i][0] * imH)))
            xmin = int(max(1, (boxes[i][1] * imW)))
            ymax = int(min(imH, (boxes[i][2] * imH)))
            xmax = int(min(imW, (boxes[i][3] * imW)))

            center_x = int((xmin + xmax) / 2)
            center_y = int((ymin + ymax) / 2)

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)

            object_name = labels[int(classes[i])]
            label = '%s: %d%%' % (object_name, int(
                scores[i]*100))
            labelSize, baseLine = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            label_ymin = max(ymin, labelSize[1] + 10)
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (
                xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED)
            cv2.putText(frame, label, (xmin, label_ymin-7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)  # Draw label text

            center_label = 'Center X: {}'.format(center_x)
            cv2.putText(frame, center_label, (center_x, center_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
    
            nt_json['cones'].append(detection_to_json_entry(object_name, center_x, scores[i]))

    net.send(nt_json)
    net.store_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    return frame

# Required function for pipeline
def loop(id_, cam, i_vals, net):
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Not working = " + id_)
            break

        update(frame, net)

    cam.release()
    cv2.destroyAllWindows()
