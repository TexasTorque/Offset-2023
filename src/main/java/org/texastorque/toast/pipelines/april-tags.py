import time
import cv2
import math
import sys
import intrinsics
import pupil_apriltags
from geometry import *

light_blue = (60, 120, 255)
white = (255, 255, 255)
width_px = 8
tag_family = "tag16h5"
tag_size_m = 6 * 2.54 * 1e-2
dec_margin = 15
annotate = True

def load_tag_detector():
    return pupil_apriltags.Detector(families=tag_family, nthreads=1)

def detection_to_json_entry(detection):
    pose = Pose3d.from_matrix(detection.pose_t)
    rot = Rot3d.from_matrix(detection.pose_R)

    return {
        'id': detection.tag_id,
        'hamming': detection.hamming,
        'decision_margin': detection.decision_margin,
        'center': list(detection.center),
        'corners': [list(corner) for corner in detection.corners],
        'euler_rot': list(rot.to_euler()),
        'quat_rot': list(rot.to_quaternion()),
        'translation': pose.to_list(),
        'pose_error': detection.pose_err,
        'homography': [list(h) for h in detection.homography]
    }

def update(frame_r, tag_detector, i_vals, net):
    frame_g = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

    detections = tag_detector.detect(
        img=frame_g, estimate_tag_pose=True,
        camera_params=i_vals.intrinsics_as_tuple(), tag_size=tag_size_m)

    nt_json = {
        'targets': []
    }
    
    frame_b = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
    for detection in detections:
        if detection.hamming == 0 and detection.decision_margin > dec_margin:
            if annotate:
                corners = detection.corners
                corner1 = (int(corners[0][0]), int(corners[0][1]))
                corner2 = (int(corners[1][0]), int(corners[1][1]))
                corner3 = (int(corners[2][0]), int(corners[2][1]))
                corner4 = (int(corners[3][0]), int(corners[3][1]))
                
                cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
                
                cv2.line(frame_b, corner1, corner2, light_blue, width_px)
                cv2.line(frame_b, corner2, corner3, light_blue, width_px)
                cv2.line(frame_b, corner3, corner4, light_blue, width_px)
                cv2.line(frame_b, corner4, corner1, light_blue, width_px)
                cv2.putText(frame_b, str(detection.tag_id), corner4, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)

            nt_json['targets'].append(detection_to_json_entry(detection))

    net.send(nt_json)
    net.store_frame(frame_b)
    return frame_r

# Required function for pipeline
def loop(id_, cam, i_vals, net):
    
    tag_detector = load_tag_detector()

    while True:
        ret, frame_r = cam.read()
        if not ret:
            print("Not working = " + id_)
            break

        update(frame_r, tag_detector, i_vals, net)

    cam.release()
    cv2.destroyAllWindows()
