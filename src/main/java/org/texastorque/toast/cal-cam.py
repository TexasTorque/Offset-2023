import time, cv2, math, os, sys, glob, numpy as np, intrinsics

complex_camera_read = False
checkerboard = (6, 9)

criteria = (cv2.TERM_CRITERIA_EPS +
            cv2.TERM_CRITERIA_MAX_ITER, 25, 0.001)

#points2d = []
#points3d = []

def img_dir(i):
    s = 'cal-imgs/' + str(i)
    if not os.path.exists(s):
        os.makedirs(s)
    return s

def load_camera(id):
    if complex_camera_read:
        s = "v4l2src device=/dev/video" + str(id) + " ! video/x-raw, width=640, height=480 ! videoconvert ! video/x-raw,format=BGR ! appsink"
        return cv2.VideoCapture(s)

    return cv2.VideoCapture(id)


def update_live(frame_r):
 
    frame_g = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
    
    frame_s = cv2.resize(frame_r, (4*50, 3*50), interpolation=cv2.INTER_AREA)

    ret, _ = cv2.findChessboardCorners(
                frame_s, checkerboard,
                cv2.CALIB_CB_ADAPTIVE_THRESH
                + cv2.CALIB_CB_FAST_CHECK +
                cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret != True:
        return frame_r

    ret, corners_3d = cv2.findChessboardCorners(
            frame_g, checkerboard,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK +
            cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret == True:
        corners_2d = cv2.cornerSubPix(
                frame_g, corners_3d, (11, 11), (-1, -1), criteria)

        frame_r = cv2.drawChessboardCorners(frame_r,
                    checkerboard, corners_2d, ret)

    return frame_r

def loop(camera, id_):
    
    print("Press 'p' to take picture")
    pic_id = 0

    while True:
        ret, frame_r = camera.read()
        if not ret:
            break

        frame_a = update_live(frame_r.copy())

        cv2.imshow("FEED", frame_a)

        if cv2.waitKey(1) == ord('p'):
            pic_id += 1
            fn = img_dir(id_) + "/cal-" + str(pic_id) + ".jpg"
            print("Image saves as " + fn)
            cv2.imwrite(fn, frame_r)

        if cv2.waitKey(1) == ord('q'):
            break

    camera.release()


    cv2.destroyAllWindows()


def update_calc(frame_r, points2d):
 
    frame_g = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

    ret, corners_3d = cv2.findChessboardCorners(
            frame_g, checkerboard,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK +
            cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret == True:
        corners_2d = cv2.cornerSubPix(
                frame_g, corners_3d, (11, 11), (-1, -1), criteria)

        points2d.append(corners_2d)

    return frame_g, points2d



def calculate(id_):
    frame_r = None
    frame_g = None
    print("Loading images...\n")
    points2d = []

    object_p3d = np.zeros((1, checkerboard[0] 
                      * checkerboard[1], 
                      3), np.float32)
    object_p3d[0, :, :2] = np.mgrid[0:checkerboard[0],
                               0:checkerboard[1]].T.reshape(-1, 2)

    for fn in glob.glob(img_dir(id_) + '/*.jpg'):
        print("Loading image " + fn)
        frame_r = cv2.imread(fn)
        frame_g, points2d = update_calc(frame_r, points2d)

    points3d = [object_p3d for _ in points2d]

    print("\nCalculating intrinsic matrix...")

    h, w = frame_r.shape[:2]

    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
            points3d, points2d, frame_g.shape[::-1], None, None)

    #unit refinement? https://stackoverflow.com/questions/37310210/camera-calibration-with-opencv-how-to-adjust-chessboard-square-size

    print("\nResulting matrix:")
    print(matrix)

    fx = matrix[0][0]
    fy = matrix[1][1]
    cx = matrix[0][2]
    cy = matrix[1][2]

    i_vals = intrinsics.Intrinsics(matrix.tolist(), distortion.tolist())

    print("\nIntrinsic values:")
    print(i_vals.intrinsics_as_tuple())
    print("\nDistortion values:")
    print(i_vals.distortion_as_tuple())

    return i_vals

def help():
    return """cal-cam.py <id> <command> <local/remote>

Commands:
    pics    take pictures for processing
    calc    calculate intrinsic camera properties and write to the database
    reset   delete calibration images for specified camera
"""

rtsp_template = "rtsp://10.14.77.77:8554/video<ID>_unicast"

def process_id(id_arg, local_arg):
    if local_arg == 'local':
        return int(id_arg), 'local' + id_arg
    elif local_arg == 'remote':
        return rtsp_template.replace('<ID>', id_arg), 'remote' + id_arg
    else:
        print("Invalid remote/local argument!\n\n" + help())
        exit(1)
        return None, None

def main(args):
    if len(args) <= 3:
        print("Invalid arguments.\n" + help())
        return

    cam_id, nominal_id = process_id(args[1], args[3])
    cmd = args[2]
    i_dir = img_dir(nominal_id)

    if cmd == 'reset':
        os.system('rm -rf ' + i_dir)

    elif cmd == 'pics':
        #if os.path.exists(img_dir):
        #    print("Calibration images already exist")
        #    return
        if not os.path.exists(i_dir):
            os.makedirs(i_dir)
        camera = load_camera(cam_id)
        loop(camera, nominal_id)

    elif cmd == 'calc':
        if not os.path.exists(i_dir):
            print("Calibration images dont exists, please take some")
            return
        i_vals = calculate(nominal_id)

        intrinsics.set_by_id(nominal_id, i_vals)

if __name__ == '__main__':
    main(sys.argv)
