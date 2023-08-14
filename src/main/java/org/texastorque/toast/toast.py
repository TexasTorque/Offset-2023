import cv2, importlib, threading, network, intrinsics, time, sys, argparse, os
from flask import Flask
from flask_cors import CORS
import base64

app = Flask(__name__)
cors = CORS(app, resources={r"/getCameraStream": {"origins": "http://localhost:3000"}})

def load_camera(id_):
    return cv2.VideoCapture(id_)

rtsp_template = "rtsp://10.14.77.77:8554/video<ID>_unicast"

camera_dispatch = {}

def get_stream_id(id_raw):
    if id_raw.startswith('local'):
        return int(id_raw.replace('local', ''))
    elif id_raw.startswith('remote'):
        return rtsp_template.replace('<ID>', id_raw.replace('remote', ''))
    else:
        return None


def init_stream(camera_name, camera_type):
    cam = load_camera(get_stream_id(camera_name))  

    module = importlib.import_module('pipelines.' + camera_type)  

    i_vals = intrinsics.get_by_id(camera_name)

    if i_vals is None:
        print("No intrinsics found for id " + camera_name)
        return None

    net = network.Sender(camera_name)
    net.push_coeffs(i_vals)

    t = threading.Thread(target=module.loop, args=(camera_name, cam, i_vals, net))
    t.start()
    return t

def dispatch():
    threads = []

    # time.sleep(22) #init hardware
    print("Waiting for Network Tables Initialization...")
    time.sleep(3)

    for cam_name, stream_type in network.get_camera_entries().items():
        camera_dispatch[cam_name] = stream_type
        t = init_stream(cam_name, stream_type)
        if t is None: continue
        threads.append(t)

def test():
    threads = []
    
    camera_dispatch['local0'] = 'april-tags'
    t = init_stream('local0', 'april-tags')
    threads.append(t) 
    
@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', 'http://localhost:3000')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    return response

@app.route("/getCameraStream", methods=['GET'])
def base75(): 
    stream = network.gather_img_for('local0')
    return base64.b64encode(stream)

@app.route("/getCameraDispatch", methods=['GET'])
def get_camera_dispatch():
    print(camera_dispatch)
    return camera_dispatch

def handle_arguments():
    parser = argparse.ArgumentParser(description='TOAST')
    parser.add_argument('-l', '--localhost', help='use localhost as the NT server', default=False, action='store_true')
    parser.add_argument('-n', '--ntserver', help='the address for the NT server', type=str, default="10.14.77.2")
    parser.add_argument('-t', '--test', help='run desktop local tests', default=False, action='store_true')
    parser.add_argument('-s', '--server', help='launch webserver', default=False, action='store_true')
    return vars(parser.parse_args())

if __name__ == "__main__":
    args = handle_arguments()
    print(args)
    network.nt_init("localhost" if args['localhost'] else args['ntserver'])

    if args['test']:
        test()
    else:
        dispatch()

    if args['server']:
        app.run(debug=True, threaded=True)
