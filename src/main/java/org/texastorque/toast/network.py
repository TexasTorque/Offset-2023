from networktables import NetworkTables as nt
import json, time, cv2
from toast import app

def nt_init(server):
    nt.initialize(server=server)

table = nt.getTable("toast")

stored_frames = {}

class Sender:
    def __init__(self, id_):
        self.id_ = id_
    
    def send(self, o):
        o['timestamp'] = time.time()
        table.putString(str(self.id_) + "_data", json.dumps(o))

    def store_frame(self, frame):
        stored_frames[self.id_] = frame
        
    def get_camera_type(self):
        return table.getString(str(self.id_) + "_type")

    def push_coeffs(self, i_vals):
        intr = [i for r in i_vals.intrinsics for i in r]
        table.putNumberArray(str(self.id_) + "_intr", intr)
        dist = i_vals.distortion[0]
        table.putNumberArray(str(self.id_) + "_dist", dist)

def gather_img_for(id_):
    while True:
        time.sleep(0.01)
        if id_ not in stored_frames.keys():
            return None
        _, jpg = cv2.imencode('.jpg', stored_frames[id_])
        return jpg.tobytes()


def get_camera_entries():
    entries = {}
    for entry in table.getKeys():
        if entry.startswith('local') or entry.startswith('remote'):
            parts = entry.split('_')
            if parts[-1] == 'type':
                name = parts[0]
                type_ = table.getString(entry, None)
                if name is not None or name == 'none':
                    entries[name] = type_
                else:
                    pass # print invalid type
    return entries

