import json

fn = "intrinsics.json"

class Intrinsics:
    def __init__(self, intrinsics, distortion):
        self.intrinsics = intrinsics
        self.distortion = distortion

    def intrinsics_as_tuple(self):
        fx = self.intrinsics[0][0]
        fy = self.intrinsics[1][1]
        cx = self.intrinsics[0][2]
        cy = self.intrinsics[1][2]
        return (fx,fy,cx,cy)

    def distortion_as_tuple(self):
        return tuple(self.distortion[0])

    def to_dict(self):
        return {
            "intrinsics": self.intrinsics,
            "distortion": self.distortion
        }

    @staticmethod
    def from_dict(d):
        return Intrinsics(d['intrinsics'], d['distortion'])

    def to_str(self):
        return json.dumps(self.to_dict())

    @staticmethod
    def from_str(s):
        return Intrinsics.from_dict(json.loads(s))

    def __str__(self):
        return self.to_str()

def _load():
    o = json.load(open(fn))
    for k in o.keys():
        o[k] = Intrinsics.from_dict(o[k])
    return o

def _dump(o):
    for k in o.keys():
        o[k] = o[k].to_dict()
    json.dump(o, open(fn, 'w'))

def set_by_id(i, val):
    o = _load()
    o[str(i)] = val
    _dump(o)

def get_by_id(i):
    o = _load()
    if i not in o.keys():
        return None
    return o[i]


