table = None

# Required for pipeline
def start(table_):
    table = table_

# Required function for pipeline
def loop(id_, cam, i_vals, debug):
    print(id_)