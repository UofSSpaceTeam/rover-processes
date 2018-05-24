import sys

def getnetwork():
    if len(sys.argv) > 1:
        return sys.argv[1]
    else:
        return None
