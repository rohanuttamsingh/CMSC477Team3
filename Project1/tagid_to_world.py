import numpy as np

def tagid_to_world(tagid, lefttags, righttags, uptags, downtags):
    left_pos = np.where(lefttags == tagid)
    right_pos = np.where(righttags == tagid)
    up_pos = np.where(uptags == tagid)
    down_pos = np.where(downtags == tagid)
    if len(left_pos[0]) > 0:
        return left_pos
    if len(right_pos[0]) > 0:
        return right_pos
    if len(up_pos[0]) > 0:
        return up_pos
    return down_pos
