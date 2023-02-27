"""

Velocity = Feedback + Feedforward

where

Feedback = K * error
Feedfwd = desired velocity

error = (desired position) - (position)

desired velocity = speed * (vector b/w current point and next)
where speed is a predefined constant


mapping from t to n:

suppose path is a list of points with length N ==> n_i = 0 and n_f = N-1
suppose we want the robot to traverse the path over T seconds ==> t_i = 0 and t_f = T
assume linear mapping (can we?) between time and array index --> then slope is
m = (n_f - n_i) / (t_f - t_i) = [(N-1) - 0] / (T - 0) = (N-1) / T
b = 0 by inspection as function intercepts origin
then mapping function is n(t) = (N-1) * (t/T)


"""

import math
from typing import List, Tuple

"""
t = expected time elapsed
T = total expected runtime for path traversal
pos = point representing current position
path = array of points
"""
def get_velocity(t: float, T: float, pos: Tuple[float, float], path: List[Tuple[float, float]], debug: bool = False):
    n = math.ceil((len(path) - 1) * (t / T))  # represents index in path array of current expected position based on time t passed in, i.e. n = f(t)
    K = 1  # positive constant of control law

    def dvelocity() -> Tuple[float, float]:
        speed = 1  # a positive constant
        currentPos = path[n]
        nextPos = path[n+1]
        norm = math.sqrt(((nextPos[0]-currentPos[0]) ** 2.0) + ((nextPos[1]-currentPos[1]) ** 2.0))
        vel = (speed * (nextPos[0]-currentPos[0]) / norm, speed * (nextPos[1]-currentPos[1]) / norm)
        return vel

    error = (path[n][0] - pos[0], path[n][1] - pos[1])

    feedback = (K * error[0], K * error[1])
    feedfwd = dvelocity()

    velocity = (feedback[0] + feedfwd[0], feedback[1] + feedfwd[1])

    if debug:
        print("n =", n)
        print("path[n] =", path[n])
        print("error =", error)
        print("feedback =", feedback)
        print("feedfwd =", feedfwd)
        print("velocity =", velocity)

    return velocity


if __name__ == "__main__":
    _path = [(1.0, 1.0), (2.0, 1.5), (3.0, 4.0), (3.5, 2.0), (5.0, 0.0)]
    _pos = (1.9, 1.4)
    _t = 1
    _T = 4

    get_velocity(_t, _T, _pos, _path, debug=True)