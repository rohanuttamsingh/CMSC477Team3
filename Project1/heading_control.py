import numpy as np

def heading_control(p_ad_c, p_a_c):
    """
    Params:
        p_ad_c: Desired april tag direction in camera frame coordinates
        P_a_c:  Actual april tag direction in camera frame coordinates
        Both are in R2 because y-coords should be 0
    Returns:
        Velocity that the robot should rotate at
    """
    K = 1
    return -K * np.cross(p_ad_c, p_a_c)
    # Cross product is in R1 because p_ad_c and p_a_c are in R2
