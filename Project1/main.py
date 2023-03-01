from pupil_apriltags import Detector
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import tagmap

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

tag_size = 0.16

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))

def to_world_coords(tag_id, posc, rotc):
    if tag_id in tagmap.tags_top:
        Rwa = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    elif tag_id in tagmap.tags_left:
        Rwa = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    elif tag_id in tagmap.tags_right:
        Rwa = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    else:     # in tagmap.tags_bottom
        Rwa = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

    tag_pos = tagmap.tagmap[tag_id]
    twa = np.array([[tag_pos[0]], [tag_pos[1]], 0])

    Rca = rotc
    tca = posc[0]
    pa = np.matrix.transpose([0, 0, 0, 1])
	
    Tca = np.zeros((4,4))
    Tca[0:3,0:3] = Rca
    Tca[0:3,3] = tca
    Tca[3,3] = 1
	
    Tac = np.linalg.inv(Tca)

    Twa = np.zeros((4,4))
    Twa[0:3,0:3] = Rwa
    Twa[0:3,3] = twa
    Twa[3,3] = 1

    return Twa@Tac@pa
    

if __name__ == "__main__":
    """
    Program flow:
        startup -> assume roughly on starting point, facing towards 32
        detect 32 and figure out exact location
        calculate path
        move along path?
        ...
    """
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis

    tag_size=0.16 # tag size in meters

    target = np.array([0, 0.5]) # 0.5 m away from tag

    found_32 = False
    pose = None
    rot = None

    # NOT the main control loop - at first we ONLY want to find tag 32
    # this is to determine our precise starting location
    while not found_32:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)

            for res in results:
                # If AprilTag in frame, only 1 result 
                _pose = find_pose_from_tag(K, res)
                _rot, jaco = cv2.Rodrigues(_pose[1], _pose[1])

                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                id = res.tag_id#.decode("utf-8")
                if id == 32:
                    found_32 = True
                    pose = _pose
                    rot = _rot
            if found_32:
                break

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

    current_pos = to_world_coords(32, pose, rot)

    
