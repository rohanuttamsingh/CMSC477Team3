from pupil_apriltags import Detector
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import tagmap
import utils
from heading_control import heading_control
from vcontrol import world_velocity
from tagmap import unit

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
    twa = np.array([[tag_pos[0]], [tag_pos[1]], [0]])

    Rca = rotc
    tca = -posc[0]
    # Invert posc because find_pose_from_tag gives translation from april tag
    # to camera, but we can translation from camera to april tag
    pc = np.array([0, 0, 0, 1]).T
	
    Tca = np.zeros((4,4))
    Tca[0:3,0:3] = Rca
    Tca[0:3,3] = tca
    Tca[3,3] = 1
	
    Tac = np.linalg.inv(Tca)

    Twa = np.zeros((4,4))
    Twa[0:3,0:3] = Rwa
    Twa[0:3,3:] = twa
    Twa[3,3] = 1

    return Twa@Tac@pc
    

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

    K=np.array([[184.752 * 1.7, 0, 320], [0, 184.752 * 1.7, 180], [0, 0, 1]])

    path = utils.get_path('Map_stayaway.csv')
    curr_idx = 1

    found_32 = False

    while True:
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

    while True:
        curr = (current_pos[0], current_pos[1])
        next_point = path[curr_idx]
        print('moving to next point')
        prev_vb = None

        while np.linalg.norm(np.array(curr) - np.array(next_point)) > 0.2:
            try:
                img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                gray.astype(np.uint8)

                results = at_detector.detect(gray, estimate_tag_pose=False)
                # Calculate robot current position from april tags
                center_tag, center_pose = None, None
                if len(results) == 1:
                    center_tag = results[0]
                    center_pose = find_pose_from_tag(K, center_tag)
                else:
                    for tag in results:
                        pose = find_pose_from_tag(K, tag)
                        x = pose[0][0]
                        yaw = pose[1][2]
                        if not center_pose or abs(x) < abs(center_pose[0][0]):
                            if yaw <= 0.5 and abs(x) < 0.3: # Thresholds for when estimate is bad
                                center_tag, center_pose = tag, pose
                
                if center_tag:
                    center_rot, jaco = cv2.Rodrigues(center_pose[1], center_pose[1])

                    pts = center_tag.corners.reshape((-1, 1, 2)).astype(np.int32)
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                    cv2.circle(img, tuple(center_tag.center.astype(np.int32)), 5, (0, 0, 255), -1)

                    current_pos = to_world_coords(center_tag.tag_id, center_pose, center_rot)
                    curr = (current_pos[0], current_pos[1])

                    velocity = np.array(next_point) - np.array(curr)
                    x_velocity = velocity[0]
                    y_velocity = velocity[1]
                    # z_velocity = heading_control(np.array([0, 0]), np.array([center_pose[1][0], center_pose[1][1]]))
                    z_velocity = 0
                    vw = np.array([x_velocity, y_velocity, z_velocity])

                    Rbc = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]])
                    # Rbc = np.array([[-1, 0, 0], [0, 0, -1], [0, 1, 0]])
                    if center_tag.tag_id in tagmap.tags_top:
                        Rwa = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
                    elif center_tag.tag_id in tagmap.tags_left:
                        Rwa = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
                    elif center_tag.tag_id in tagmap.tags_right:
                        Rwa = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
                    else:     # in tagmap.tags_bottom
                        Rwa = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
                    Rac = center_rot.T
                    Rwc = Rwa @ Rac
                    Rcw = Rwc.T
                    vb = 0.5 * Rbc @ Rcw @ vw
                    prev_vb = vb

                    print(f'center_tag.tag_id: {center_tag.tag_id}')
                    print(f'current_pos: {current_pos}')
                else:
                    vb = prev_vb
                print(f'next_point: {next_point}')
                print(f'vw: {vw}')
                print(f'vb: {vb}')
                ep_chassis.drive_speed(x=vb[0], y=vb[1], z=0, timeout=0.5)

                cv2.imshow("img", img)
                cv2.waitKey(10)

            except KeyboardInterrupt:
                ep_camera.stop_video_stream()
                ep_robot.close()
                print('Exiting')
                exit(1)

        curr_idx += 1
