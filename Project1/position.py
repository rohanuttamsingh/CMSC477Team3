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
        Rwa = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]])
    elif tag_id in tagmap.tags_right:
        Rwa = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]])
    else:     # in tagmap.tags_bottom
        Rwa = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

    tag_pos = tagmap.tagmap[tag_id]
    twa = np.array([[tag_pos[0]], [tag_pos[1]], [0]])

    Rca = rotc#.T
    tca = -posc[0]
    print(tca)
    # Invert posc because find_pose_from_tag gives translation from april tag
    # to camera, but we care about translation from camera to april tag
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
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis

    tag_size=0.16 # tag size in meters

    K=np.array([[184.752 * 1.7, 0, 320], [0, 184.752 * 1.7, 180], [0, 0, 1]])

    while True:
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
                    if not center_pose or abs(x) < abs(center_pose[0][0]):
                        center_tag, center_pose = tag, pose
            
            if center_tag:
                center_rot, jaco = cv2.Rodrigues(center_pose[1], center_pose[1])

                pts = center_tag.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(center_tag.center.astype(np.int32)), 5, (0, 0, 255), -1)

                current_pos = to_world_coords(center_tag.tag_id, center_pose, center_rot)
                yaw = np.tan(center_pose[1][2] / center_pose[1][0])

                print(f'center_tag.tag_id: {center_tag.tag_id}')
                print(f'center tag position: {tagmap.tagmap[center_tag.tag_id]}')
                print(f'current_pos: {current_pos}')
                print(f'yaw: {yaw}')
            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)
