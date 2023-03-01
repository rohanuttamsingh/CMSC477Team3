from pupil_apriltags import Detector
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera

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

if __name__ == "__main__":
    """
    Program flow:
        startup -> assume roughly on starting point, facing towards 32
        detect 32 and figure out exact location
        calculate path
        ...
    """
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis

    tag_size=0.16 # tag size in meters

    target = np.array([0, 0.5]) # 0.5 m away from tag

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)

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
                rot, jaco = cv2.Rodrigues(center_pose[1], center_pose[1])

                pts = center_tag.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(center_tag.center.astype(np.int32)), 5, (0, 0, 255), -1)
                id = center_tag.tag_id
                print(id)
                t = np.array([center_pose[0][0], center_pose[0][2]])

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

