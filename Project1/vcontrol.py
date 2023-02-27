from pupil_apriltags import Detector
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera

at_detector = Detector(
	families = "tag36h11",
	nthreads = 1,
	quad_decimate = 1.0,
	quad_sigma = 0.0,
	refine_edges = 1,
	decode_sharpening = 0.25,
	debug = 0
)

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


def world_velocity(posc,rotc,posw,rotw,vw):
	Rca = rotc
	Rac = np.linalg(Rca)
	tca = posc[0]
	pa = np.matrix.transpose([0, 0, 0, 1])
	
	Tca = np.zeros((4,4))
	Tca[0:3,0:3] = Rca
	Tca[0:3,3] = tca
	Tca[3,3] = 1
	
	Tac = np.linalg.inv(Tca)
	Rwa = rotw
	twa = posw[0]
	
	Twa = np.zeros((4,4))
	Twa[0:3,0:3] = Rwa
	Twa[0:3,3] = twa
	Twa[3,3] = 1
	  
	pw = Twa@pa
	Rwc = Rwa@Rac
	pc = Twa@Tac@np.matrix.transpose([0, 0, 0, 1])
	
	vb = Rac@(Rwc.T)@vw
	return vb
