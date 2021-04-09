'''
Autonomous Drone Exchange 2021
Spencer Buebel, Fran Romano

Author: stbuebel@gmail.com
Date: 4/2021

This script reads images from the Rasp Pi camera,
locates an apriltag, performing localization based
on its position, and commands the velocity of the
connected pixhawk flight controller to navigate to
the tag.
'''

import cv2
import apriltag
import time
import math
import threading
import sys

from picamera import PiCamera
from picamera.array import PiRGBArray

from pymavlink import mavutil

# connect to the mavlink to get pitch and roll
# gonna have to send velocity commands too
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
master.mav.request_data_stream_send(1, 30, mavutil.mavlink.MAV_DATA_STREAM_ALL, 30, 1)

# parameters
TAG_SIZE = 0.1476 # meters
MIDPOINT = (320, 240) # center of the image
FOV_H = 62.2 # degrees, horizontal
FOV_V = 48.8 # degrees, vertical
__P__ = 2.5
__I__ = 0.0
__D__ = 5
SETPOINT = (0, 0) # relative position navigation goal (from apriltag)

# these are global - shared between threads
roll = 0.0
pitch = 0.0

# global velocity command that we send
vx = 0.0
vy = 0.0
vz = 0.0

# this is important because we need to give control back to user
TAG_IN_VIEW = False

# quick helper function
def distance(a, b):
	inner = (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1])
	return math.sqrt(inner)

####################################################################
# update_state()
#	- update the pitch and roll in degrees based on the FC data
####################################################################
def update_state():
	global roll, pitch
	
	while True:
		msg = master.recv_match()
		
		if not msg:
			continue
		
		if msg.get_type() == 'ATTITUDE':
			roll = msg.to_dict()['roll']*180/3.1415
			pitch = msg.to_dict()['pitch']*180/3.1415
			print(msg.to_dict())

####################################################################
# send_pid_velocities()
#	- send the global velocity to the pixhawk
####################################################################
def send_pid_velocities():
	global vx, vy, vz
	
	while True:
		# basically just continually write the desired velocity if tag is seen
		if TAG_IN_VIEW:
			# how to change flight mode:
			mode_id = master.mode_mapping()['GUIDED']
			master.mav.set_mode_send(0, # target sys
				mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
				mode_id)

			master.mav.set_position_target_local_ned_send(0, 0, 0,
				mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
				0b0000111111000111, 0, 0, 0,
				vx, vy, -vz, 0, 0, 0, 0, 0)

			time.sleep(0.05)

			print("[GUIDED] - sent v:", (vx, vy, -1*vz))

		else:
			# goto position
			mode_id = master.mode_mapping()['POSITION']
			master.mav.set_mode_send(0, # target sys
				mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
				mode_id)

			time.sleep(1)

			print("[POSITION] - no target found")

####################################################################
# localize()
#	- computationally expensive function to localize based on pos
#	  of apriltag and our pitch and roll
####################################################################
def localize():
	global vx, vy, vz
	
	# configure and start camera stream
	cam = PiCamera()
	cam.resolution = (640, 480)
	cam.framerate = 30
	cam.vflip = True # we mounted it upside down
	raw = PiRGBArray(cam, size=cam.resolution)
	time.sleep(0.1)
	
	# apriltag detector
	opts = apriltag.DetectorOptions(families="tag36h11")
	detector = apriltag.Detector(opts)
	
	# for D term of PID
	last_dy = 0.0
	last_dz = 0.0
	pid_i_term_y = 0.0
	pid_i_term_z = 0.0

	for frame in cam.capture_continuous(raw, format="bgr", use_video_port=True):

		# get image and show as grayscale
		im = frame.array
		
		# first, rotate image to account for roll
		rot_mat = cv2.getRotationMatrix2D(MIDPOINT, -1*roll, 1.0)
		im = cv2.warpAffine(im, rot_mat, cam.resolution, flags=cv2.INTER_LINEAR)
		
		gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
		
		# detect april tags in image
		apr_res = detector.detect(gray)
				
		if len(apr_res) != 0:
			TAG_IN_VIEW = True

			r = apr_res[0]
		
			(pa, pb, pc, pd) = r.corners
			pa = (int(pa[0]), int(pa[1]))
			pb = (int(pb[0]), int(pb[1]))
			pc = (int(pc[0]), int(pc[1]))
			pd = (int(pd[0]), int(pd[1]))
			
			# we can map this to a distance in real life
			total_pix_dist = 0.25*(distance(pa, pb) + distance(pb, pc) + distance(pc, pd) + distance(pa, pd))
			box_angle_taken = FOV_H*(total_pix_dist / 640)
			dx = TAG_SIZE*0.5 / math.tan(3.1415/180 * box_angle_taken)
			
			# now calculate relative position
			dz_pixels = r.center[1] - MIDPOINT[1]
			dy_pixels = r.center[0] - MIDPOINT[0]
			
			# so angle is positive if box is below us (opposite intuition because of NED)
			dz_angle = FOV_V * dz_pixels / 480 - pitch
			dy_angle = FOV_H * dy_pixels / 640
			
			dz = dx*math.tan(dz_angle * 3.1415/180) - SETPOINT[1]
			dy = dx*math.tan(dy_angle * 3.1415/180) - SETPOINT[0]
			
			# now, compute PID velocities
			pid_p_term_y = __P__*dy
			pid_p_term_z = __P__*dz
						
			pid_i_term_y += __I__*pid_p_term_y
			pid_i_term_z += __I__*pid_p_term_z
			
			pid_d_term_y = __D__*(dy - last_dy)
			pid_d_term_z = __D__*(dz - last_dz)
		
			last_dy = dy
			last_dz = dz
			
			# finally, we can compute the desired velocity - display on screen
			vy = pid_p_term_y + pid_i_term_y + pid_d_term_y
			vz = pid_p_term_z + pid_i_term_z + pid_d_term_z
			
			# annotations
			cv2.putText(im, "Pitch: "+str(pitch), (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			cv2.putText(im, "Roll: "+str(roll), (15, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			cv2.putText(im, "dx: "+str(dx), (15, 69), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			cv2.putText(im, "dy: "+str(dy), (15, 91), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			cv2.putText(im, "dz: "+str(dz), (15, 113), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		
			# minus on y axis bc visualization is backwards (pixels go down...)
			cv2.arrowedLine(im, MIDPOINT, (int(MIDPOINT[0] + 150*vy), int(MIDPOINT[1] - 150*vz)), (0, 255, 255), 2)
			cv2.line(im, pa, pb, (0, 255, 0), 2)
			cv2.line(im, pb, pc, (0, 255, 0), 2)
			cv2.line(im, pc, pd, (0, 255, 0), 2)
			cv2.line(im, pd, pa, (0, 255, 0), 2)
			cv2.circle(im, (int(r.center[0]), int(r.center[1])), 5, (0, 0, 255), -1)
		else:
			TAG_IN_VIEW = False

		# this is debugging - show image
		cv2.imshow("frame", im)
		key = cv2.waitKey(1) & 0xff
		
		# clear stream for next image
		raw.truncate(0)
		
		# allow clean way to break out of the loop
		if key == ord("q"):
			sys.exit(0)

####################################################################
# __main__()
#	- start all our threads
####################################################################
if __name__ == "__main__":
	mav = threading.Thread(target=update_state)
	mav.start()
	
	apr = threading.Thread(target=localize)
	apr.start()
	
	vel = threading.Thread(target=send_pid_velocities)
	vel.start()
