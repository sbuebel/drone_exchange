import cv2
import apriltag
import time
import math

from picamera import PiCamera
from picamera.array import PiRGBArray

# meters
TAG_SIZE = 0.1476
MIDPOINT = (320, 240) # center of the image
FOV_H = 62.2 # degrees, horizontal
FOV_V = 48.8 # degrees, vertical

def distance(a, b):
	inner = (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1])
	return math.sqrt(inner)

if __name__ == "__main__":

	roll = 10.0
	pitch = 0.0

	
	# configure and start camera stream
	cam = PiCamera()
	cam.resolution = (640, 480)
	cam.framerate = 30
	raw = PiRGBArray(cam, size=cam.resolution)
	time.sleep(0.1)
	
	# apriltag detector
	opts = apriltag.DetectorOptions(families="tag36h11")
	detector = apriltag.Detector(opts)

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
			r = apr_res[0]
		
			(pa, pb, pc, pd) = r.corners
			pa = (int(pa[0]), int(pa[1]))
			pb = (int(pb[0]), int(pb[1]))
			pc = (int(pc[0]), int(pc[1]))
			pd = (int(pd[0]), int(pd[1]))
			
			# we can map this to a distance in real life
			total_pix_dist = 0.25*(distance(pa, pb) + distance(pb, pc) + distance(pc, pd) + distance(pa, pd))
			box_angle_taken = FOV_H*(total_pix_dist / 640)
			dist = TAG_SIZE*0.5 / math.tan(3.1415/180 * box_angle_taken)
			
			# now calculate relative position
			dy_pixels = r.center[1] - MIDPOINT[1]
			dx_pixels = r.center[0] - MIDPOINT[0]
			
			dy_angle = -1*(FOV_V * dy_pixels / 480) + pitch
			dx_angle = FOV_H * dx_pixels / 640
			
			dy = dist*math.tan(dy_angle * 3.1415/180)
			dx = dist*math.tan(dx_angle * 3.1415/180)
			
		
			# annotations
			cv2.putText(im, "Pitch: "+str(pitch), (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
			cv2.putText(im, "Roll: "+str(roll), (15, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
			
			cv2.putText(im, "Dist: "+str(dist), (15, 69), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			cv2.putText(im, "dx: "+str(dx), (15, 91), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			cv2.putText(im, "dy: "+str(dy), (15, 113), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		
			cv2.line(im, pa, pb, (0, 255, 0), 2)
			cv2.line(im, pb, pc, (0, 255, 0), 2)
			cv2.line(im, pc, pd, (0, 255, 0), 2)
			cv2.line(im, pd, pa, (0, 255, 0), 2)
			
			cv2.circle(im, (int(r.center[0]), int(r.center[1])), 5, (0, 0, 255), -1)
		
		
		
		# this is debugging - show image
		cv2.imshow("frame", im)
		key = cv2.waitKey(1) & 0xff
		
		# clear stream for next image
		raw.truncate(0)
		
		# allow clean way to break out of the loop
		if key == ord("q"):
			break
