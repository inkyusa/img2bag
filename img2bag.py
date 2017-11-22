#!/usr/bin/python

#Original source from
#https://answers.ros.org/question/11537/creating-a-bag-file-out-of-a-image-sequence/
# Modified by Inkyu Sa, enddl22@gmail.com, 19/Sep/2017 in Bonn
# Many modifications since it didn't work as is...
# Most importantly, a file list should be sorted based on sequence number otherwise...

import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
import ImageFile
import yaml
from sensor_msgs.msg import CameraInfo

#yaml_to_CameraInfo function was stealed from below.
#https://gist.github.com/rossbar/ebb282c3b73c41c1404123de6cea4771#file-yaml_to_camera_info_publisher-py-L13
def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    camName=calib_data["camera_name"]
    return camera_info_msg,camName

def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []
    left_files = []
    right_files = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    if 'left' in f or 'left' in path:
                        left_files.append( os.path.join( path, f ) )
                    elif 'right' in f or 'right' in path:
                        right_files.append( os.path.join( path, f ) )
                    all.append( os.path.join( path, f ) )
    left_files.sort()
    right_files.sort()
    all.sort()
    return all, left_files, right_files


def CreateMonoBag(imgs,bagname,yamlName):
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')
    try:
        for i in range(len(imgs)):
            print("Adding %s" % imgs[i])
            fp = open( imgs[i], "r" )
            p = ImageFile.Parser()

            while 1:
                s = fp.read(1024)
                if not s:
                    break
                p.feed(s)

            im = p.close()

            Stamp = rospy.rostime.Time.from_sec(time.time())
            Img = Image()
            Img.header.stamp = Stamp
            Img.width = im.size[0]
            Img.height = im.size[1]
            if im.mode=='RGB': #(3x8-bit pixels, true color)
              Img.encoding = "rgb8"
              Img.header.frame_id = "camera_rgb_optical_frame"
              Img.step = Img.width*3
              Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            elif im.mode=='L': #(8-bit pixels, black and white)
              Img.encoding = "mono8"
              Img.header.frame_id = "camera_gray_optical_frame"
              Img.step = Img.width
              Img_data=[pix for pixdata in [im.getdata()] for pix in pixdata]
            Img.data = Img_data
            [calib, cameraName]=yaml_to_CameraInfo(yamlName)
            calib.header.stamp = Stamp
            if im.mode=='RGB':
              calib.header.frame_id = "camera_rgb_optical_frame"
            elif im.mode=='L':
              calib.header.frame_id = "camera_gray_optical_frame"
            bag.write( cameraName + '/camera_info', calib, Stamp)
            bag.write( cameraName + '/image_raw', Img, Stamp)
    finally:
        bag.close()

def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs, left_imgs, right_imgs = GetFilesFromDir(args[0])
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[1])
        exit()
    else:
        # create bagfile with mono camera image stream
        CreateMonoBag(all_imgs, args[2], args[1])

if __name__ == "__main__":
    if len( sys.argv ) == 4:
        CreateBag( sys.argv[1:])
    else:
        print( "Usage: img2bag imagedir yamlfile bagfilename")
