#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function
from openface_helper import OpenFaceAnotater
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from openface_classifier.cfg import classifierConfig as ConfigType


import roslib
import threading

# roslib.load_manifest('my_package')

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from rwth_perception_people_msgs.msg import UpperBodyDetector
from spencer_tracking_msgs.msg import (
    CompositeDetectedPersons,
    DetectedPerson,
    DetectedPersons,
    TrackedPerson,
    TrackedPersons,
    TrackIdentityAssociation,
    TrackIdentityAssociations
    )

import message_filters

import spencer_detected_person_association


class image_converter:

    def __init__(self, args):
        self.lock = threading.Lock()
        self.image_pub = rospy.Publisher('image_tracked', Image, queue_size=10)
        self.track_pub = rospy.Publisher('persons_tracked', TrackedPersons, queue_size=100)
        self.track_identity_assoc_pub = rospy.Publisher('track_identity_assoc', TrackIdentityAssociations,queue_size=100)

        self.bridge = CvBridge()

        image_sub = \
            message_filters.Subscriber('/spencer/sensors/rgbd_front_top/rgb/image_rect_color'
                , Image)
        detections_sub = \
            message_filters.Subscriber('/spencer/perception_internal/people_detection/rgbd_front_top/upper_body_detector/detections'
                , UpperBodyDetector) #ROIs (the upper body rectangles)
        detected_persons_sub = \
            message_filters.Subscriber('/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body'
                , DetectedPersons)   #detection_ids corresponding to the ROIs(the ids corresponding to the rectangles)
        tracked_persons_sub = \
            message_filters.Subscriber('persons_tracked'
                , TrackedPersons)    #Tracks

        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, detections_sub, detected_persons_sub, tracked_persons_sub], 300, 0.02)
        self.time_sync.registerCallback(self.time_sync_callback)

        track_sync_sub = message_filters.Subscriber('/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body', DetectedPersons) #detection_ids
        self.track_sync = spencer_detected_person_association.TrackSynchronizer(track_sync_sub, 300) #Buffers the detection_ids till it gets tracks with same or newer timestamp
        self.track_sync.registerCallback(self.track_sync_callback) #callback happens once a track is found with same or newer timestamp

        self.server = DynamicReconfigureServer(ConfigType, self.reconfigureClassifier)

        self.openface_annotater = OpenFaceAnotater(args)

    def reconfigureClassifier(self, config, level):
        with self.lock:
            self.openface_annotater.load_model(config['classifier_path'])
            self.openface_annotater.load_features(config['feature_path'])
        return config

    def time_sync_callback(
        self,
        image,
        detections,
        detected_persons,
        tracked_persons,
        ):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError, e:
            print(e)
        print('time_sync_callback')

        detectionIndexLookup = {}
        for i, detected_person in enumerate(detected_persons.detections):
            detectionIndexLookup[detected_person.detection_id] = i
        trackLookups = {}
        for track in tracked_persons.tracks:
            track_id = track.track_id
            detection_id = track.detection_id
            try:
                index = detectionIndexLookup[detection_id]
                trackLookup = {'track_id': track_id, 'index':index}
                trackLookups[track.detection_id] = trackLookup
            except:
                pass

        if not trackLookups:
            print('No track.detection_id found in detected_persons.detections')
        else:
            with self.lock:
                track_assocs = TrackIdentityAssociations()
                track_assocs.header = tracked_persons.header
                for detection_id in trackLookups:
                    trackLookup = trackLookups[detection_id]
                    index = trackLookup['index']
                    track_id = trackLookup['track_id']

                    track_assoc = TrackIdentityAssociation()
                    track_assoc.track_id = track_id
                    track_assoc.detection_id = detection_id

                    print('detection_id: {} track_id: {}'.format(detection_id, track_id))
                    x = detections.pos_x[index]
                    y = detections.pos_y[index]
                    w = detections.width[index]
                    h = detections.height[index]

                    cv_image_crop = cv_image[y:y + h, x:x + w]

                    print ("cv_image_crop size is",cv_image_crop.shape)

                    if cv_image_crop.shape[0]>0 and cv_image_crop.shape[1]>0:
                        cv_image_crop = cv2.cvtColor(cv_image_crop,cv2.COLOR_BGR2RGB)
                        cv_image_crop,identities = self.openface_annotater.predictWithLabel(cv_image_crop,[], multiple=False, scale=None)
                        #identities has a single element
                        person = identities[0]
                        track_assoc.person_name = person
                        cv_image_crop = cv2.cvtColor(cv_image_crop,cv2.COLOR_RGB2BGR)
                        track_assocs.tracks.append(track_assoc)
                        try:
                            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image_crop,
                                                   'bgr8'))
                        except CvBridgeError, e:
                            print(e)

                self.track_identity_assoc_pub.publish(track_assocs)

    def track_sync_callback(
        self,
        trackAssociation,
        detected_persons):
        age = rospy.Time.now() - detected_persons.header.stamp
        output = "New detections with track association available (age of detections = %.2f sec)! Detection to track association: " % age.to_sec()

        if detected_persons.detections:
            tracked_persons = TrackedPersons() # TrackedPersons message created 
            tracked_persons.header = detected_persons.header
            for detected_person in detected_persons.detections:
                detection_id = detected_person.detection_id
                track_id = trackAssociation.lookupTrackId(detection_id)
                if track_id is not None:
                    # print('detection_id: {} track_id: {}'.format(detection_id, track_id))
                    tracked_person = TrackedPerson()
                    tracked_person.track_id = track_id
                    tracked_person.detection_id = detection_id
                    tracked_persons.tracks.append(tracked_person)
                    output += "\n[det %d --> track %s]" % (detection_id, str(track_id))
            self.track_pub.publish(tracked_persons)
        else:
            output += "Empty set of detections!"

        # rospy.loginfo(output)

def main(args):
    ic = image_converter(args)
    rospy.init_node('image_converter', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
