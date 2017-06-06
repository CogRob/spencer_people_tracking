#! /usr/bin/env python

import roslib
import rospy
from spencer_tracking_msgs.msg import (
    TrackedPerson,
    TrackedPersons,
    TrackIdentityAssociation,
    TrackIdentityAssociations
)

class Filter:

	def __init__(self):
		self.labeled_tracks_topic = rospy.get_param('~labeled_tracks_topic')
		self.filtered_tracks_topic = rospy.get_param('~filtered_labeled_tracks_topic')
		self.buffer_size = 10
		self.buffer_type = 'low_pass'
		rospy.Subscriber(self.labeled_tracks_topic, TrackIdentityAssociations, self.classified_track_callback) #subscribes to (track,feature) message
		self.filtered_track_publisher = rospy.Publisher(self.filtered_tracks_topic, TrackIdentityAssociations, queue_size=10)

	def classified_track_callback(self, labelled_tracks):
		for labelled_track in labelled_tracks.tracks: