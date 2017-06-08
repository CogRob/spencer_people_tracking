#! /usr/bin/env python
import rospy
import roslib
import threading
from openface_classifier.srv import *
from spencer_tracking_msgs.msg import (
    TrackedPerson,
    TrackedPersons,
    TrackIdentityAssociation,
    TrackIdentityAssociations
)


class Label_Pose_Server:

	def __init__(self):
		self.lock = threading.Lock()
		self.label_track_map = {} #label->TrackedPerson object
		self.trackid_label_map = {} #track_id->label
		self.labeled_tracks_topic = rospy.get_param('~labeled_tracks_topic')
		self.tracks_with_pose_topic = rospy.get_param('~tracks_with_pose_topic')
		rospy.Service('label_pose', LabelPoseService, self.label_pose_service_callback)
		rospy.Subscriber(self.tracks_with_pose_topic, TrackedPersons, self.tracks_with_pose_callback) #subscribe to Tracks
		rospy.Subscriber(self.labeled_tracks_topic, TrackIdentityAssociations, self.labeled_tracks_callback) #subscribe to Labelled Tracks outputted by classifier

	def tracks_with_pose_callback(self,tracked_persons):
		with self.lock:
			print 'tracks_with_pose_callback:',tracked_persons
			for tracked_person in tracked_persons.tracks:
				track_id = tracked_person.track_id
				print track_id
				print self.trackid_label_map
				if track_id in self.trackid_label_map: #if track_id is associated to a person
					label = self.trackid_label_map[track_id]
					self.label_track_map[label] = tracked_person #update trackedperson object corresponding to label

					print self.label_track_map


	def labeled_tracks_callback(self,labelled_tracks):
		with self.lock:
			print 'labeled_tracks_callback'
			for labelled_track in labelled_tracks.tracks:
				label = labelled_track.person_name
				track_id = labelled_track.track_id
				if label!='Unknown':
					print '(track_id,label):',(track_id,label)
					#remove old entry logic
					remove_ids = []
					for iter_id in self.trackid_label_map:
						if self.trackid_label_map[iter_id]==label and iter_id!=track_id:
							remove_ids.append(iter_id)
					for iter_id in remove_ids:
						del self.trackid_label_map[iter_id]
					
					self.trackid_label_map[track_id] = label
			print self.trackid_label_map


	def label_pose_service_callback(self,request):
		label = request.label
		tracked_person = None
		with self.lock:
			#search in the dictionary and return pose
			if label in self.label_track_map:
				tracked_person = self.label_track_map[label]
		if tracked_person:
			return LabelPoseServiceResponse(tracked_person)
		else:
			print 'Label: '+str(label)+' not found'
			print self.trackid_label_map
			print self.label_track_map

if __name__ == '__main__':
	rospy.init_node('label_pose_service_node')
	label_pose_server_object = Label_Pose_Server()
	rospy.spin()