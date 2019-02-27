#!/usr/bin/env python
import rospy
import numpy as np
import dlib
from std_msgs.msg import String
from image_recognition_msgs.msg import Recognitions, CategoryProbability, FaceProperties
from sensor_msgs.msg import Image, RegionOfInterest
import rosservice
from cv_bridge import CvBridge, CvBridgeError
from image_recognition_msgs.msg import Annotation
from robot_vision.msg import PersonCharacteristics, PositionCommand


class pepper_answer:

	def recognize_object(self, img):
	    try:
                result = self.obj_srv(image=img)
	    except Exception as e:
		print(e)
	    
	    for r in result.recognitions:
                text_array = []
                best = CategoryProbability(label="unknown", probability=r.categorical_distribution.unknown_probability)
                for p in r.categorical_distribution.probabilities:
                    text_array.append("%s: %.2f" % (p.label, p.probability))
                    if p.probability > best.probability:
                        best = p
            print(result.recognitions)

	def face_properties(self, img):

	    if self.already == True:
		return
	    self.already = True
	    try:
                result = self.prop_srv(face_image_array=[img])
	    except Exception as e:
		print(e)
	    
	    print(result.properties_array)
	    #msg = ""
	    msg = PersonCharacteristics()
	   
            for properties in result.properties_array:
				self.guest_gender = properties.gender
				self.guest_age = properties.age
				self.guest_glasses = properties.glasses

		# msg += "- FaceProperties(gender=%s, age=%s, glasses=%s, mood=%s)" % \
        #            ("male" if properties.gender == FaceProperties.MALE else "female", properties.age,
		#      "false" if properties.glasses == 0 else "true")

		#return msg




	def recognize_face(self, img):
	    try:
                result = self.rec_srv(image=img)
	    except Exception as e:
		print(e)
	    
	    for r in result.recognitions:
                text_array = []
                best = CategoryProbability(label="unknown", probability=r.categorical_distribution.unknown_probability)
                for p in r.categorical_distribution.probabilities:
                    text_array.append("%s: %.2f" % (p.label, p.probability))
                    if p.probability > best.probability:
                        best = p
	        if best.probability > self.tresh:
		    self.pepper_face_recognized_pub.publish(best.label)
		    self.recognize_object(img)
		    self.face_properties(img)
		    msg = PersonCharacteristics()
		    msg.name = best.label
		    msg.gender = self.guest_gender
		    msg.age = self.guest_age
		    msg.glasses = self.guest_glasses
		    self.pepper_prop_pub.publish(msg)
		else:
		    self.pepper_face_recognized_pub.publish("unknown_guest")
		    self.learn_face(img)

	def learn_face(self, roi_image):
	    
	    if self.label == "nobody":
		return

	    self.tresh = 50
	    print("try to learn")

	    if roi_image is not None and self.an_srv is not None:
            	height = roi_image.height
                width = roi_image.width
            try:
                self.an_srv(image=roi_image,
                          annotations=[Annotation(label=self.label,
                                                  roi=RegionOfInterest(x_offset=0, y_offset=0,
                                                                       width=width, height=height))])
            except Exception as e:		
		print(e)
		

	def callback_recognition(self, data):
	    greetings=["hello","greetings","hi","hey","sup","good day"]
	    # Get the best recognition
	    for r in data.recognitions:
		best = CategoryProbability(label="unknown", probability=r.categorical_distribution.unknown_probability)
		for p in r.categorical_distribution.probabilities:
		        if p.probability > best.probability:
		                    best = p

		# If the confidence is higher than 80% publish the result and which camera the img came from
		if best.probability > 0.8:
                    if best.label != self.last_rec_label:
	                self.last_rec_label = best.label
		        self.pepper_string_pub.publish(np.random.choice(greetings)+best.label)


        def callback_image(self, img):
		try:
		
	        	cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
			faceRects = self.hog_face_detector(cv_image, 0)
			print(len(faceRects))
			if len(faceRects) == 1:
				self.recognize_face(img)
			elif len(faceRects) == 0:
				#self.pepper_face_recognized_pub.publish("nobody")		
				self.tresh = 2
				self.label = "nobody"	
			else:
				#self.pepper_face_recognized_pub.publish("multiple")
				self.tresh = 2
				self.label = "nobody"
				
		except CvBridgeError as e:
			rospy.logerr(e)
	
			
   

        def callback_label(self, data): 
		self.label = data.data

	def callback_currentloc(self,data):
		if data.data == "bar" and self.label != "nobody":
			self.pepper_face_recognized_pub.publish(self.label)
		else:
			self.pepper_face_recognized_pub.publish("unknown_guest")
			

	def pepper_answer(self):

	    # In ROS, nodes are uniquely named. If two nodes with the same
	    # name are launched, the previous one is kicked off. The
	    # anonymous=True flag means that rospy will choose a unique
	    # name for our 'listener' node so that multiple listeners can
	    # run simultaneously.
	    rospy.init_node('pepper_answer', anonymous=True)
            self.last_rec_label = "nobody"

	    self.hog_face_detector = dlib.get_frontal_face_detector()
	    self.bridge = CvBridge()
            self.label = "nobody"
	    self.tresh = 2
            self.already = False
	    self.guest_gender = 0
            self.guest_glasses = 0
            self.guest_age = 20

	    # publishers
	    self.pepper_string_pub  = rospy.Publisher("/speech", String, queue_size=1)
	    self.pepper_face_recognized_pub  = rospy.Publisher("/which_guest", String, queue_size=1)
	    self.pepper_prop_pub = rospy.Publisher("/person_characteristics", PersonCharacteristics, queue_size=1)
	    self.pepper_go_to_pub = rospy.Publisher("/position_command", PositionCommand, queue_size=1)

	    # services
	    self.rec_srv = rospy.ServiceProxy("/recognize", rosservice.get_service_class_by_name("/recognize"))
	    self.an_srv = rospy.ServiceProxy("/annotate", rosservice.get_service_class_by_name("/annotate"))
	    self.obj_srv = rospy.ServiceProxy("/object_recognition", rosservice.get_service_class_by_name("/object_recognition"))
	    self.prop_srv = rospy.ServiceProxy("/get_face_properties", rosservice.get_service_class_by_name("/get_face_properties"))

	    # subscribers
	    rospy.Subscriber("/recognitions", Recognitions, self.callback_recognition)
            rospy.Subscriber("/naoqi_driver/camera/front/image_raw", Image, self.callback_image)
            rospy.Subscriber("/person_name", String, self.callback_label)
	    rospy.Subscriber("/current_location", String, self.callback_currentloc)

	    # spin() simply keeps python from exiting until this node is stopped
	    rospy.spin()

if __name__ == '__main__':
    pepper_answer = pepper_answer()
    pepper_answer.pepper_answer()


