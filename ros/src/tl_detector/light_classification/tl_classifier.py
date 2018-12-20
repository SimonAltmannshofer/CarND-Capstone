from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy
import time
import os



class TLClassifier(object):
    def __init__(self, is_site):
        # load classifier
        if is_site:
            model = 'models_frozen/frozen_srb_simon_tf1-3.pb'
        else:
            model = 'models_frozen/frozen_srb_simon_tf1-3.pb'

        # relative to file path, otherwise we have got problems with launch-type specific working directory
        base_path = os.path.dirname(os.path.abspath(__file__))
        model = os.path.join(base_path, model)

        if not os.path.exists(model):
            rospy.logwarn("frozen inference model missing: " + os.path.abspath(model))

        self.index2light = {1: {'id': TrafficLight.GREEN, 'name': 'GREEN'},
                            2: {'id': TrafficLight.RED, 'name': 'RED'},
                            3: {'id': TrafficLight.YELLOW, 'name': 'YELLOW'},
                            4: {'id': TrafficLight.UNKNOWN, 'name': 'UNKNOWN'}}

        self.graph = tf.Graph()

        with self.graph.as_default():
            frozen = tf.GraphDef()

            with tf.gfile.GFile(model, 'rb') as fid:
                serialized = fid.read()
                frozen.ParseFromString(serialized)
                tf.import_graph_def(frozen, name='')

            # generate one session that we will keep open for performance reasons
            self.sess = tf.Session(graph=self.graph)

            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')

            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')

            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        time1 = time.time()

        with self.graph.as_default():
            # single frame processing == batch size 1
            image_list = np.expand_dims(image, axis=0)

            # inference forward pass
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_list})

            time2 = time.time()

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            # we have lots of possible matches, decide which color wins
            keep = scores > 0.5

            if np.any(keep):
                boxes = boxes[keep]
                scores = scores[keep]
                classes = classes[keep]

                members, index, counts = np.unique(classes, return_inverse=True, return_counts=True)
                member_scores = np.zeros((len(members),))

                for i in range(len(members)):
                    member_scores[i] = np.sum(scores[index == i])

                select = np.argmax(member_scores)
                winner = members[select]
                count = counts[select]
                score = member_scores[select]

                rospy.loginfo("traffic lights: {} detections in {}ms: best match {} with {} matches and sum of scores {} ".format(
                    len(scores),
                    round(100*(time2-time1)),
                    self.index2light[winner]['name'],
                    count,
                    score
                ))

                state = self.index2light[winner]['id']
            else:
                rospy.loginfo("traffic lights: no detection in {}ms".format(round(100*(time2-time1))))
                state = TrafficLight.UNKNOWN

        return state
