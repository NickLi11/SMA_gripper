#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class ArTransformBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.ar_tfs = ["ar_marker_4", "ar_marker_6", "ar_marker_7"]

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            transforms = []

            for tf in self.ar_tfs:
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = tf
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = "off_" + tf
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.1

                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                transforms.append(t)

            print transforms

            tfm = tf2_msgs.msg.TFMessage(transforms)
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('ar_transform_broadcaster')
    atb = ArTransformBroadcaster()

    rospy.spin()