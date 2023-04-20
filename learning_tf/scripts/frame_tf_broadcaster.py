#!/usr/bin/python3
import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf

"""
フレームを追加するサンプル(python)
"""

if __name__ == '__main__':
  # ノード名「my_tf_frame_broadcaster」
  rospy.init_node('my_tf_frame_broadcaster')

  # Broadcaster
  br = tf.TransformBroadcaster()

  rate = rospy.Rate(10.0) # 10Hz

  # ROSが起動している限り無限ループ
  while not rospy.is_shutdown():
    # turtle1を親とするcarrot1フレームを作成して送信
    # turtle1のy軸方向に2移動した場所にcarrot1の基準点が作成されている
    br.sendTransform((0.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "carrot1", "turtle1")
    rate.sleep()
