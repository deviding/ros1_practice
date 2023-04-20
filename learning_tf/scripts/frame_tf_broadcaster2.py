#!/usr/bin/python3
import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf
import math

"""
フレームを追加するサンプル2(python)
"""

if __name__ == '__main__':
  # ノード名「my_tf_frame2_broadcaster」
  rospy.init_node('my_tf_frame2_broadcaster')

  # Broadcaster
  br = tf.TransformBroadcaster()

  rate = rospy.Rate(10.0) # 10Hz

  # ROSが起動している限り無限ループ
  while not rospy.is_shutdown():
    # 現在時刻の秒数にPIをかける
    t = rospy.Time.now().to_sec() * math.pi

    # turtle1を親とするcarrot1フレームを作成して送信
    # 時間に応じて円周上をcarrot1の基準点が移動
    br.sendTransform((2.0 * math.sin(t), 2.0 * math.cos(t), 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "carrot1", "turtle1")
    rate.sleep()
