#!/usr/bin/python3
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

"""
turtleのtfをbroadcastするサンプル(python)
"""

# Subscribeした際のコールバック関数
def handle_turtle_pose(msg, turtlename):
  # Broadcaster
  br = tf.TransformBroadcaster()

  # eulerからquaternionを作成
  q = tf.transformations.quaternion_from_euler(0, 0, msg.theta)

  # worldフレームにtfを配信
  # 「rospy.Time.now()」でタイムスタンプとして現在のROSの時間を設定
  # 親フレーム：world、子フレーム：変数turtlenameの値
  br.sendTransform((msg.x, msg.y, 0), q, rospy.Time.now(), turtlename, "world")


if __name__ == '__main__':
  # ノード名「turtle_tf_broadcaster」
  rospy.init_node('turtle_tf_broadcaster')

  # 引数「turtle」の値を取得
  turtlename = rospy.get_param('~turtle')

  # turtleのPoseを取得してコールバック関数である「handle_turtle_pose」を実行
  rospy.Subscriber('/%s/pose' % turtlename, turtlesim.msg.Pose, handle_turtle_pose, turtlename)
  rospy.spin()
