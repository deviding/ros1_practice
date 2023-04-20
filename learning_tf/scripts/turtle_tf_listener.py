#!/usr/bin/python3
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

"""
turtleのtfをlistenするサンプル(python)
"""

if __name__ == '__main__':
  # ノード名「tf_turtle」
  rospy.init_node('tf_turtle')

  # spawnサービスを実行
  rospy.wait_for_service('spawn')
  spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
  spawner(4, 2, 0, 'turtle2')

  # Publishの設定
  # トピック「turtle2/cmd_vel」、Twist型
  turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

  # Listener
  # tfのlistenerはループ内で毎回定義すると失敗するので必ずループ外に記述。
  # 基本的にはクラス変数として定義して使うのが良い。
  # 一旦listenerが作成されると10秒間バッファに貯める。
  listener = tf.TransformListener()

  rate = rospy.Rate(10.0) # 10Hz

  # ROSが起動している限り無限ループ
  while not rospy.is_shutdown():
    try:
      # tfを受信
      # 変換ベースフレーム(from):turtle2, 変換対象フレーム(to):turtle1, rospy.Time(0)で常に最新のtfを取得
      (trans, rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue

    # 取得したtfを元にPublishする内容を設定
    angular = 4 * math.atan2(trans[1], trans[0])
    linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular

    # Publish実行
    turtle_vel.publish(cmd)

    rate.sleep()
