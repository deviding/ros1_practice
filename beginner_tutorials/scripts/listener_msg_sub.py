#!/usr/bin/python3
import rospy
from beginner_tutorials.msg import Pubsub # 作成したメッセージをインポート


# Subscribeした際のコールバック関数
def callback(data):
  rospy.loginfo(rospy.get_caller_id() + 'I heard python %s', data.msg + ": " + str(data.count))


# listener関数
def listener():
  rospy.init_node('listener_msg_py', anonymous=True)

  # Subscriberの設定(メッセージとしてPubsubを使う)
  rospy.Subscriber('chatter_msg_py', Pubsub, callback)

  # ROSの無限ループ待ち
  rospy.spin()


if __name__ == '__main__':
  listener()