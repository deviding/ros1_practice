#!/usr/bin/python3
import rospy                    # ROSに関するライブラリのインポート
from std_msgs.msg import String # std_msgsパッケージのString(要は文字列を受け取るという意味)


# Subscribeした際のコールバック関数
def callback(data):
  rospy.loginfo(rospy.get_caller_id() + 'I heard python %s', data.data)


# listener関数
def listener():
  # ノード名を「listener_py」に設定(/を含んではいけない)
  rospy.init_node('listener_py', anonymous=True)

  # Subscriberの設定。
  # トピック名「chatter_py」でSubscribeしたときにコールバック関数として「callback」を実行。
  rospy.Subscriber('chatter_py', String, callback)

  # ROSの無限ループ待ち
  # (Subscribeされる度にコールバック関数が処理される)
  rospy.spin()


if __name__ == '__main__':
  # ファイル実行時にlistener関数を呼び出し
  listener()