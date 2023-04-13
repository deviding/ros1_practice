#!/usr/bin/python3
import rospy
from beginner_tutorials.msg import Pubsub # 作成したメッセージをインポート


# Publishする関数
def talker():
  rospy.init_node('talker_msg_py', anonymous=True)

  # Publisherの設定(メッセージとしてPubsubを使う)
  pub = rospy.Publisher('chatter_msg_py', Pubsub, queue_size=10)
  rate = rospy.Rate(10) # 10Hz

  # Publish回数カウンタ
  count = 0

  # ROSが終了するまで無限ループ
  while not rospy.is_shutdown():
    # Publishするメッセージを作成
    pubsub_obj = Pubsub()
    pubsub_obj.msg = "hello world python"
    pubsub_obj.count = count

    rospy.loginfo(pubsub_obj.msg + ": " + str(pubsub_obj.count)) # ROSのログにINFOレベルで書き込み

    # Publishを実行
    pub.publish(pubsub_obj)

    # 設定周波数間隔になるようスリープ
    rate.sleep()

    # カウントアップ
    count += 1


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass