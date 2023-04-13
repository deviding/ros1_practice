#!/usr/bin/python3
import rospy                    # ROSに関するライブラリのインポート
from std_msgs.msg import String # std_msgsパッケージのString(要は文字列を送るという意味)


# Publishする関数
def talker():
  # ノード名を「talker_py」に設定(/を含んではいけない)
  rospy.init_node('talker_py', anonymous=True)

  # Publisherの設定。
  # トピック名「chatter_py」で「std_msgsのString」型のメッセージを送る。
  # メッセージキューを10とする。
  pub = rospy.Publisher('chatter_py', String, queue_size=10)

  # ROSの周波数設定。10Hz(つまり0.1秒ごとにPublish)
  rate = rospy.Rate(10)

  # ROSが終了するまで無限ループ
  while not rospy.is_shutdown():
    # Publishするメッセージを作成
    hello_str = "hello world python %s" % rospy.get_time()

    rospy.loginfo(hello_str) # ROSのログにINFOレベルで書き込み

    # Publishを実行
    pub.publish(hello_str)

    # 設定周波数間隔になるようスリープ
    rate.sleep()


if __name__ == '__main__':
  # ファイル実行時
  try:
    # talker関数を呼び出し
    talker()
  except rospy.ROSInterruptException:
    # ROSのエラーが出たら無視
    pass