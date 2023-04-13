#!/usr/bin/python3
import rospy
from beginner_tutorials.srv import * # サービスファイルすべて読み込み

"""
サービスサーバ(python)のサンプル
"""

# サービスが呼ばれた時のコールバック関数
def handle_add_two_ints(req):
  print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
  return AddTwoIntsResponse(req.a + req.b)


# メイン関数
def add_two_ints_server():
  # ノード名「add_two_ints_server_python」
  rospy.init_node('add_two_ints_server_python')

  # サービスクライアントの設定
  # サービス名「add_two_ints_python」で呼ばれたときにコールバック関数「handle_add_two_ints」を実行
  s = rospy.Service('add_two_ints_python', AddTwoInts, handle_add_two_ints)
  print("Ready to add two ints.")
  rospy.spin()


if __name__ == "__main__":
  add_two_ints_server()