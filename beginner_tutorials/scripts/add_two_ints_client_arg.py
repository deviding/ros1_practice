#!/usr/bin/python3
import sys
import rospy
from beginner_tutorials.srv import * # サービスファイルすべて読み込み

"""
pythonのlaunch引数ありのサンプル
"""

# サービスを実行する関数
def add_two_ints_client(x, y):
  # サービス名「add_two_ints_python」が使えるようになるまで待機
  rospy.wait_for_service('add_two_ints_python')

  try:
    # サービスのハンドラを作成
    add_two_ints = rospy.ServiceProxy('add_two_ints_python', AddTwoInts)
    # サービスの実行
    resp1 = add_two_ints(x, y)
    return resp1.sum
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


if __name__ == "__main__":
  # ノード名の設定(ノード名を設定してないと引数を受け取れない)
  rospy.init_node('add_two_ints_python_arg')

  # 引数の取得とデフォルト値の設定
  x = rospy.get_param("~add1", 1)
  y = rospy.get_param("~add2", 2)

  print("Requesting %s+%s"%(x, y))
  print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
