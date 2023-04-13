#!/usr/bin/python3
import sys
import rospy
from beginner_tutorials.srv import * # サービスファイルすべて読み込み

"""
サービスクライアント(python)のサンプル
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


# 引数がおかしい場合に表示する関数
def usage():
    return "%s [x y]"%sys.argv[0]


if __name__ == "__main__":
  # ファイル実行時の引数判定
  if len(sys.argv) == 3:
    # 引数の数が正しい場合
    x = int(sys.argv[1])
    y = int(sys.argv[2])
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
  else:
    # 引数の数がおかしい場合
    print(usage())
    sys.exit(1)
