#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h" // 作成したサービス(パッケージ名/サービス名.hの形でinclude)
#include <cstdlib>

/**
 * サービスクライアント(C++)のサンプル
 */

int main(int argc, char **argv)
{
  // ノード名「add_two_ints_client」
  ros::init(argc, argv, "add_two_ints_client");

  // ファイル実行時の引数の数を判定
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;

  // サービスクライアントの定義
  // サービス名「add_two_ints」を呼び出す
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");

  // サービスに送るリクエストのサービスを作成(ファイル実行時の引数を設定)
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  // サービスの呼び出し
  // サービスの処理が終了するまで待機になる(正常終了ならtrueが返ってくる)
  if (client.call(srv))
  {
    // 正常終了の場合
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    // 異常終了の場合
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}