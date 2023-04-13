#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h" // 作成したサービス(パッケージ名/サービス名.hの形でinclude)


/**
 * サービスサーバ(C++)のサンプル
 */
// 足し算を行う関数
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  // リクエストのa, bの足し算を計算してレスポンスのsumに結果を設定
  res.sum = req.a + req.b;
  // ログに結果を出力
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);

  // サービスのコールバック関数自体の結果を返す
  return true;
}

int main(int argc, char **argv)
{
  // ノード名「add_two_ints_server」
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  // サービスサーバの定義
  // サービス名「add_two_ints」で呼ばれたときにコールバック関数「add」を実行
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");

  // ROSの無限ループ待ち
  ros::spin();

  return 0;
}