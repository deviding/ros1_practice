#include "ros/ros.h"
#include "beginner_tutorials/Pubsub.h"

#include<string>
#include <sstream>

/**
 * C++のlaunch引数ありのサンプル
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_msg_arg");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~"); // launchファイルからの引数を取得するNodeHandleを追加(必須)

  // 引数の取得部分
  std::string arg_str = "msg_str"; // 初期値を指定
  pnh.getParam("msg", arg_str);    // 引数を変数に設定

  // Publisherの設定
  ros::Publisher chatter_pub = nh.advertise<beginner_tutorials::Pubsub>("chatter_msg", 1000);
  ros::Rate loop_rate(10); // 10Hz

  // publish回数カウンタ
  int count = 0;

  // ROSが終了するまで無限ループ
  while (ros::ok())
  {
    // Publishするメッセージオブジェクト作成
    beginner_tutorials::Pubsub data;
    data.msg   = arg_str;
    data.count = count;

    // 表示用の文字列作成
    std::stringstream ss;
    ss << data.msg << ": " << data.count;
    char char_array[256];
    ss.get(char_array, 256);

    ROS_INFO("%s", char_array); // ROSのログにINFOレベルで書き込み

    // Publish実行
    chatter_pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();

    ++count; // Pub回数カウントアップ
  }

  return 0;
}