#include "ros/ros.h"
#include "beginner_tutorials/Pubsub.h" // 作成したメッセージ(パッケージ名/メッセージ名.hの形でinclude)

#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_msg");
  ros::NodeHandle n;

  // Publisherの設定(メッセージのPubsubを使用)
  ros::Publisher chatter_pub = n.advertise<beginner_tutorials::Pubsub>("chatter_msg", 1000);
  ros::Rate loop_rate(10); // 10Hz

  // publish回数カウンタ
  int count = 0;

  // ROSが終了するまで無限ループ
  while (ros::ok())
  {
    // Publishするメッセージオブジェクト作成
    beginner_tutorials::Pubsub data;
    data.msg = "hello world";
    data.count = count;

    // 表示用の文字列作成
    std::stringstream ss;
    ss << data.msg << ": " << data.count;
    char char_array[256];
    ss.get(char_array, 256);

    ROS_INFO("%s", char_array); // ROSのログにINFOレベルで書き込み

    // Publish実行(advertiseで設定した形でPublishされる)
    chatter_pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();

    ++count; // Pub回数カウントアップ
  }

  return 0;
}