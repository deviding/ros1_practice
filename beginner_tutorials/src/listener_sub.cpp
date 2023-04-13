#include "ros/ros.h"         // ROSに関するヘッダーファイル
#include "std_msgs/String.h" // std_msgsパッケージのString(要は文字列を受け取るという意味)

/**
 * beginner_tutorialsのSubscriberサンプル
 */

// Subscribeした際のコールバック関数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  // ノード名を「listener」に設定(/を含んではいけない)
  ros::init(argc, argv, "listener");

  // ROSシステムで通信するためのNodeHandle
  ros::NodeHandle n;

  // Subscriberの設定。
  // トピック名「chatter」でSubscribeしたときにコールバック関数として「chatterCallback」を実行。
  // メッセージキューを1000とする。
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  // ROSの無限ループ待ち
  // (Subscribeされる度にコールバック関数が処理される)
  ros::spin();

  return 0;
}