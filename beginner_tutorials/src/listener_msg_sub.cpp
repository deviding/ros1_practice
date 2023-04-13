#include "ros/ros.h"
#include "beginner_tutorials/Pubsub.h" // 作成したメッセージ(パッケージ名/メッセージ名.hの形でinclude)


// Subscribeした際のコールバック関数(メッセージのPubsubを使用)
void chatterCallback(const beginner_tutorials::Pubsub::ConstPtr& data)
{
  // 表示用の文字列作成
  std::stringstream ss;
  ss << data->msg << ": " << data->count;
  char char_array[256];
  ss.get(char_array, 256);

  ROS_INFO("I heard: [%s]", char_array); // ROSのログにINFOレベルで書き込み(表示)
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_msg");
  ros::NodeHandle n;

  // Subscriberの設定。
  ros::Subscriber sub = n.subscribe("chatter_msg", 1000, chatterCallback);

  // ROSの無限ループ待ち
  ros::spin();

  return 0;
}