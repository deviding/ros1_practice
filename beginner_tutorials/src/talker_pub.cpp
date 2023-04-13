#include "ros/ros.h"         // ROSに関するヘッダーファイル
#include "std_msgs/String.h" // std_msgsパッケージのString(要は文字列を送るという意味)

#include <sstream>

/**
 * beginner_tutorialsのPublisherサンプル
 */
int main(int argc, char **argv)
{
  // ノード名を「talker」に設定(/を含んではいけない)
  ros::init(argc, argv, "talker");

  // ROSシステムで通信するためのNodeHandle
  ros::NodeHandle n;

  // Publisherの設定。
  // トピック名「chatter」で「std_msgs::String」型のメッセージを送る。
  // メッセージキューを1000とする。
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // ROSの周波数設定。10Hz(つまり0.1秒ごとにPublish)
  ros::Rate loop_rate(10);

  // publish回数カウンタ
  int count = 0;

  // ROSが終了するまで無限ループ
  while (ros::ok())
  {
    // Publishするメッセージオブジェクト
    std_msgs::String msg;

    // メッセージを作成
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str()); // ROSのログにINFOレベルで書き込み

    // Publish実行(advertiseで設定した形でPublishされる)
    chatter_pub.publish(msg);

    // コールバック関数を一度実行
    // (このソースファイルではコールバックは設定してないのでなくてもよい)
    ros::spinOnce();
    // 設定周波数間隔になるようスリープ
    loop_rate.sleep();
    // Pub回数カウントアップ
    ++count;
  }

  return 0;
}