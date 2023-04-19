#include <ros/ros.h>
#include <tf/transform_listener.h> // tfのListenerを使うヘッダーファイル読み込み
#include <geometry_msgs/Twist.h>   // Twistメッセージファイルを読み込み
#include <turtlesim/Spawn.h>       // turtlesimで使われるSpawnサービスファイルを読み込み

/**
 * turtleのtfをlistenするサンプル5(C++)
 */

int main(int argc, char** argv){
  // ノード名「my_tf_listener5」
  ros::init(argc, argv, "my_tf_listener5");
  ros::NodeHandle node;

  // spawnサービスを実行
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  // Publishの設定
  // トピック「turtle2/cmd_vel」、Twist型
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  // Listener
  // tfのlistenerはループ内で毎回定義すると失敗するので必ずループ外に記述。
  // 基本的にはクラス変数として定義して使うのが良い。
  // 一旦listenerが作成されると10秒間バッファに貯める。
  tf::TransformListener listener;

  ros::Rate rate(10.0); // 10Hz

  // ROSが起動している限り無限ループ
  while (node.ok()){
    // タイムスタンプを持つtf
    tf::StampedTransform transform;
    try {
      // tfを受信
      // 現在時刻を取得
      ros::Time now = ros::Time::now();

      // 現在時刻より5.0秒前の時刻を取得
      ros::Time past = now - ros::Duration(5.0);

      // turtle1の5秒前のフレーム待ち。最大1秒間待機
      listener.waitForTransform("/turtle2", now, "/turtle1", past, "/world", ros::Duration(1.0));

      // turtle1の5秒前のフレームを取得
      listener.lookupTransform("/turtle2", now, "/turtle1", past, "/world", transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // 取得したtfを元にPublishする内容を設定
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    vel_msg.linear.x  = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    
    // Publish実行
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};