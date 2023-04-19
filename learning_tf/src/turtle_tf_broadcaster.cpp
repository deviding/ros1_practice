#include <ros/ros.h>
#include <tf/transform_broadcaster.h> // tfのBroadcasterを使うヘッダーファイル読み込み
#include <turtlesim/Pose.h>           // turtlesimで使われるPoseメッセージファイルを読み込み

/**
 * turtleのtfをbroadcastするサンプル(C++)
 */

std::string turtle_name; // 亀の名前を保持する変数

// Subscribeした時のコールバック関数
void poseCallback(const turtlesim::PoseConstPtr& msg){
  // Broadcaster
  static tf::TransformBroadcaster br;

  // tfの3次元座標を作成
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );

  // tfの姿勢座標を作成
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);

  // worldフレームにtfを配信
  // 「ros::Time::now()」でタイムスタンプとして現在のROSの時間を設定
  // 親フレーム：world、子フレーム：変数turtle_nameの値
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

// メイン関数
int main(int argc, char** argv){
  // ノード名「my_tf_broadcaster」
  ros::init(argc, argv, "my_tf_broadcaster");

  // 引数チェック
  if (argc != 2) { ROS_ERROR("need turtle name as argument"); return -1; };

  // 引数の1つ目を亀の名前にする
  turtle_name = argv[1];

  // 「亀の名前/pose」のトピックをSubscribe
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};