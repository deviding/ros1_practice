#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/**
 * フレームを追加するC++のサンプル2
 */

int main(int argc, char** argv){
  // ノード名「my_tf_frame2_broadcaster」
  ros::init(argc, argv, "my_tf_frame2_broadcaster");
  ros::NodeHandle node;

  // Broadcasterとtransform
  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0); // 10Hz

  // ROSが起動している限り無限ループ
  while (node.ok()){
    // tramsformの設定
    // 時間によって直径2の円周上を移動
    transform.setOrigin( tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

    // turtle1を親とするcarrot1フレームを作成して送信
    // 送信するtramsformの設定にあるように、turtle1を中心とした円周上をcarrot1の基準点が移動
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
    rate.sleep();
  }

  return 0;
};
