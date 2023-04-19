#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/**
 * フレームを追加するC++のサンプル
 */

int main(int argc, char** argv){
  // ノード名「my_tf_frame_broadcaster」
  ros::init(argc, argv, "my_tf_frame_broadcaster");
  ros::NodeHandle node;

  // Broadcasterとtransform
  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0); // 10Hz

  // ROSが起動している限り無限ループ
  while (node.ok()){
    // tramsformの設定
    transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

    // turtle1を親とするcarrot1フレームを作成して送信
    // 送信するtramsformの設定にあるように、turtle1のy軸方向に2移動した場所にcarrot1の基準点が作成されている
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
    rate.sleep();
  }

  return 0;
};
