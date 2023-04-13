#include <ros/ros.h>
#include <visualization_msgs/Marker.h> // マーカー表示のメッセージファイルをinclude

/**
 * rvizにマーカーを表示させるサンプル
 * (ソースコードから直接図形を表示させることはまずないので内容としてはそこまで重要ではない)
 */

int main( int argc, char** argv )
{
  // ノード名「basic_shapes」
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1); // 1Hz

  // 「visualization_marker」というトピック名でPublish
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // マーカータイプの初期値を「Cube」に設定
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    // Publishするオブジェクト
    visualization_msgs::Marker marker;

    // フレームIDとタイムスタンプを設定(これらは後々出てきた段階で説明する)
    marker.header.frame_id = "my_frame";
    marker.header.stamp = ros::Time::now();

    // マーカーのネームスペースとIDを設定。IDはユニークである必要がある。
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape; // 形を設定

    // マーカーアクションを設定
    marker.action = visualization_msgs::Marker::ADD;

    // 表示させる座標位置と方向を設定
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 大きさを設定
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // 色を設定
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // ライフタイムを設定(ros::Duration()でライフタイムは無限)
    marker.lifetime = ros::Duration();

    // マーカーをPublish
    marker_pub.publish(marker);

    // 形を変更
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();
  }
}