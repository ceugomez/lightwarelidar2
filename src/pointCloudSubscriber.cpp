/*

This class/node is used to susbcribe to the /points topic that the SF45 is publishing
to. The callback when receiving a /points message will print the first, second, third,
and last point of the data in the message.

To view message fields, use ros2 topic echo /points to see the message description and fields.

*/
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;

class PointcloudSubscriber : public rclcpp::Node
{
  public:
    PointcloudSubscriber()
    : Node("pointcloud_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", 10, std::bind(&PointcloudSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {

      // Ensure there is at least one point in the cloud
      if (msg->data.empty()) {
        RCLCPP_INFO(this->get_logger(), "The point cloud is empty");
        return;
      }

      // Create iterators for the x, y, z, time, and ring fields
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_int(*msg, "intensity");
      sensor_msgs::PointCloud2ConstIterator<float> iter_time(*msg, "time");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_dd(*msg, "dummy_data");

      // Print the first point's x, y, z, intensity, time values, ring number, and dummy data
      RCLCPP_INFO(this->get_logger(), "1st point: x=%f, y=%f, z=%f, intensity=%f, time=%f, ring=%hu, dummy_data=%hu",
            *iter_x, *iter_y, *iter_z, *iter_int, *iter_time, *iter_ring, *iter_dd);

      // Increment iterators to point to the second point
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_int;
      ++iter_time;
      ++iter_ring;
      ++iter_dd;

      // Print the second point's x, y, z, time values and ring number
      RCLCPP_INFO(this->get_logger(), "2nd point: x=%f, y=%f, z=%f, intensity=%f, time=%f, ring=%hu, dummy_data=%hu",
            *iter_x, *iter_y, *iter_z, *iter_int, *iter_time, *iter_ring, *iter_dd);

      // The compiler barfed when I tried to use the decrement operator so hacking this together
      // Iterate to the last point
      int indexOfLastPoint = 1499;  // (999 for 1000 points)
      
      // Move iterators to the last point accounting for the prior increments
      iter_x += (indexOfLastPoint - 3);
      iter_y += (indexOfLastPoint - 3);
      iter_z += (indexOfLastPoint - 3);
      iter_int += (indexOfLastPoint - 3);
      iter_time += (indexOfLastPoint - 3);
      iter_ring += (indexOfLastPoint - 3);
      iter_dd += (indexOfLastPoint - 3);

      RCLCPP_INFO(this->get_logger(), "Last point: x=%f, y=%f, z=%f, intensity=%f, time=%f, ring=%hu, dummy_data=%hu",
            *iter_x, *iter_y, *iter_z, *iter_int, *iter_time, *iter_ring, *iter_dd);
  }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}