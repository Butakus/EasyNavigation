// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in sh0rt)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/types/ImagePerception.hpp"

#include "tf2_ros/transform_listener.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "gtest/gtest.h"


class PerceptionsTestCase : public ::testing::Test
{
protected:
  ~PerceptionsTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    if (!initialized) {
      rclcpp::init(0, nullptr);
      initialized = true;
    }
  }

  void TearDown()
  {
  }

  bool initialized {false};
};

// Helper subclass to inspect PointPerception internals in tests.
class TestPointPerception : public easynav::PointPerception
{
public:
  // Accessors to internal buffer state
  std::size_t buffer_size() const {return buffer.size();}
  bool buffer_empty() const {return buffer.empty();}
  bool buffer_full() const {return buffer.full();}

  // Get latest element stored in the circular buffer
  bool buffer_latest(easynav::PointPerceptionBufferType & out) const
  {
    return buffer.latest(out);
  }

  // Debug helper: print buffer contents and current visible state
  void debug_print_buffer([[maybe_unused]] const std::string & header)
  {
    // std::cerr << "-----------------------------\n";
    // std::cerr << "DEBUG: " << header << "\n";
    // std::cerr << "Visible state:\n";
    // std::cerr << "  frame_id=" << frame_id
    //          << " stamp=" << stamp.nanoseconds()
    //          << " valid=" << std::boolalpha << valid
    //          << " new_data=" << new_data << "\n";

    const std::size_t sz = buffer.size();
    // const std::size_t cap = buffer.capacity();
    // std::cerr << "Buffer state: size=" << sz
    //          << " capacity=" << cap << "\n";

    // Dump logical content by popping all items and re-pushing them
    std::vector<easynav::PointPerceptionBufferType> tmp;
    tmp.reserve(sz);

    for (std::size_t i = 0; i < sz; ++i) {
      easynav::PointPerceptionBufferType item;
      if (buffer.pop(item)) {
        // std::cerr << "  [" << i << "] "
        //           << "stamp=" << item.stamp.nanoseconds()
        //           << " frame=" << item.frame
        //           << " points=" << item.data.size()
        //           << "\n";
        tmp.push_back(std::move(item));
      } else {
        // std::cerr << "  [" << i << "] (pop failed)\n";
        break;
      }
    }

    // Restore buffer content
    for (auto & item : tmp) {
      buffer.push(std::move(item));
    }

    // std::cerr << "-----------------------------\n";
  }
};

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser_1";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[4] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser_2";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 4.0);
  ret.ranges[4] = 1.0;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_0(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = static_cast<float>(i);
    p.y = static_cast<float>(i);
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = std::numeric_limits<float>::infinity();
    p.y = std::numeric_limits<float>::infinity();
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 5.0;
    p.y = 5.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pc[4].x = 0.3;
  pc[4].y = 0.3;
  pc[4].z = 0.0;

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 5.0;
    p.y = 5.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pc[4].x = 1.0;
  pc[4].y = 1.0;
  pc[4].z = 0.0;

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

using namespace std::chrono_literals;


TEST_F(PerceptionsTestCase, PointPerceptionHandlerWorks)
{
  using easynav::RTTFBuffer;

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_handler_node");

  // Use the same RTTFBuffer singleton that integrate_pending_perceptions() uses
  auto tf_buffer = RTTFBuffer::getInstance();
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  // Robot and sensor frames
  const std::string robot_frame = tf_buffer->get_tf_info().robot_frame;
  const std::string sensor_frame = "base_laser";

  // Common timestamp for TF and scan
  rclcpp::Time ts = node->get_clock()->now();

  // Insert TF: robot_frame -> base_laser
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = ts;
  tf.header.frame_id = robot_frame;
  tf.child_frame_id = sensor_frame;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.w = 1.0;  // identity
  tf_buffer->setTransform(tf, "test_authority", false);

  auto handler = std::make_shared<easynav::PointPerceptionHandler>();
  auto perception = handler->create("laser1");

  auto cb_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/test_scan",
    "sensor_msgs/msg/LaserScan",
    perception,
    cb_group);

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub =
    node->create_publisher<sensor_msgs::msg::LaserScan>(
      "/test_scan",
      rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());

  const auto data = get_scan_test_2(ts);
  laser_pub->publish(data);

  auto start = node->get_clock()->now();
  while (node->get_clock()->now() - start < 100ms) {
    exe.spin_some();
  }

  auto loaded = std::dynamic_pointer_cast<easynav::PointPerception>(perception);
  ASSERT_NE(loaded, nullptr);

  ASSERT_TRUE(loaded->valid);
  ASSERT_TRUE(loaded->new_data);
  ASSERT_EQ(loaded->frame_id, sensor_frame);
  ASSERT_EQ(loaded->stamp.nanoseconds(), ts.nanoseconds());
  ASSERT_EQ(loaded->data.size(), data.ranges.size());

  double angle = data.angle_min;
  for (size_t i = 0; i < 16; ++i) {
    ASSERT_NEAR(loaded->data[i].x, data.ranges[i] * std::cos(angle), 0.01);
    ASSERT_NEAR(loaded->data[i].y, data.ranges[i] * std::sin(angle), 0.01);
    ASSERT_NEAR(loaded->data[i].z, 0.0, 0.0001);

    angle += data.angle_increment;
  }
}

TEST_F(PerceptionsTestCase, PointPerceptionBufferAndTFWorks)
{
  using easynav::RTTFBuffer;
  using easynav::PointPerceptionHandler;

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_point_perception_buffer");

  // Initialize RTTFBuffer singleton with this node clock
  auto tf_buffer = RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  // Perception and handler
  auto perception = std::make_shared<TestPointPerception>();
  auto handler = std::make_shared<PointPerceptionHandler>();

  auto cb_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/test_scan3",
    "sensor_msgs/msg/LaserScan",
    perception,
    cb_group);

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/test_scan3", rclcpp::SensorDataQoS().reliable());
  laser_pub->on_activate();  // lifecycle publisher must be activated

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());

  //
  // 1) Initial state: integrate_pending_perceptions keeps an "empty" perception visible.
  //    Since there is no pending data and the buffer is empty, the buffer
  //    should remain empty.
  //
  // std::cerr << "=== STEP 1: initial flush ===\n";
  // perception->debug_print_buffer("Before first flush");
  perception->integrate_pending_perceptions();
  // perception->debug_print_buffer("After first flush");

  EXPECT_FALSE(perception->valid);
  EXPECT_EQ(perception->data.size(), 0u);
  EXPECT_EQ(perception->buffer_size(), 0u);  // buffer must remain empty

  //
  // 2) Add TF base_link -> base_laser_1 and publish 20 scans with valid TFs.
  //    Each time, we expect integrate_pending_perceptions() to:
  //      - choose the newest item with a valid TF (the just received one),
  //      - drop older items,
  //      - keep only that candidate in the buffer.
  //
  std::vector<rclcpp::Time> valid_stamps;
  valid_stamps.reserve(20);

  const std::string robot_frame = "base_link";
  const std::string sensor_frame = "base_laser_1";

  for (int i = 0; i < 20; ++i) {
    // Time for this scan
    rclcpp::Time ts = node->get_clock()->now();
    valid_stamps.push_back(ts);

    // Add corresponding TF to RTTFBuffer: base_link -> base_laser_1
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = ts;
    tf.header.frame_id = robot_frame;
    tf.child_frame_id = sensor_frame;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;  // identity
    tf_buffer->setTransform(tf, "test_authority", false);

    // Publish scan in frame base_laser_1 with that stamp
    auto scan = get_scan_test_3(ts);
    laser_pub->publish(scan);

    // Spin until callback runs
    exe.spin_some(std::chrono::milliseconds(50));

    // std::cerr << "=== STEP 2: valid scan #" << i
    //          << " ts=" << ts.nanoseconds() << " ===\n";
    // perception->debug_print_buffer("Before flush after valid scan");

    // Now flush the buffer and check that the last perception is this one
    perception->integrate_pending_perceptions();

    // perception->debug_print_buffer("After flush after valid scan");

    EXPECT_TRUE(perception->valid);
    EXPECT_EQ(perception->frame_id, sensor_frame);
    EXPECT_EQ(perception->stamp.nanoseconds(), ts.nanoseconds());
    EXPECT_EQ(perception->data.size(), scan.ranges.size());

    // With the current integrate_pending_perceptions() semantics:
    //  - one candidate with valid TF is kept,
    //  - there are no newer candidates yet,
    //  -> buffer must contain exactly 1 entry.
    EXPECT_EQ(perception->buffer_size(), 1u);

    easynav::PointPerceptionBufferType latest_item;
    ASSERT_TRUE(perception->buffer_latest(latest_item));
    EXPECT_EQ(latest_item.stamp.nanoseconds(), perception->stamp.nanoseconds());
    EXPECT_EQ(latest_item.frame, sensor_frame);
  }

  // std::cerr << "=== STEP 2: after all 20 valid scans ===\n";
  // perception->debug_print_buffer("End of step 2");

  // After 20 valid insertions with current behavior, buffer still holds only
  // the newest valid candidate.
  EXPECT_FALSE(perception->buffer_full());
  EXPECT_EQ(perception->buffer_size(), 1u);
  EXPECT_EQ(
    perception->stamp.nanoseconds(),
    valid_stamps.back().nanoseconds());

  //
  // 3) Publish 4 scans without adding their TFs.
  //    We expect, after each integrate_pending_perceptions():
  //      - visible perception remains the last valid one from step 2,
  //      - buffer size grows by one per new invalid candidate
  //        (they are kept because they might become valid in the future).
  //
  std::array<rclcpp::Time, 4> invalid_stamps;

  const auto last_valid_stamp_ns = perception->stamp.nanoseconds();
  const std::size_t base_size_step3 = perception->buffer_size();  // should be 1

  for (int i = 0; i < 4; ++i) {
    rclcpp::Time ts =
      node->get_clock()->now() + rclcpp::Duration(0, 10'000'000 * i);
    invalid_stamps[i] = ts;

    auto scan = get_scan_test_3(ts);
    laser_pub->publish(scan);
    exe.spin_some(std::chrono::milliseconds(50));

    // std::cerr << "=== STEP 3: invalid scan #" << i
    //          << " ts=" << ts.nanoseconds() << " ===\n";
    // perception->debug_print_buffer("Before flush after invalid scan");

    // No corresponding TF for this stamp yet
    perception->integrate_pending_perceptions();

    // perception->debug_print_buffer("After flush after invalid scan");

    EXPECT_TRUE(perception->valid);
    EXPECT_EQ(perception->frame_id, sensor_frame);
    EXPECT_EQ(perception->stamp.nanoseconds(), last_valid_stamp_ns);

    // Buffer size should have increased by one for each new invalid candidate
    // while still keeping the last valid one from step 2.
    std::size_t expected_size = base_size_step3 + static_cast<std::size_t>(i + 1);
    EXPECT_EQ(perception->buffer_size(), expected_size);
  }

  // std::cerr << "=== STEP 3: after 4 invalid scans ===\n";
  // perception->debug_print_buffer("End of step 3");

  //
  // 4) Now we publish TFs for those 4 previously invalid scans, one by one.
  //    Semantics:
  //      - After adding a TF for invalid_stamps[i] and calling integrate_pending_perceptions():
  //          * that candidate becomes the newest valid one,
  //          * all older entries are pruned,
  //          * newer entries (if any) are kept.
  //      - In our setup at this point:
  //          buffer has: [last valid from step 2, invalid0, invalid1, invalid2, invalid3]
  //        so sizes after each flush should be:
  //          i=0 -> 4, i=1 -> 3, i=2 -> 2, i=3 -> 1.
  //
  std::size_t buffer_size_step3_end = perception->buffer_size();  // should be 5
  ASSERT_EQ(buffer_size_step3_end, 5u);
  std::size_t expected_size_step4 = buffer_size_step3_end;

  for (int i = 0; i < 4; ++i) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = invalid_stamps[i];
    tf.header.frame_id = robot_frame;
    tf.child_frame_id = sensor_frame;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    // std::cerr << "=== STEP 4: adding TF for invalid_stamps[" << i
    //          << "] ts=" << invalid_stamps[i].nanoseconds() << " ===\n";
    // perception->debug_print_buffer("Before setTransform");

    tf_buffer->setTransform(tf, "test_authority", false);

    // perception->debug_print_buffer("Before flush after TF");

    perception->integrate_pending_perceptions();

    // perception->debug_print_buffer("After flush after TF");

    EXPECT_TRUE(perception->valid);
    EXPECT_EQ(perception->frame_id, sensor_frame);
    EXPECT_EQ(
      perception->stamp.nanoseconds(),
      invalid_stamps[i].nanoseconds());

    // After choosing invalid_stamps[i] as newest valid:
    //   - older entries are pruned,
    //   - newer ones are kept.
    // With our pattern, sizes must go 4,3,2,1.
    expected_size_step4 -= 1;
    EXPECT_EQ(perception->buffer_size(), expected_size_step4);

    easynav::PointPerceptionBufferType latest_item;
    ASSERT_TRUE(perception->buffer_latest(latest_item));
    EXPECT_EQ(latest_item.stamp.nanoseconds(), invalid_stamps[3].nanoseconds());
    EXPECT_EQ(latest_item.frame, sensor_frame);
  }

  // std::cerr << "=== STEP 4: end of step 4 ===\n";
  // perception->debug_print_buffer("End of step 4");
  EXPECT_EQ(perception->buffer_size(), 1u);

  //
  // 5) Final scenario:
  //    - Publish 4 new scans without TFs.
  //    - Then publish TF only for the latest of those 4.
  //    Expected behavior:
  //      * After the 4 invalid scans, buffer size grows only for those scans
  //        whose stamp is >= cutoff_stamp (the last valid stamp from step 4).
  //      * After adding TF for the latest scan and calling integrate_pending_perceptions(),
  //        only that latest scan remains in the buffer.
  //
  std::array<rclcpp::Time, 4> late_invalid_stamps;
  const auto base_stamp = perception->stamp;
  const std::size_t base_size_step5 = perception->buffer_size();  // should be 1
  const auto cutoff_ns_step5 = perception->stamp.nanoseconds();
  std::size_t kept_newer_or_equal = 0;

  for (int i = 0; i < 4; ++i) {
    rclcpp::Time ts = base_stamp + rclcpp::Duration(0, (i + 1) * 10'000'000);
    late_invalid_stamps[i] = ts;

    auto scan = get_scan_test_3(ts);
    laser_pub->publish(scan);
    exe.spin_some(std::chrono::milliseconds(50));

    // // std::cerr << "=== STEP 5: late invalid scan #" << i
    //           << " ts=" << ts.nanoseconds() << " ===\n";
    // perception->debug_print_buffer("Before flush after late invalid scan");

    perception->integrate_pending_perceptions();

    // perception->debug_print_buffer("After flush after late invalid scan");

    // Visible perception must remain the last valid one from step 4
    // (until there is a TF for one of these new scans).
    EXPECT_TRUE(perception->valid);
    EXPECT_EQ(perception->frame_id, sensor_frame);

    // Only scans with stamp >= cutoff_ns_step5 will remain in the buffer
    // after integrate_pending_perceptions(); older ones are dropped.
    if (ts.nanoseconds() >= cutoff_ns_step5) {
      ++kept_newer_or_equal;
    }

    std::size_t expected_size = base_size_step5 + (i + 1);
    EXPECT_EQ(perception->buffer_size(), expected_size);
  }

  // std::cerr << "=== STEP 5: before adding TF for the latest late invalid ===\n";
  // perception->debug_print_buffer("Before adding TF for latest late invalid");

  std::size_t expected_size_before_tf = base_size_step5 + 4u;
  EXPECT_EQ(perception->buffer_size(), expected_size_before_tf);

  // Add TF only for the latest of the 4 late invalid scans
  geometry_msgs::msg::TransformStamped tf_latest;
  tf_latest.header.stamp = late_invalid_stamps[3];
  tf_latest.header.frame_id = robot_frame;
  tf_latest.child_frame_id = sensor_frame;
  tf_latest.transform.translation.x = 0.0;
  tf_latest.transform.translation.y = 0.0;
  tf_latest.transform.translation.z = 0.0;
  tf_latest.transform.rotation.w = 1.0;

  // std::cerr << "=== STEP 5: adding TF for latest late invalid "
  //          << late_invalid_stamps[3].nanoseconds() << " ===\n";
  tf_buffer->setTransform(tf_latest, "test_authority", false);

  // perception->debug_print_buffer("Before flush after TF of latest late invalid");

  perception->integrate_pending_perceptions();

  // perception->debug_print_buffer("After flush after TF of latest late invalid");

  // Now the newest valid candidate is the latest late invalid,
  // and all older entries should be pruned, leaving exactly one entry.
  EXPECT_TRUE(perception->valid);
  EXPECT_EQ(perception->frame_id, sensor_frame);
  EXPECT_EQ(perception->stamp.nanoseconds(), late_invalid_stamps[3].nanoseconds());
  EXPECT_EQ(perception->buffer_size(), 1u);

  easynav::PointPerceptionBufferType final_latest;
  ASSERT_TRUE(perception->buffer_latest(final_latest));
  EXPECT_EQ(final_latest.stamp.nanoseconds(), late_invalid_stamps[3].nanoseconds());
  EXPECT_EQ(final_latest.frame, sensor_frame);

  // std::cerr << "=== STEP 5: end of test ===\n";
  // perception->debug_print_buffer("Final buffer state");
}


TEST_F(PerceptionsTestCase, PointPerceptionHandlerPC2Works)
{
  using easynav::RTTFBuffer;

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_pc2_handler_node");

  // Initialize RTTFBuffer singleton with this node clock
  auto tf_buffer = RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  const std::string robot_frame = tf_buffer->get_tf_info().robot_frame;
  const std::string sensor_frame = "lidar_frame";

  rclcpp::Time ts = node->get_clock()->now();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = ts;
  tf.header.frame_id = robot_frame;
  tf.child_frame_id = sensor_frame;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.w = 1.0;  // identity
  tf_buffer->setTransform(tf, "test_authority", false);

  auto handler = std::make_shared<easynav::PointPerceptionHandler>();
  auto perception = handler->create("lidar1");

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/test_pc2",
    "sensor_msgs/msg/PointCloud2",
    perception,
    cb_group);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test_pc2", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.header.frame_id = sensor_frame;
  pcl_cloud.width = 3;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;

  pcl_cloud.points.push_back(pcl::PointXYZ{1.0, 2.0, 3.0});
  pcl_cloud.points.push_back(pcl::PointXYZ{-1.5, 0.0, 0.5});
  pcl_cloud.points.push_back(pcl::PointXYZ{0.0, -2.0, -3.0});

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pcl_cloud, msg);
  msg.header.frame_id = sensor_frame;
  msg.header.stamp = ts;

  pub->publish(msg);

  auto start = node->now();
  while (node->now() - start < 100ms) {
    exe.spin_some();
  }

  auto loaded = std::dynamic_pointer_cast<easynav::PointPerception>(perception);
  ASSERT_NE(loaded, nullptr);
  ASSERT_TRUE(loaded->valid);
  ASSERT_TRUE(loaded->new_data);
  ASSERT_EQ(loaded->frame_id, sensor_frame);
  ASSERT_EQ(loaded->data.size(), 3u);

  ASSERT_NEAR(loaded->data[0].x, 1.0, 0.001);
  ASSERT_NEAR(loaded->data[0].y, 2.0, 0.001);
  ASSERT_NEAR(loaded->data[0].z, 3.0, 0.001);

  ASSERT_NEAR(loaded->data[1].x, -1.5, 0.001);
  ASSERT_NEAR(loaded->data[1].y, 0.0, 0.001);
  ASSERT_NEAR(loaded->data[1].z, 0.5, 0.001);

  ASSERT_NEAR(loaded->data[2].x, 0.0, 0.001);
  ASSERT_NEAR(loaded->data[2].y, -2.0, 0.001);
  ASSERT_NEAR(loaded->data[2].z, -3.0, 0.001);
}


TEST_F(PerceptionsTestCase, ImagePerceptionHandlerWorks)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_image_handler_node");

  auto handler = std::make_shared<easynav::ImagePerceptionHandler>();
  auto perception = handler->create("camera1");

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/test_image",
    "sensor_msgs/msg/Image",
    perception,
    cb_group);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub =
    node->create_publisher<sensor_msgs::msg::Image>(
    "/test_image", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());

  cv::Mat test_img(2, 3, CV_8UC3, cv::Scalar(10, 20, 30));
  test_img.at<cv::Vec3b>(0, 1) = cv::Vec3b(100, 110, 120);

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", test_img).toImageMsg();
  msg->header.frame_id = "camera_link";
  msg->header.stamp = node->now();

  image_pub->publish(*msg);

  auto start = node->now();
  while (node->now() - start < 100ms) {
    exe.spin_some();
  }

  auto loaded = std::dynamic_pointer_cast<easynav::ImagePerception>(perception);
  ASSERT_NE(loaded, nullptr);
  ASSERT_TRUE(loaded->valid);
  ASSERT_TRUE(loaded->new_data);
  ASSERT_EQ(loaded->frame_id, "camera_link");
  ASSERT_EQ(loaded->data.cols, 3);
  ASSERT_EQ(loaded->data.rows, 2);
  ASSERT_EQ(loaded->data.type(), CV_8UC3);
  ASSERT_EQ(loaded->data.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));
  ASSERT_EQ(loaded->data.at<cv::Vec3b>(0, 1), cv::Vec3b(100, 110, 120));
}

class DummyPerception : public easynav::PerceptionBase
{
public:
  std::string content;
};

class DummyHandler : public easynav::PerceptionHandler
{
public:
  std::string group() const override {return "dummy";}

  std::shared_ptr<easynav::PerceptionBase> create(const std::string &) override
  {
    auto p = std::make_shared<DummyPerception>();
    p->content = "initialized";
    return p;
  }

  rclcpp::SubscriptionBase::SharedPtr create_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic,
    [[maybe_unused]] const std::string & type,
    std::shared_ptr<easynav::PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group) override
  {
    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = cb_group;

    return node.create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1),
      [target](const std_msgs::msg::String::SharedPtr msg)
      {
        auto p = std::dynamic_pointer_cast<DummyPerception>(target);
        p->content = msg->data;
        p->valid = true;
        p->new_data = true;
      },
      options);
  }
};

TEST_F(PerceptionsTestCase, CustomPerceptionHandlerCanBeRegistered)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_custom_handler_node");

  auto handler = std::make_shared<DummyHandler>();
  auto perception = handler->create("dummy1");

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/dummy_topic",
    "std_msgs/msg/String",
    perception,
    cb_group);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub =
    node->create_publisher<std_msgs::msg::String>(
    "/dummy_topic", rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());

  std_msgs::msg::String msg;
  msg.data = "HelloWorld";
  pub->publish(msg);

  auto start = node->now();
  while (node->now() - start < 100ms) {
    exe.spin_some();
  }

  auto loaded = std::dynamic_pointer_cast<DummyPerception>(perception);
  ASSERT_NE(loaded, nullptr);
  ASSERT_EQ(loaded->content, "HelloWorld");
  ASSERT_TRUE(loaded->valid);
}
