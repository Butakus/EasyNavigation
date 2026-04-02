// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <thread>
#include <atomic>
#include <vector>

#include "gtest/gtest.h"

#include "easynav_common/types/NavState.hpp"
#include "geometry_msgs/msg/pose.hpp"

class NavStateTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(NavStateTest, SetAndGet)
{
  easynav::NavState state;
  state.set("name", std::string("robot"));
  state.set("count", 42);
  EXPECT_EQ(state.get<std::string>("name"), "robot");
  EXPECT_EQ(state.get<int>("count"), 42);
}

TEST_F(NavStateTest, OverwriteEntry)
{
  easynav::NavState state;
  state.set("x", 10);
  state.set("x", 20);
  EXPECT_EQ(state.get<int>("x"), 20);
}

TEST_F(NavStateTest, HasReturnsCorrect)
{
  easynav::NavState state;
  EXPECT_FALSE(state.has("missing"));
  state.set("exists", 1.0);
  EXPECT_TRUE(state.has("exists"));
}

TEST_F(NavStateTest, MissingKeyThrows)
{
  easynav::NavState state;
  EXPECT_THROW(state.get<float>("invalid"), std::runtime_error);
}

TEST_F(NavStateTest, DebugStringWithPosePrinter)
{
  easynav::NavState state;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.w = 1.0;

  state.set("pose", pose);
  state.set<int>("age", 10);

  easynav::NavState::register_printer<geometry_msgs::msg::Pose>(
    [](const geometry_msgs::msg::Pose & p) {
      std::ostringstream oss;
      oss << "Position: (" << p.position.x << ", " << p.position.y << ", " << p.position.z
          << ") Orientation: (" << p.orientation.x << ", " << p.orientation.y << ", "
          << p.orientation.z << ", " << p.orientation.w << ")";
      return oss.str();
    });

  std::string output = state.debug_string();

  std::cerr << output << std::endl;

  EXPECT_NE(output.find("Position: (1, 2, 3)"), std::string::npos);
  EXPECT_NE(output.find("Orientation:"), std::string::npos);
  EXPECT_NE(output.find(": 10"), std::string::npos);
}

TEST(NavStateStressTest, ConcurrentMultiKeyReadWrite)
{
  easynav::NavState state;
  std::atomic<bool> start_flag{false};

  state.set<int>("int_key", 0);
  state.set<double>("double_key", 0.0);
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.orientation.w = 1.0;
  state.set("pose_key", pose);

  auto writer = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      for (int i = 0; i < 10000; ++i) {
        state.set<int>("int_key", i);
        state.set<double>("double_key", static_cast<double>(i) / 2.0);
        geometry_msgs::msg::Pose p;
        p.position.x = static_cast<double>(i);
        p.orientation.w = 1.0;
        state.set("pose_key", p);
      }
    };

  auto reader = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      for (int i = 0; i < 10000; ++i) {
        int vi = state.get<int>("int_key");
        double vd = state.get<double>("double_key");
        geometry_msgs::msg::Pose vp = state.get<geometry_msgs::msg::Pose>("pose_key");

        ASSERT_GE(vi, 0);
        ASSERT_GE(vd, 0.0);
        ASSERT_EQ(vp.orientation.w, 1.0);
      }
    };

  std::vector<std::thread> threads;
  for (int i = 0; i < 3; ++i) {
    threads.emplace_back(writer);
  }
  for (int i = 0; i < 3; ++i) {
    threads.emplace_back(reader);
  }

  start_flag.store(true);

  for (auto & t : threads) {
    t.join();
  }
  SUCCEED();
}

// ─────────────────────────────────────────────────────────────────────────────
// std::vector<std::string> printer (added in register_basic_printers)
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(NavStateTest, VectorStringPrinterEmptyVector)
{
  easynav::NavState state;
  state.set("keys", std::vector<std::string>{});
  std::string s = state.debug_string();
  EXPECT_NE(s.find("[]"), std::string::npos) << "Empty vector must render as []\n" << s;
}

TEST_F(NavStateTest, VectorStringPrinterSingleElement)
{
  easynav::NavState state;
  state.set("keys", std::vector<std::string>{"sensor_a"});
  std::string s = state.debug_string();
  EXPECT_NE(s.find("[sensor_a]"), std::string::npos)
    << "Single element must render as [sensor_a]\n" << s;
}

TEST_F(NavStateTest, VectorStringPrinterMultipleElements)
{
  easynav::NavState state;
  state.set("keys", std::vector<std::string>{"sensor_a", "sensor_b", "sensor_c"});
  std::string s = state.debug_string();
  EXPECT_NE(s.find("[sensor_a, sensor_b, sensor_c]"), std::string::npos)
    << "Multiple elements must render comma-separated\n" << s;
}

TEST_F(NavStateTest, SetGroupIsVisibleInDebugString)
{
  easynav::NavState state;
  state.set_group("points", {"lidar_front", "lidar_back"});
  std::string s = state.debug_string();
  EXPECT_NE(s.find("lidar_front"), std::string::npos)
    << "Group member must appear in debug_string\n" << s;
  EXPECT_NE(s.find("lidar_back"), std::string::npos)
    << "Group member must appear in debug_string\n" << s;
}
