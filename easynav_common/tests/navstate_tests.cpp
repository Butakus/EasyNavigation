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
