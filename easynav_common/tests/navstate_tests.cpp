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

#include "gtest/gtest.h"

#include "easynav_common/types/NavState.hpp"

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


TEST_F(NavStateTest, GetRef)
{
  easynav::NavState state;
  state.set("value", std::string("easy"));

  const auto & ref = state.get_ref<std::string>("value");
  EXPECT_EQ(ref, "easy");
}


TEST_F(NavStateTest, GetMutable)
{
  easynav::NavState state;
  state.set("x", 3.14f);

  float & val = state.get_mutable<float>("x");
  val += 1.0f;

  EXPECT_FLOAT_EQ(state.get<float>("x"), 4.14f);
}


TEST_F(NavStateTest, HasAndClear)
{
  easynav::NavState state;
  state.set("exists", true);

  EXPECT_TRUE(state.has("exists"));
  EXPECT_FALSE(state.has("missing"));

  state.clear();
  EXPECT_FALSE(state.has("exists"));
}


TEST_F(NavStateTest, GetMissingThrows)
{
  easynav::NavState state;

  EXPECT_THROW(state.get<int>("not_there"), std::out_of_range);
  EXPECT_THROW(state.get_ref<int>("not_there"), std::out_of_range);
  EXPECT_THROW(state.get_mutable<int>("not_there"), std::out_of_range);
}


TEST_F(NavStateTest, GetBadCastThrows)
{
  easynav::NavState state;
  state.set("str", std::string("hello"));

  EXPECT_THROW(state.get<int>("str"), std::bad_cast);
  EXPECT_THROW(state.get_ref<int>("str"), std::bad_cast);
  EXPECT_THROW(state.get_mutable<int>("str"), std::bad_cast);
}


TEST_F(NavStateTest, DebugStringWorks)
{
  easynav::NavState state;
  state.set("a", 123);
  state.set("b", std::string("abc"));

  easynav::NavState::register_printer<int>([](const int & val) {
    return std::to_string(val);
  });

  easynav::NavState::register_printer<std::string>([](const std::string & val) {
    return "\"" + val + "\"";
  });

  std::string debug = state.debug_string();

  EXPECT_NE(debug.find("a: 123"), std::string::npos);
  EXPECT_NE(debug.find("b: \"abc\""), std::string::npos);
}
