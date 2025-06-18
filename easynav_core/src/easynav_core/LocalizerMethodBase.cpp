// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

/// \file
/// \brief Implementation of the abstract base class LocalizerMethodBase.

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/MethodBase.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"

namespace easynav
{

bool
LocalizerMethodBase::internal_update_rt(NavState & nav_state, bool trigger)
{
  if (isTime2RunRT() || trigger) {
    EASYNAV_TRACE_EVENT;

    update_rt(nav_state);
    return true;
  } else {
    return false;
  }
}

void
LocalizerMethodBase::internal_update(NavState & nav_state)
{
  if (isTime2Run()) {
    EASYNAV_TRACE_EVENT;

    update(nav_state);
  }
}

}  // namespace easynav
