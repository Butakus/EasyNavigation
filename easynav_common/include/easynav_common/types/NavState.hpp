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
/// \brief A blackboard-like structure to hold the current state of the navigation system.

#ifndef EASYNAV_COMMON_TYPES__NAVSTATE_HPP_
#define EASYNAV_COMMON_TYPES__NAVSTATE_HPP_

#include <string>
#include <map>
#include <any>
#include <typeinfo>
#include <stdexcept>
#include <sstream>
#include <execinfo.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>

#include "rclcpp/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/MapTypeBase.hpp"

namespace easynav
{

/**
 * @brief A typed key-value store for sharing navigation-related state.
 *
 * Provides safe typed access to stored values using std::any.
 */
class NavState
{
public:
  /**
   * @brief Stores a value of any type under the given key.
   *
   * @tparam T Type of the value.
   * @param key Identifier to store the value under.
   * @param value The value to store.
   */
  template<typename T>
  void set(const std::string & key, const T & value)
  {
    data_[key] = value;
  }

  /**
   * @brief Retrieves a copy of the value stored under the given key.
   *
   * @tparam T Expected type of the value.
   * @param key Key of the value.
   * @return T Copy of the stored value.
   * @throws std::bad_cast if the stored type does not match T.
   * @throws std::out_of_range if the key is not found.
   */
  template<typename T>
  T get(const std::string & key) const
  {
    const auto & any_ref = get_raw(key);
    try {
      return std::any_cast<T>(any_ref);
    } catch (const std::bad_any_cast &) {
      std::cerr << "[NavState] std::bad_cast in get<" << typeid(T).name() << ">(\"" <<
        key << "\")\n";
      print_stacktrace();
      throw std::bad_cast();
    }
  }

  /**
   * @brief Retrieves a mutable reference to the stored value.
   *
   * @tparam T Expected type of the value.
   * @param key Key of the value.
   * @return T& Reference to the stored value.
   * @throws std::bad_cast if the stored type does not match T.
   * @throws std::out_of_range if the key is not found.
   */
  template<typename T>
  T & get_mutable(const std::string & key)
  {
    auto & any_ref = get_raw_mutable(key);
    T * ptr = std::any_cast<T>(&any_ref);
    if (!ptr) {
      std::cerr << "[NavState] std::bad_cast in get_mutable<" << typeid(T).name() << ">(\"" << 
        key << "\")\n";
      print_stacktrace();
      throw std::bad_cast();
    }
    return *ptr;
  }

  /**
   * @brief Retrieves a const reference to the stored value (no copy).
   *
   * @tparam T Expected type of the value.
   * @param key Key of the value.
   * @return const T& Const reference to the stored value.
   * @throws std::bad_cast if the stored type does not match T.
   * @throws std::out_of_range if the key is not found.
   */
  template<typename T>
  const T & get_ref(const std::string & key) const
  {
    const auto & any_ref = get_raw(key);
    const T * ptr = std::any_cast<T>(&any_ref);
    if (!ptr) {
      std::cerr << "[NavState] std::bad_cast in get_ref<" << typeid(T).name() << ">(\"" <<
        key << "\")\n";
      print_stacktrace();
      throw std::bad_cast();
    }
    return *ptr;
  }

  /**
   * @brief Checks whether a key exists.
   */
  bool has(const std::string & key) const
  {
    return data_.find(key) != data_.end();
  }

  /**
   * @brief Removes all entries from the state.
   */
  void clear()
  {
    data_.clear();
  }

  /**
   * @brief Registers a printer function for a specific type T.
   */
  template<typename T>
  static void register_printer(std::function<std::string(const T &)> printer)
  {
    auto wrapper = [printer](const std::any & val) -> std::string {
      return printer(std::any_cast<const T &>(val));
    };
    printers_[std::type_index(typeid(T))] = wrapper;
  }

  /**
   * @brief Returns a debug string of the entire NavState content.
   */
  std::string debug_string() const
  {
    std::ostringstream out;
    for (const auto & [key, val] : data_) {
      out << key << ": ";
      auto it = printers_.find(std::type_index(val.type()));
      if (it != printers_.end()) {
        try {
          out << it->second(val);
        } catch (...) {
          out << "<error printing value>";
        }
      } else {
        out << "<" << val.type().name() << ">";
      }
      out << "\n";
    }
    return out.str();
  }
private:
  /**
   * @brief Internal access to the std::any associated with a key.
   *
   * @param key Key to look up.
   * @return const std::any& Reference to the stored any object.
   * @throws std::out_of_range if the key is not found.
   */
  const std::any & get_raw(const std::string & key) const
  {
    auto it = data_.find(key);
    if (it == data_.end()) {
      throw std::out_of_range("Key not found in NavState: " + key);
    }
    return it->second;
  }

  /**
   * @brief Internal mutable access to the std::any associated with a key.
   *
   * @param key Key to look up.
   * @return std::any& Reference to the stored any object.
   * @throws std::out_of_range if the key is not found.
   */
  std::any & get_raw_mutable(const std::string & key)
  {
    auto it = data_.find(key);
    if (it == data_.end()) {
      throw std::out_of_range("Key not found in NavState: " + key);
    }
    return it->second;
  }

  void print_stacktrace(std::ostream & out = std::cerr, int max_frames = 64) const
  {
    void * addrlist[max_frames + 1];
    int addrlen = backtrace(addrlist, max_frames);

    if (addrlen == 0) {
      out << "  <empty stack>\n";
      return;
    }

    char ** symbols = backtrace_symbols(addrlist, addrlen);
    out << "  Stack trace:\n";
    for (int i = 1; i < addrlen; ++i) {
      out << "    " << symbols[i] << "\n";
    }
    free(symbols);
  }

  std::map<std::string, std::any> data_;

  using AnyPrinter = std::function<std::string(const std::any &)>;
  static inline std::map<std::type_index, AnyPrinter> printers_;
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__NAVSTATE_HPP_
