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


#ifndef EASYNAV__TYPES__NAVSTATE_HPP_
#define EASYNAV__TYPES__NAVSTATE_HPP_

#include <string>
#include <unordered_map>
#include <memory>
#include <atomic>
#include <stdexcept>
#include <sstream>
#include <type_traits>
#include <iostream>
#include <functional>
#include <execinfo.h>
#include <typeinfo>

namespace easynav
{

class NavState
{
public:
  NavState()
  {
    register_basic_printers();
  }
  virtual ~NavState() = default;

  template<typename T>
  void set(const std::string & key, const T & value)
  {
    static_assert(std::is_copy_constructible_v<T>, "T must be copy constructible");
    auto ptr = std::make_shared<T>(value);
    std::atomic_store(&values_[key], std::static_pointer_cast<void>(ptr));
    types_[key] = typeid(T).hash_code();
  }

  template<typename T>
  void set_ptr(const std::string & key, T * raw_ptr)
  {
    if (raw_ptr == nullptr) {
      throw std::invalid_argument("Cannot store nullptr in set_ptr");
    }
    std::shared_ptr<T> shared(raw_ptr, [](T *){});
    std::atomic_store(&values_[key], std::static_pointer_cast<void>(shared));
    types_[key] = typeid(T).hash_code();
  }

  template<typename T>
  void set_shared_ptr(const std::string & key, std::shared_ptr<T> shared)
  {
    if (!shared) {
      throw std::invalid_argument("Cannot store nullptr shared_ptr in set_shared_ptr");
    }
    std::atomic_store(&values_[key], std::static_pointer_cast<void>(shared));
    types_[key] = typeid(T).hash_code();
  }

  template<typename T>
  T get(const std::string & key) const
  {
    return *get_ptr<T>(key);
  }

  template<typename T>
  const T & get_ref(const std::string & key) const
  {
    return *get_ptr<T>(key);
  }

  template<typename T>
  std::shared_ptr<T> get_ptr(const std::string & key) const
  {
    auto it = values_.find(key);
    if (it == values_.end()) {
      print_stacktrace();
      throw std::out_of_range("Key not found: " + key);
    }

    auto base_ptr = std::atomic_load(&it->second);
    if (!base_ptr) {
      print_stacktrace();
      throw std::runtime_error("Null pointer in NavState at key: " + key);
    }

    return std::static_pointer_cast<T>(base_ptr);
  }

  bool has(const std::string & key) const
  {
    return values_.find(key) != values_.end();
  }

  using AnyPrinter = std::function<std::string(std::shared_ptr<void>)>;

  template<typename T>
  static void register_printer(std::function<std::string(const T &)> printer)
  {
    auto wrapper = [printer](std::shared_ptr<void> base_ptr) -> std::string {
        auto typed_ptr = std::static_pointer_cast<T>(base_ptr);
        return printer(*typed_ptr);
      };
    type_printers_[typeid(T).hash_code()] = wrapper;
  }

  std::string debug_string() const
  {
    std::stringstream ss;
    for (const auto & kv : values_) {
      ss << kv.first << " = ";
      auto ptr = std::atomic_load(&kv.second);
      if (ptr) {
        auto type_it = types_.find(kv.first);
        if (type_it != types_.end()) {
          auto printer_it = type_printers_.find(type_it->second);
          if (printer_it != type_printers_.end()) {
            ss << printer_it->second(ptr);
          } else {
            ss << "[" << ptr.get() << " : " << type_it->second << "]";
          }
        } else {
          ss << "[" << ptr.get() << " : unknown]";
        }
      } else {
        ss << "[null]";
      }
      ss << std::endl;
    }
    return ss.str();
  }

  static void print_stacktrace()
  {
    void *array[50];
    int size = backtrace(array, 50);
    char **strings = backtrace_symbols(array, size);
    std::cerr << "\nStack trace:\n";
    for (int i = 0; i < size; ++i) {
      std::cerr << strings[i] << std::endl;
    }
    std::cerr << std::endl;
    free(strings);
  }

  static void register_basic_printers()
  {
    register_printer<int>([](const int & v) {return std::to_string(v);});
    register_printer<float>([](const float & v) {return std::to_string(v);});
    register_printer<double>([](const double & v) {return std::to_string(v);});
    register_printer<std::string>([](const std::string & v) {return v;});
    register_printer<bool>([](const bool & v) {return v ? "true" : "false";});
    register_printer<char>([](const char & v) {return std::string(1, v);});
  }

private:
  mutable std::unordered_map<std::string, std::atomic<std::shared_ptr<void>>> values_;
  mutable std::unordered_map<std::string, size_t> types_;
  static inline std::unordered_map<size_t, AnyPrinter> type_printers_;
};

}  // namespace easynav

#endif  // EASYNAV__TYPES__NAVSTATE_HPP_
