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
///
/// This file defines the NavState class, which provides a lock-free key-value store
/// where values can be of any type and stored/retrieved via smart pointers.
/// It is designed for concurrent, type-safe access in robotics applications.

#ifndef EASYNAV__TYPES__NAVSTATE_HPP_
#define EASYNAV__TYPES__NAVSTATE_HPP_

#include <string>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <type_traits>
#include <iostream>
#include <functional>
#include <execinfo.h>
#include <typeinfo>

namespace easynav
{


/// \class NavState
/// \brief A generic, type-safe, lock-free blackboard to hold runtime state.
///
/// NavState provides:
/// - Type-erased storage using `std::shared_ptr<void>`.
/// - Runtime type verification and safe casting via `typeid`.
/// - Support for raw, shared, and copy-based insertion.
/// - Debug utilities including stack trace and introspection.
///
/// Example usage:
/// ```cpp
/// NavState state;
/// state.set("goal_reached", false);
/// bool reached = state.get<bool>("goal_reached");
/// ```
class NavState
{
public:
  /// \brief Constructs an empty NavState and registers basic type printers.
  NavState()
  {
    register_basic_printers();
  }

  /// \brief Destructor.
  virtual ~NavState() = default;

  /// \brief Stores a copy of a value in the blackboard.
  /// \tparam T The type of the value (must be copy-constructible).
  /// \param key The string identifier.
  /// \param value The value to copy and store.
  template<typename T>
  void set(const std::string & key, const T & value)
  {
    static_assert(std::is_copy_constructible_v<T>, "T must be copy constructible");
    auto ptr = std::make_shared<T>(value);
    std::atomic_store(&values_[key], std::static_pointer_cast<void>(ptr));
    types_[key] = typeid(T).hash_code();
  }

  /// \brief Stores a raw pointer without taking ownership.
  ///
  /// The destructor is a no-op. Use only for externally managed memory.
  ///
  /// \tparam T Type of the pointed object.
  /// \param key Key to store under.
  /// \param raw_ptr Raw pointer to the value (must not be null).
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

  /// \brief Stores a shared pointer.
  ///
  /// This variant takes ownership and ensures atomic safety.
  ///
  /// \tparam T The stored type.
  /// \param key Key for the value.
  /// \param shared Valid shared pointer to the value.
  template<typename T>
  void set_shared_ptr(const std::string & key, std::shared_ptr<T> shared)
  {
    if (!shared) {
      throw std::invalid_argument("Cannot store nullptr shared_ptr in set_shared_ptr");
    }
    std::atomic_store(&values_[key], std::static_pointer_cast<void>(shared));
    types_[key] = typeid(T).hash_code();
  }

  /// \brief Returns a copy of the value stored under the key.
  ///
  /// \tparam T Expected type of the value.
  /// \param key Key to look up.
  /// \return A copy of the stored value.
  /// \throws std::bad_cast or std::out_of_range on type/key mismatch.
  template<typename T>
  T get(const std::string & key) const
  {
    return *get_ptr<T>(key);
  }

  /// \brief Returns a const reference to the value stored under the key.
  ///
  /// This function is extremelly dangerous. Use only if you are sure it will not change
  ///
  /// \tparam T Expected type.
  /// \param key Lookup key.
  /// \return Const reference to the stored value.
  template<typename T>
  const T & get_ref(const std::string & key) const
  {
    return *get_ptr<T>(key);
  }

  /// \brief Returns a shared pointer to the stored value.
  ///
  /// \tparam T Expected type.
  /// \param key Lookup key.
  /// \return Shared pointer to the object.
  /// \throws std::runtime_error or std::out_of_range if invalid or not found.
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

  /// \brief Checks whether a key exists in the NavState.
  /// \param key Lookup key.
  /// \return True if the key is registered.
  bool has(const std::string & key) const
  {
    return values_.find(key) != values_.end();
  }

  /// \brief Type alias for a generic printer function.
  ///
  /// Used to print debug output for stored values.
  using AnyPrinter = std::function<std::string(std::shared_ptr<void>)>;

  /// \brief Registers a printer for a given type.
  ///
  /// The function will be used to convert values of this type into strings
  /// for use in `debug_string()`.
  ///
  /// \tparam T Type to register.
  /// \param printer Function that converts a const reference to string.
  template<typename T>
  static void register_printer(std::function<std::string(const T &)> printer)
  {
    auto wrapper = [printer](std::shared_ptr<void> base_ptr) -> std::string {
        auto typed_ptr = std::static_pointer_cast<T>(base_ptr);
        return printer(*typed_ptr);
      };
    type_printers_[typeid(T).hash_code()] = wrapper;
  }

  /// \brief Dumps all keys and their values to a formatted string.
  ///
  /// If a printer is registered for a given type, it is used;
  /// otherwise, the raw pointer address and hash are shown.
  ///
  /// \return String representation of current state.
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
            ss << "[" << ptr.get() << "] : " << printer_it->second(ptr);
          } else {
            ss << "[" << ptr.get() << "] : " << type_it->second << "]";
          }
        } else {
          ss << "[" << ptr.get() << "] : unknown]";
        }
      } else {
        ss << "[null]";
      }
      ss << std::endl;
    }
    return ss.str();
  }

  /// \brief Prints the current C++ stack trace to standard error.
  ///
  /// Used to assist debugging in exception contexts.
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

  /// \brief Registers default string printers for basic types:
  /// `int`, `float`, `double`, `std::string`, `bool`, `char`.
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
  /// \brief Internal storage of values as shared void pointers.
  mutable std::unordered_map<std::string, std::shared_ptr<void>> values_;

  /// \brief Stores typeid hashes for each key.
  mutable std::unordered_map<std::string, size_t> types_;

  /// \brief Maps typeid hashes to printable string renderers.
  static inline std::unordered_map<size_t, AnyPrinter> type_printers_;
};

}  // namespace easynav

#endif  // EASYNAV__TYPES__NAVSTATE_HPP_
