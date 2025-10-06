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
/// This file defines the NavState class, which provides a thread-safe key-value store
/// where values can be of any type and stored/retrieved via smart pointers.
/// It is designed for concurrent, type-safe access in robotics applications.

#ifndef EASYNAV__TYPES__NAVSTATE_HPP_
#define EASYNAV__TYPES__NAVSTATE_HPP_

#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <type_traits>
#include <iostream>
#include <functional>
#include <execinfo.h>
#include <typeinfo>
#include <cxxabi.h>
#include <execinfo.h>

namespace easynav
{

/// \brief Demangles a C++ RTTI type name if possible.
/// \param name Mangled type name (from \c typeid(T).name()).
/// \return Readable (demangled) type name if available; otherwise returns \p name.
inline std::string demangle(const char * name)
{
  int status = 0;
  char * p = abi::__cxa_demangle(name, nullptr, nullptr, &status);
  std::string out = (status == 0 && p) ? p : name;
  std::free(p);
  return out;
}

/// \brief Captures a C/C++ stack trace as a string.
/// \param skip Number of initial frames to skip (e.g., the \c stacktrace frame itself).
/// \param max_frames Maximum number of frames to capture.
/// \return A multi-line string with one frame per line.
inline std::string stacktrace(std::size_t skip = 1, std::size_t max_frames = 64)
{
  void * buf[128];
  const int n = backtrace(buf, static_cast<int>(std::min<std::size_t>(max_frames, 128)));
  char ** syms = backtrace_symbols(buf, n);
  std::ostringstream oss;
  for (int i = static_cast<int>(skip); i < n; ++i) {
    oss << "#" << (i - static_cast<int>(skip)) << " " << syms[i] << "\n";
  }
  std::free(syms);
  return oss.str();
}

/// \class NavState
/// \brief A generic, type-safe, thread-safe blackboard to hold runtime state.
///
/// NavState provides:
/// - Type-erased storage using \c std::shared_ptr<void>.
/// - Runtime type verification and safe casting via \c typeid.
/// - Support for value-based and shared-pointer-based insertion.
/// - Debug utilities including stack trace and introspection.
///
/// \note Thread-safety is enforced with an internal \c std::mutex.
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

  /// \brief Stores a value of type \p T associated with \p key (by copy).
  ///
  /// If \p key does not exist, a new \c std::shared_ptr<T> is created and stored.
  /// If \p key exists, the stored value is overwritten in place.
  ///
  /// \tparam T Value type. Must be copy-assignable.
  /// \param key Key associated with the value.
  /// \param value Value to store (copied into internal storage).
  /// \throws std::runtime_error If \p key exists with a different stored type; includes a stack trace.
  template<typename T>
  void set(const std::string & key, const T & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      values_[key] = std::make_shared<T>(value);
      types_[key] = typeid(T).hash_code();
    } else {
      if (types_[key] != typeid(T).hash_code()) {
        std::ostringstream oss;
        oss << "Type mismatch in set(\"" << key << "\")\n"
            << "  expected(hash): " << types_[key] << "\n"
            << "  provided     : " << demangle(typeid(T).name()) << "\n"
            << "Backtrace:\n" << stacktrace(1);
        throw std::runtime_error(oss.str());
      }

      auto ptr = std::static_pointer_cast<T>(it->second);
      *ptr = value;
    }
  }

  /// \brief Stores a value of type \p T associated with \p key (by shared pointer).
  ///
  /// If \p key does not exist, \p value_ptr is stored directly.
  /// If \p key exists, the stored \c std::shared_ptr<T> is replaced by \p value_ptr.
  ///
  /// \tparam T Value type.
  /// \param key Key associated with the value.
  /// \param value_ptr Shared pointer to the value to store.
  /// \throws std::runtime_error If \p key exists with a different stored type; includes a stack trace.
  template<typename T>
  void set(const std::string & key, const std::shared_ptr<T> value_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      values_[key] = std::shared_ptr<T>(value_ptr);
      types_[key] = typeid(T).hash_code();
    } else {
      if (types_[key] != typeid(T).hash_code()) {
        std::ostringstream oss;
        oss << "Type mismatch in set(\"" << key << "\")\n"
            << "  expected(hash): " << types_[key] << "\n"
            << "  provided     : " << demangle(typeid(T).name()) << "\n"
            << "Backtrace:\n" << stacktrace(1);
        throw std::runtime_error(oss.str());
      }

      auto ptr = std::static_pointer_cast<T>(it->second);
      ptr = std::shared_ptr<T>(value_ptr);
    }
  }

  /// \brief Retrieves a const reference to the stored value of type \p T for \p key.
  ///
  /// The reference refers to the object managed by the internal \c std::shared_ptr<T>.
  ///
  /// \tparam T Expected stored type.
  /// \param key Key to retrieve.
  /// \return Const reference to the stored \p T.
  /// \throws std::runtime_error If \p key is missing or the stored type does not match \p T.
  template<typename T>
  const T & get(const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      throw std::runtime_error("Key not found in get: " + key);
    }

    if (types_.at(key) != typeid(T).hash_code()) {
      throw std::runtime_error("Type mismatch in get for key: " + key);
    }

    auto ptr = std::static_pointer_cast<T>(it->second);
    return *ptr;
  }

  /// \brief Retrieves the shared_ptr to the stored value of type \p T for \p key.
  ///
  /// The pointer refers to the object managed by the internal \c std::shared_ptr<T>.
  ///
  /// \tparam T Expected stored type.
  /// \param key Key to retrieve.
  /// \return shared_ptr to the stored \p T.
  /// \throws std::runtime_error If \p key is missing or the stored type does not match \p T.
  template<typename T>
  const std::shared_ptr<T> get_ptr(const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      throw std::runtime_error("Key not found in get: " + key);
    }

    if (types_.at(key) != typeid(T).hash_code()) {
      throw std::runtime_error("Type mismatch in get for key: " + key);
    }

    return std::static_pointer_cast<T>(it->second);
  }

  /// \brief Checks whether \p key exists in the state.
  /// \param key Key to query.
  /// \return \c true if present, otherwise \c false.
  bool has(const std::string & key) const
  {
    return values_.find(key) != values_.end();
  }

  /// \brief Type alias for a generic printer functor used by \ref debug_string().
  ///
  /// The functor receives the stored value as a \c std::shared_ptr<void>
  /// and returns a string representation.
  using AnyPrinter = std::function<std::string(std::shared_ptr<void>)>;

  /// \brief Registers a pretty-printer for type \p T used by \ref debug_string().
  ///
  /// \tparam T Type to register.
  /// \param printer Functor that renders \c const T& to a string.
  template<typename T>
  static void register_printer(std::function<std::string(const T &)> printer)
  {
    auto wrapper = [printer](std::shared_ptr<void> base_ptr) -> std::string {
        auto typed_ptr = std::static_pointer_cast<T>(base_ptr);
        return printer(*typed_ptr);
      };
    type_printers_[typeid(T).hash_code()] = wrapper;
  }

  /// \brief Generates a human-readable dump of all stored keys and values.
  ///
  /// For types with registered printers, their printer is used; otherwise, the raw pointer
  /// address and type hash are shown.
  /// \return Multi-line string with one entry per key.
  std::string debug_string() const
  {
    std::stringstream ss;
    for (const auto & kv : values_) {
      ss << kv.first << " = ";
      auto ptr = kv.second;
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

  /// \brief Prints the current C++ stack trace to \c std::cerr.
  /// \note Intended for debugging in exception contexts.
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

  /// \brief Registers default printers for common scalar types.
  ///
  /// Registers printers for: \c int, \c float, \c double, \c std::string, \c bool, \c char.
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
  mutable std::mutex mutex_;  ///< Guards access to \ref values_ and \ref types_.

  /// \brief Internal storage of values as type-erased shared pointers.
  mutable std::unordered_map<std::string, std::shared_ptr<void>> values_;

  /// \brief Stored type hash (from \c typeid(T).hash_code()) per key.
  mutable std::unordered_map<std::string, size_t> types_;

  /// \brief Registry of type-hash → printer functors used by \ref debug_string().
  static inline std::unordered_map<size_t, AnyPrinter> type_printers_;
};

}  // namespace easynav

#endif  // EASYNAV__TYPES__NAVSTATE_HPP_
