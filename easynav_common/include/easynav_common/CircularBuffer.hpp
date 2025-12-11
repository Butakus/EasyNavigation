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

#ifndef EASYNAV_COMMON__CIRCULARBUFFER_HPP_
#define EASYNAV_COMMON__CIRCULARBUFFER_HPP_

#include <vector>
#include <mutex>
#include <cstddef>

namespace easynav
{

/// \brief Fixed-size circular buffer, thread-safe (mutex-based), copyable.
///
/// All storage is allocated in the constructor. Push and pop never allocate.
/// Copying the buffer creates a new internal storage and copies the content.
/// This class is safe for multiple producers and consumers, but access is
/// serialized by a single mutex.
template<typename T>
class CircularBuffer
{
public:
  /// \brief Construct a buffer with the given capacity.
  explicit CircularBuffer(std::size_t capacity)
  : buffer_(capacity),
    capacity_(capacity),
    head_(0),
    tail_(0),
    size_(0)
  {
  }

  /// \brief Copy constructor.
  ///
  /// Creates a new buffer with the same capacity and copies all elements and
  /// index state. The internal mutex is not shared.
  CircularBuffer(const CircularBuffer & other)
  : buffer_(other.capacity_),
    capacity_(other.capacity_),
    head_(0),
    tail_(0),
    size_(0)
  {
    std::lock_guard<std::mutex> lock(other.mutex_);
    buffer_ = other.buffer_;
    head_ = other.head_;
    tail_ = other.tail_;
    size_ = other.size_;
  }

  /// \brief Copy assignment.
  CircularBuffer & operator=(const CircularBuffer & other)
  {
    if (this == &other) {
      return *this;
    }

    // Lock both mutexes in a deadlock-safe way.
    std::scoped_lock<std::mutex, std::mutex> lock(mutex_, other.mutex_);

    capacity_ = other.capacity_;
    buffer_ = other.buffer_;
    head_ = other.head_;
    tail_ = other.tail_;
    size_ = other.size_;
    return *this;
  }

  /// \brief Move operations are optional. For now we delete them to avoid
  /// surprising semantics.
  CircularBuffer(CircularBuffer &&) = delete;
  CircularBuffer & operator=(CircularBuffer &&) = delete;

  /// \brief Maximum number of elements that can be stored.
  std::size_t capacity() const noexcept
  {
    return capacity_;
  }

  /// \brief Current number of valid elements.
  std::size_t size() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_;
  }

  /// \brief True if the buffer is empty.
  bool empty() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_ == 0;
  }

  /// \brief True if the buffer is full.
  bool full() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_ == capacity_;
  }

  /// \brief Remove all elements, keeping the allocated storage.
  void clear() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = 0;
    tail_ = 0;
    size_ = 0;
  }

  /// \brief Push a new element (copy). Overwrites the oldest if the buffer is full.
  void push(const T & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[head_] = value;
    advance_head_();
  }

  /// \brief Push a new element (move). Overwrites the oldest if the buffer is full.
  void push(T && value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[head_] = std::move(value);
    advance_head_();
  }

  /// \brief Pop the oldest element.
  ///
  /// \param out Destination for the oldest element.
  /// \return False if the buffer is empty.
  bool pop(T & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (size_ == 0) {
      return false;
    }

    out = buffer_[tail_];
    tail_ = (tail_ + 1) % capacity_;
    --size_;
    return true;
  }

  /// \brief Get a copy of the newest element without removing it.
  ///
  /// \param out Destination for the newest element.
  /// \return False if the buffer is empty.
  bool latest(T & out) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (size_ == 0) {
      return false;
    }

    const std::size_t newest_index =
      (head_ + capacity_ - 1) % capacity_;
    out = buffer_[newest_index];
    return true;
  }

  /// \brief Get a const reference to the newest element without removing it.
  const T & latest_ref() const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const std::size_t newest_index =
      (head_ + capacity_ - 1) % capacity_;
    return buffer_[newest_index];
  }

  // DEBUG ONLY: raw access to slots
  struct DebugSlotView
  {
    bool has_value;
    const T & value;
  };

  DebugSlotView raw_slot(std::size_t idx) const
  {
    if (!buffer_[idx].engaged) {
      // dummy static empty
      static T dummy{};
      static const DebugSlotView empty{false, dummy};
      return empty;
    }
    return DebugSlotView{true, buffer_[idx].storage};
  }

private:
  /// \brief Advance head_ and update size_ after a push.
  void advance_head_()
  {
    head_ = (head_ + 1) % capacity_;
    if (size_ < capacity_) {
      ++size_;
    } else {
      // Buffer is full, we overwrite the oldest element.
      tail_ = (tail_ + 1) % capacity_;
    }
  }

  mutable std::mutex mutex_;
  std::vector<T> buffer_;
  std::size_t capacity_;
  std::size_t head_;   // next position to write
  std::size_t tail_;   // next position to read
  std::size_t size_;   // number of valid elements
};

}  // namespace easynav

#endif  // EASYNAV_COMMON__CIRCULARBUFFER_HPP_
