#ifndef PYSICS_SHARED_SLICE_HPP
#define PYSICS_SHARED_SLICE_HPP

#include <cassert>
#include <stdexcept>
#include <boost/shared_array.hpp>

namespace pysics {
    template <typename T>
    class shared_slice {
    private:
        boost::shared_array<T> arr_;
        T *first_;
        T *last_;

    public:
        shared_slice()
        : first_(), last_()
        { }

        shared_slice(const shared_slice &other, std::size_t first,
                     std::size_t last)
        : arr_(other.arr_),
          first_(other.first_ + first),
          last_(other.first_ + last)
        {
            assert(first_ <= last_);
        }

        template <typename InputIterator>
        shared_slice(InputIterator first, InputIterator last,
                     std::size_t size)
        : arr_(size ? new T[size] : 0),
          first_(arr_.get()),
          last_(arr_.get() + size)
        {
            std::copy(first, last, arr_.get());
        }

        template <typename RandomAccessIterator>
        shared_slice(RandomAccessIterator first, RandomAccessIterator last)
        : arr_(first != last ? new T[last - first] : 0),
          first_(arr_.get()),
          last_(arr_.get() + (last - first))
        {
            assert(first_ <= last_);
            std::copy(first, last, arr_.get());
        }

        T& operator[](std::size_t index) const
        {
            assert(index < size());
            return first_[index];
        }
        
        T& at(std::size_t index) const
        {
            if (index >= size()) {
                throw std::out_of_range();
            }
            return first_[index];
        }

        T* begin() const
        {
            return first_;
        }

        T* end() const
        {
            return last_;
        }

        std::size_t size() const
        {
            assert(first_ <= last_);
            return last_ - first_;
        }
        
        bool empty() const
        {
            assert(first_ <= last_);
            return first_ == last_;
        }
    };
}

#endif
