#ifndef PYSICS_OPERATOR_HPP
#define PYSICS_OPERATOR_HPP

#include <cstddef>

namespace pysics {
    template <typename T>
    bool eq_ptr(const T *left, const T *right)
    {
        return left == right;
    }

    template <typename T>
    bool ne_ptr(const T *left, const T *right)
    {
        return left != right;
    }

    template <typename T>
    std::size_t hash_ptr(const T *ptr)
    {
        return reinterpret_cast<std::size_t>(ptr);
    }
}

#endif
