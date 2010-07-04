#ifndef PYSICS_CONVERT_HPP
#define PYSICS_CONVERT_HPP

#include <boost/python.hpp>

namespace pysics {
    template <typename T>
    boost::python::object convert_raw_ptr(T *ptr)
    {
        typename boost::python::reference_existing_object::apply<T *>::type converter;
        return boost::python::object(boost::python::handle<>(converter(ptr)));
    }
}

#endif
