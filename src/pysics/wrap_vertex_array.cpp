#include "wrap_vertex_array.hpp"

#include <boost/python.hpp>

using namespace boost::python;

namespace pysics {
    std::auto_ptr<vertex_array> construct_vertex_array(list &vertices)
    {
        long n = len(vertices);
        boost::shared_array<b2Vec2> arr(new b2Vec2[n]);
        for (long i = 0; i != n; ++i) {
            arr[i] = extract<b2Vec2 &>(vertices[i]);
        }
        return std::auto_ptr<vertex_array>(new vertex_array(arr, n));
    }

    b2Vec2 &vertex_array_getitem(vertex_array &b, std::size_t index)
    {
        return b.at(index);
    }

    void vertex_array_setitem(vertex_array &b, std::size_t index, b2Vec2 &v)
    {
        b.at(index) = v;
    }

    void wrap_vertex_array()
    {
        class_<vertex_array>("VertexArray")
            .def("__init__", make_constructor(&construct_vertex_array))
            .def("__len__", &vertex_array::size)
            .def("__getitem__", &vertex_array_getitem, return_internal_reference<>())
            .def("__setitem__", &vertex_array_setitem)
        ;
    }
}
