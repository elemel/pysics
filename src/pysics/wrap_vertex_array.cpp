#include "wrap_vertex_array.hpp"

#include <boost/python.hpp>

using namespace boost::python;

namespace pysics {
    std::auto_ptr<VertexArray> construct_vertex_array(list vertices)
    {
        long n = len(vertices);
        boost::shared_array<b2Vec2> arr(new b2Vec2[n]);
        for (long i = 0; i != n; ++i) {
            arr[i] = extract<b2Vec2 &>(vertices[i]);
        }
        return std::auto_ptr<VertexArray>(new VertexArray(arr, n));
    }

    b2Vec2 *vertex_array_getitem(VertexArray *b, std::size_t index)
    {
        return &b->at(index);
    }

    void vertex_array_setitem(VertexArray *b, std::size_t index, b2Vec2 *v)
    {
        b->at(index) = *v;
    }

    void wrap_vertex_array()
    {
        class_<VertexArray>("VertexArray")
            .def("__init__", make_constructor(&construct_vertex_array))
            .def("__len__", &VertexArray::size)
            .def("__getitem__", &vertex_array_getitem, return_internal_reference<>())
            .def("__setitem__", &vertex_array_setitem)
        ;
    }
}
