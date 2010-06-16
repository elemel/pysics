#include "wrap_math.hpp"

#include <boost/python.hpp>
#include <Box2D/Common/b2Math.h>

using namespace boost::python;

namespace pysics {
    float32 vec_2_getitem(const b2Vec2 &v, int32 index)
    {
        if (index >= 0 && index < 2) {
            return v(index);
        } else {
            throw std::out_of_range("Vec2 index out of range");
        }
    }

    void vec_2_setitem(b2Vec2 &v, int32 index, float32 value)
    {
        if (index >= 0 && index < 2) {
            v(index) = value;
        } else {
            throw std::out_of_range("Vec2 index out of range");
        }
    }

    int vec_2_len(b2Vec2 &v)
    {
        return 2;
    }

    float32 *vec_2_begin(b2Vec2 &v)
    {
        return &v.x;
    }

    float32 *vec_2_end(b2Vec2 &v)
    {
        return &v.x + 2;
    }

    std::string vec_2_repr(const b2Vec2 &v)
    {
        std::ostringstream out;
        out << "Vec2(" << v.x << ", " << v.y << ")";
        return out.str();
    }

    void wrap_vec_2()
    {
        class_<b2Vec2>("Vec2", init<float32, float32>())
            .def("set_zero", &b2Vec2::SetZero)
            .def("set", &b2Vec2::Set)
            .def(-self)
            .def("__getitem__", &vec_2_getitem)
            .def("__setitem__", &vec_2_setitem)
            .def(self += self)
            .def(self -= self)
            .def(self *= float32())
            .add_property("length", &b2Vec2::Length)
            .add_property("length_squared", &b2Vec2::LengthSquared)
            .def("normalize", &b2Vec2::Normalize)
            .add_property("valid", &b2Vec2::IsValid)
            .def_readwrite("x", &b2Vec2::x)
            .def_readwrite("y", &b2Vec2::y)
            .def("__len__", &vec_2_len)
            .def("__iter__", range(&vec_2_begin, &vec_2_end))
            .def("__repr__", &vec_2_repr)
            .def(self + self)
            .def(self - self)
            .def(float32() * self)
            .def(self == self)
        ;
    }

    void wrap_math()
    {
        float32 (*dot)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;
        float32 (*cross_1)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;
        float32 (*cross_2)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;
        float32 (*cross_3)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;

        def("dot", dot);
        def("cross", cross_1);
        def("cross", cross_2);
        def("cross", cross_3);
        def("distance", &b2Distance);
        def("distance_squared", &b2DistanceSquared);
    }
}
