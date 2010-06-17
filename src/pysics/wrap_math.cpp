#include "wrap_math.hpp"

#include <boost/python.hpp>
#include <Box2D/Common/b2Math.h>

using namespace boost::python;

namespace pysics {
    std::auto_ptr<b2Vec2> create_vec_2()
    {
        return std::auto_ptr<b2Vec2>(new b2Vec2(0.0f, 0.0f));
    }

    int vec_2_len(b2Vec2 &v)
    {
        return 2;
    }

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

    std::string vec_2_repr(const b2Vec2 &v)
    {
        std::ostringstream out;
        out << "Vec2(" << v.x << ", " << v.y << ")";
        return out.str();
    }

    void wrap_vec_2()
    {
        class_<b2Vec2>("Vec2", init<float32, float32>())
            .def("__init__", make_constructor(&create_vec_2))

            .def(self += self)
            .def(self -= self)
            .def(self *= float32())

            .def(-self)
            .def(self + self)
            .def(self - self)
            .def(float32() * self)
            .def(self == self)

            .def("__len__", &vec_2_len)
            .def("__getitem__", &vec_2_getitem)
            .def("__setitem__", &vec_2_setitem)

            .def("__abs__", &b2Vec2::Length)
            .def("__repr__", &vec_2_repr)

            .def("set_zero", &b2Vec2::SetZero)
            .def("set", &b2Vec2::Set)
            .def("normalize", &b2Vec2::Normalize)

            .def_readwrite("x", &b2Vec2::x)
            .def_readwrite("y", &b2Vec2::y)
            .add_property("length", &b2Vec2::Length)
            .add_property("length_squared", &b2Vec2::LengthSquared)
            .add_property("valid", &b2Vec2::IsValid)
        ;
    }

    void wrap_math()
    {
        float32 (*dot)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;
        float32 (*cross_1)(const b2Vec2 &, const b2Vec2 &) = &b2Cross;
        b2Vec2 (*cross_2)(const b2Vec2 &, float32) = &b2Cross;
        b2Vec2 (*cross_3)(float32, const b2Vec2 &) = &b2Cross;

        def("dot", dot);
        def("cross", cross_1);
        def("cross", cross_2);
        def("cross", cross_3);
        def("distance", &b2Distance);
        def("distance_squared", &b2DistanceSquared);
    }
}
