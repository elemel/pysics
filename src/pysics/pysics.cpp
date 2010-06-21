#include "wrap_body.hpp"
#include "wrap_fixture.hpp"
#include "wrap_shape.hpp"
#include "wrap_vertex_array.hpp"
#include "wrap_world.hpp"

#include <boost/python.hpp>

#include <Box2D/Common/b2Settings.h>

using namespace boost::python;

namespace pysics {
    void translate_assertion_failed(const pysics::assertion_failed &e)
    {
        std::string what("assertion failed: ");
        what += e.what();
        PyErr_SetString(PyExc_AssertionError, what.c_str());
    }

    struct Vec2ToTuple {
        static PyObject *convert(b2Vec2 v)
        {
            return incref(make_tuple(v.x, v.y).ptr());
        }
    };

    struct Vec2FromTuple {
        Vec2FromTuple()
        {
            converter::registry::push_back(&convertible, &construct,
                                           type_id<b2Vec2>());
        }

        static void *convertible(PyObject *obj_ptr)
        {
            return PyTuple_Check(obj_ptr) && PyTuple_Size(obj_ptr) == 2 ? obj_ptr : 0;
        }

        static void construct(PyObject *obj_ptr,
                              converter::rvalue_from_python_stage1_data *data)
        {
            double x = PyFloat_AsDouble(PyTuple_GetItem(obj_ptr, 0));
            double y = PyFloat_AsDouble(PyTuple_GetItem(obj_ptr, 1));
            void *storage = ((converter::rvalue_from_python_storage<b2Vec2> *) data)->storage.bytes;
            new (storage) b2Vec2(x, y);
            data->convertible = storage;
        }
    };
}

BOOST_PYTHON_MODULE(pysics)
{
    using namespace pysics;

    register_exception_translator<assertion_failed>(&translate_assertion_failed);

    to_python_converter<b2Vec2, Vec2ToTuple>();
    Vec2FromTuple();

    wrap_vertex_array();
    wrap_body_type();
    wrap_world();
    wrap_body();
    wrap_fixture();
    wrap_mass_data();
    wrap_shape_type();
    wrap_shape();
    wrap_circle_shape();
    wrap_edge_shape();
    wrap_polygon_shape();
    wrap_loop_shape();
}
