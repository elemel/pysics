#include <boost/python.hpp>
#include <Box2D/Common/b2Math.h>

using namespace boost::python;

namespace pysics {
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

    void wrap_vec_2()
    {
        to_python_converter<b2Vec2, Vec2ToTuple>();
        Vec2FromTuple();
    }
}
