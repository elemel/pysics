#include <boost/python.hpp>

#include <Box2D/Common/b2Settings.h>

using namespace boost::python;

void wrap_vec_2();
void wrap_math();

void wrap_world();

void wrap_body_type();
void wrap_body_def();
void wrap_body();

void wrap_filter();
void wrap_fixture_def();
void wrap_fixture();

void wrap_mass_data();
void wrap_shape_type();
void wrap_shape();
void wrap_circle_shape();
void wrap_edge_shape();
void wrap_polygon_shape();
void wrap_loop_shape();

void translate_assertion_failed(const pysics::assertion_failed &e)
{
    std::string what("assertion failed: ");
    what += e.what();
    PyErr_SetString(PyExc_AssertionError, what.c_str());
}

BOOST_PYTHON_MODULE(pysics)
{
    register_exception_translator<pysics::assertion_failed>(&translate_assertion_failed);

    wrap_vec_2();
    wrap_math();
    wrap_world();
    wrap_body_type();
    wrap_body_def();
    wrap_body();
    wrap_filter();
    wrap_fixture_def();
    wrap_fixture();
    wrap_mass_data();
    wrap_shape_type();
    wrap_shape();
    wrap_circle_shape();
    wrap_edge_shape();
    wrap_polygon_shape();
    wrap_loop_shape();
}
