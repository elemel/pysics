#include "wrap_body.hpp"
#include "wrap_fixture.hpp"
#include "wrap_math.hpp"
#include "wrap_shape.hpp"
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
}

BOOST_PYTHON_MODULE(pysics)
{
    using namespace pysics;

    register_exception_translator<assertion_failed>(&translate_assertion_failed);

    wrap_vec_2();
    wrap_math();
    wrap_body_type();
    wrap_world();
    wrap_body();
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
