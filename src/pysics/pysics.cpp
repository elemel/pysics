#include "wrap_body.hpp"
#include "wrap_exception.hpp"
#include "wrap_fixture.hpp"
#include "wrap_shape.hpp"
#include "wrap_vec_2.hpp"
#include "wrap_vertex_array.hpp"
#include "wrap_world.hpp"

#include <boost/python.hpp>
#include <Box2D/Common/b2Settings.h>

using namespace boost::python;

BOOST_PYTHON_MODULE(pysics)
{
    using namespace pysics;

    wrap_exception();
    wrap_vec_2();
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
