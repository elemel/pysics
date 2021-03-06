#include "pysics.hpp"

#include <boost/python.hpp>

using namespace boost::python;
using namespace pysics;

BOOST_PYTHON_MODULE(pysics)
{
    wrap_body_type();
    wrap_exception();
    wrap_joint_type();
    wrap_mass_data();
    wrap_shape_type();
    wrap_vec_2();
    wrap_vertex_array();

    wrap_shape();
    wrap_circle_shape();
    wrap_edge_shape();
    wrap_polygon_shape();
    wrap_loop_shape();

    wrap_world();
    wrap_debug_draw();
    wrap_body();
    wrap_fixture();

    wrap_joint();
    wrap_revolute_joint();
    wrap_prismatic_joint();
    wrap_distance_joint();
    wrap_pulley_joint();
    wrap_mouse_joint();
    wrap_gear_joint();
    wrap_line_joint();
    wrap_weld_joint();
    wrap_friction_joint();
}
