#include <boost/python.hpp>

#include <Box2D/Dynamics/b2Fixture.h>

using namespace boost::python;

void wrap_filter()
{
    class_<b2Filter>("Filter")
        .def_readwrite("category_bits", &b2Filter::categoryBits)
        .def_readwrite("mask_bits", &b2Filter::maskBits)
        .def_readwrite("group_index", &b2Filter::groupIndex)
    ;
}

void wrap_fixture_def()
{
    class_<b2FixtureDef>("FixtureDef")
        .def_readwrite("shape", &b2FixtureDef::shape)
        .def_readwrite("user_data", &b2FixtureDef::userData)
        .def_readwrite("friction", &b2FixtureDef::friction)
        .def_readwrite("restitution", &b2FixtureDef::restitution)
        .def_readwrite("density", &b2FixtureDef::density)
        .def_readwrite("sensor", &b2FixtureDef::isSensor)
        .def_readwrite("filter", &b2FixtureDef::filter)
    ;
}

void wrap_fixture()
{
    b2Shape *(b2Fixture::*get_shape)() = &b2Fixture::GetShape;
    b2Body *(b2Fixture::*get_body)() = &b2Fixture::GetBody;
    b2Fixture *(b2Fixture::*get_next)() = &b2Fixture::GetNext;

    class_<b2Fixture, boost::noncopyable>("Fixture", no_init)
        .add_property("type", &b2Fixture::GetType)
        .add_property("shape", make_function(get_shape, return_internal_reference<>()))
        .add_property("sensor", &b2Fixture::IsSensor, &b2Fixture::SetSensor)
        .add_property("filter_data", make_function(&b2Fixture::GetFilterData, return_value_policy<copy_const_reference>()), &b2Fixture::SetFilterData)
        .add_property("body", make_function(get_body, return_internal_reference<>()))
        .add_property("next", make_function(get_next, return_internal_reference<>()))
        .add_property("user_data", &b2Fixture::GetUserData, &b2Fixture::SetUserData)
        .def("test_point", &b2Fixture::TestPoint)
        .def("ray_cast", &b2Fixture::RayCast)
        .def("get_mass_data", &b2Fixture::GetMassData)
        .add_property("density", &b2Fixture::GetDensity, &b2Fixture::SetDensity)
        .add_property("friction", &b2Fixture::GetFriction, &b2Fixture::SetFriction)
        .add_property("restitution", &b2Fixture::GetRestitution, &b2Fixture::SetRestitution)
        .def("get_aabb", make_function(&b2Fixture::GetAABB, return_value_policy<copy_const_reference>()))
    ;
}
