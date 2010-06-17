#include "wrap_fixture.hpp"

#include <boost/python.hpp>
#include <Box2D/Dynamics/b2Fixture.h>

using namespace boost::python;

namespace pysics {
    namespace {
        uint16 fixture_def_get_category_bits(const b2FixtureDef *fixture_def)
        {
            return fixture_def->filter.categoryBits;
        }

        void fixture_def_set_category_bits(b2FixtureDef *fixture_def, uint16 category_bits)
        {
            fixture_def->filter.categoryBits = category_bits;
        }

        uint16 fixture_def_get_mask_bits(const b2FixtureDef *fixture_def)
        {
            return fixture_def->filter.maskBits;
        }

        void fixture_def_set_mask_bits(b2FixtureDef *fixture_def, uint16 mask_bits)
        {
            fixture_def->filter.maskBits = mask_bits;
        }

        uint16 fixture_def_get_group_index(const b2FixtureDef *fixture_def)
        {
            return fixture_def->filter.groupIndex;
        }

        void fixture_def_set_group_index(b2FixtureDef *fixture_def, uint16 group_index)
        {
            fixture_def->filter.groupIndex = group_index;
        }

        uint16 fixture_get_category_bits(const b2Fixture *fixture)
        {
            return fixture->GetFilterData().categoryBits;
        }

        void fixture_set_category_bits(b2Fixture *fixture, uint16 category_bits)
        {
            b2Filter filter = fixture->GetFilterData();
            filter.categoryBits = category_bits;
            fixture->SetFilterData(filter);
        }

        uint16 fixture_get_mask_bits(const b2Fixture *fixture)
        {
            return fixture->GetFilterData().maskBits;
        }

        void fixture_set_mask_bits(b2Fixture *fixture, uint16 mask_bits)
        {
            b2Filter filter = fixture->GetFilterData();
            filter.maskBits = mask_bits;
            fixture->SetFilterData(filter);
        }

        uint16 fixture_get_group_index(const b2Fixture *fixture)
        {
            return fixture->GetFilterData().groupIndex;
        }

        void fixture_set_group_index(b2Fixture *fixture, uint16 group_index)
        {
            b2Filter filter = fixture->GetFilterData();
            filter.groupIndex = group_index;
            fixture->SetFilterData(filter);
        }
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
            .add_property("category_bits", &fixture_def_get_category_bits, &fixture_def_set_category_bits)
            .add_property("mask_bits", &fixture_def_get_mask_bits, &fixture_def_set_mask_bits)
            .add_property("group_index", &fixture_def_get_group_index, &fixture_def_set_group_index)
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
            .add_property("category_bits", &fixture_get_category_bits, &fixture_set_category_bits)
            .add_property("mask_bits", &fixture_get_mask_bits, &fixture_set_mask_bits)
            .add_property("group_index", &fixture_get_group_index, &fixture_set_group_index)
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
}
