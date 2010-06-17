#include "wrap_body.hpp"

#include <boost/python.hpp>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>

using namespace boost::python;

namespace pysics {
    void wrap_body_type()
    {
        enum_<b2BodyType>("BodyType")
            .value("STATIC_BODY", b2_staticBody)
            .value("KINEMATIC_BODY", b2_kinematicBody)
            .value("DYNAMIC_BODY", b2_dynamicBody)
            .export_values()
        ;
    }

    b2Fixture *create_fixture(b2Body *body,
                              const b2Shape *shape,
                              b2UserData user_data,
                              float32 friction,
                              float32 restitution,
                              float32 density,
                              bool sensor,
                              uint16 category_bits,
                              uint16 mask_bits,
                              uint16 group_index)
    {
        b2FixtureDef fixture_def;
        fixture_def.shape = shape;
        fixture_def.userData = user_data;
        fixture_def.friction = friction;
        fixture_def.restitution = restitution;
        fixture_def.density = density;
        fixture_def.isSensor = sensor;
        fixture_def.filter.categoryBits = category_bits;
        fixture_def.filter.maskBits = mask_bits;
        fixture_def.filter.groupIndex = group_index;
        return body->CreateFixture(&fixture_def);
    }

    void wrap_body()
    {
        b2Fixture *(b2Body::*get_fixture_list)() = &b2Body::GetFixtureList;
        b2JointEdge *(b2Body::*get_joint_list)() = &b2Body::GetJointList;
        b2ContactEdge *(b2Body::*get_contact_list)() = &b2Body::GetContactList;
        b2Body *(b2Body::*get_next)() = &b2Body::GetNext;
        b2World *(b2Body::*get_world)() = &b2Body::GetWorld;

        class_<b2Body, boost::noncopyable>("Body", no_init)
            .def("create_fixture", &create_fixture, return_internal_reference<>(),
                 (arg("self"),
                  arg("shape"),
                  arg("user_data")=object(),
                  arg("friction")=0.2f,
                  arg("restitution")=0.0f,
                  arg("density")=0.0f,
                  arg("sensor")=false,
                  arg("category_bits")=0x0001,
                  arg("mask_bits")=0xffff,
                  arg("group_index")=0))
            .def("destroy_fixture", &b2Body::DestroyFixture)
            .add_property("transform", make_function(&b2Body::GetTransform, return_value_policy<copy_const_reference>()), &b2Body::SetTransform)
            .add_property("position", make_function(&b2Body::GetPosition, return_value_policy<copy_const_reference>()))
            .add_property("angle", &b2Body::GetAngle)
            .add_property("world_center", make_function(&b2Body::GetWorldCenter, return_value_policy<copy_const_reference>()))
            .add_property("local_center", make_function(&b2Body::GetLocalCenter, return_value_policy<copy_const_reference>()))
            .add_property("linear_velocity", &b2Body::GetLinearVelocity, &b2Body::SetLinearVelocity)
            .add_property("angular_velocity", &b2Body::GetAngularVelocity, &b2Body::SetAngularVelocity)
            .def("apply_force", &b2Body::ApplyForce)
            .def("apply_torque", &b2Body::ApplyTorque)
            .def("apply_linear_impulse", &b2Body::ApplyLinearImpulse)
            .def("apply_angular_impulse", &b2Body::ApplyAngularImpulse)
            .add_property("mass", &b2Body::GetMass)
            .add_property("inertia", &b2Body::GetInertia)
            .add_property("mass_data", &b2Body::GetMassData, &b2Body::SetMassData)
            .def("reset_mass_data", &b2Body::ResetMassData)
            .def("get_world_point", &b2Body::GetWorldPoint)
            .def("get_world_vector", &b2Body::GetWorldVector)
            .def("get_local_point", &b2Body::GetLocalPoint)
            .def("get_local_vector", &b2Body::GetLocalVector)
            .def("get_linear_velocity_from_world_point", &b2Body::GetLinearVelocityFromWorldPoint)
            .def("get_linear_velocity_from_local_point", &b2Body::GetLinearVelocityFromLocalPoint)
            .add_property("linear_damping", &b2Body::GetLinearDamping, &b2Body::SetLinearDamping)
            .add_property("angular_damping", &b2Body::GetAngularDamping, &b2Body::SetAngularDamping)
            .add_property("type", &b2Body::GetType, &b2Body::SetType)
            .add_property("bullet", &b2Body::IsBullet, &b2Body::SetBullet)
            .add_property("sleeping_allowed", &b2Body::IsSleepingAllowed, &b2Body::SetSleepingAllowed)
            .add_property("awake", &b2Body::IsAwake, &b2Body::SetAwake)
            .add_property("active", &b2Body::IsActive, &b2Body::SetActive)
            .add_property("fixed_rotation", &b2Body::IsFixedRotation, &b2Body::SetFixedRotation)
            .add_property("fixture_list", make_function(get_fixture_list, return_internal_reference<>()))
            .add_property("joint_list", make_function(get_joint_list, return_internal_reference<>()))
            .add_property("contact_list", make_function(get_contact_list, return_internal_reference<>()))
            .add_property("next", make_function(get_next, return_internal_reference<>()))
            .add_property("user_data", &b2Body::GetUserData, &b2Body::SetUserData)
            .add_property("world", make_function(get_world, return_internal_reference<>()))
        ;
    }
}
