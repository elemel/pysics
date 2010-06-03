#include <boost/noncopyable.hpp>
#include <boost/python.hpp>

#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>

using namespace boost::python;

b2Fixture* (b2Body::*b2BodyCreateFixture1)(const b2FixtureDef*) = &b2Body::CreateFixture;
b2Fixture* (b2Body::*b2BodyCreateFixture2)(const b2Shape*, float32) = &b2Body::CreateFixture;
b2Fixture* (b2Body::*b2BodyGetFixtureList)() = &b2Body::GetFixtureList;
b2JointEdge* (b2Body::*b2BodyGetJointList)() = &b2Body::GetJointList;
b2ContactEdge* (b2Body::*b2BodyGetContactList)() = &b2Body::GetContactList;
b2Body* (b2Body::*b2BodyGetNext)() = &b2Body::GetNext;
b2World* (b2Body::*b2BodyGetWorld)() = &b2Body::GetWorld;

BOOST_PYTHON_MODULE(box2d)
{
	class_<b2World>("World", init<const b2Vec2&, bool>())
        .def("create_body", &b2World::CreateBody, return_internal_reference<>())
		.def("destroy_body", &b2World::DestroyBody)
    ;

    class_<b2Vec2>("Vec2", init<float32, float32>())
        .def_readwrite("x", &b2Vec2::x)
        .def_readwrite("y", &b2Vec2::y)
    ;

	enum_<b2BodyType>("BodyType")
		.value("static_body", b2_staticBody)
		.value("kinematic_body", b2_kinematicBody)
		.value("dynamic_body", b2_dynamicBody)
		.export_values()
    ;
    
	class_<b2BodyDef>("BodyDef")
		.def_readwrite("user_data", &b2BodyDef::userData)
		.def_readwrite("position", &b2BodyDef::position)
		.def_readwrite("angle", &b2BodyDef::angle)
		.def_readwrite("linear_velocity", &b2BodyDef::linearVelocity)
		.def_readwrite("angular_velocity", &b2BodyDef::angularVelocity)
		.def_readwrite("linear_damping", &b2BodyDef::linearDamping)
		.def_readwrite("angular_damping", &b2BodyDef::angularDamping)
		.def_readwrite("allow_sleep", &b2BodyDef::allowSleep)
		.def_readwrite("awake", &b2BodyDef::awake)
		.def_readwrite("fixed_rotation", &b2BodyDef::fixedRotation)
		.def_readwrite("bullet", &b2BodyDef::bullet)
		.def_readwrite("type", &b2BodyDef::type)
		.def_readwrite("active", &b2BodyDef::active)
		.def_readwrite("inertia_scale", &b2BodyDef::inertiaScale)
	;

	class_<b2Body, boost::noncopyable>("Body", no_init)
		.def("create_fixture", b2BodyCreateFixture1, return_internal_reference<>())
		.def("create_fixture", b2BodyCreateFixture2, return_internal_reference<>())
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
		// .def("ApplyImpulse", &b2Body::ApplyImpulse)
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
		.add_property("fixture_list", make_function(b2BodyGetFixtureList, return_internal_reference<>()))
		.add_property("joint_list", make_function(b2BodyGetJointList, return_internal_reference<>()))
		.add_property("contact_list", make_function(b2BodyGetContactList, return_internal_reference<>()))
		.add_property("next", make_function(b2BodyGetNext, return_internal_reference<>()))
		// .def("GetUserData", &b2Body::GetUserData)
		// .def("SetUserData", &b2Body::SetUserData)
		.add_property("world", make_function(b2BodyGetWorld, return_internal_reference<>()))
	;

	class_<b2FixtureDef>("FixtureDef")
	;

	class_<b2Fixture, boost::noncopyable>("Fixture", no_init)
	;

	class_<b2CircleShape>("CircleShape")
	;

	class_<b2PolygonShape>("PolygonShape")
	;
}
