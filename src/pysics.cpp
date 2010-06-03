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
		.def("CreateFixture", b2BodyCreateFixture1, return_internal_reference<>())
		.def("CreateFixture", b2BodyCreateFixture2, return_internal_reference<>())
		.def("DestroyFixture", &b2Body::DestroyFixture)
		.def("SetTransform", &b2Body::SetTransform)
		.def("GetTransform", &b2Body::GetTransform, return_value_policy<copy_const_reference>())
		.def("GetPosition", &b2Body::GetPosition, return_value_policy<copy_const_reference>())
		.def("GetAngle", &b2Body::GetAngle)
		.def("GetWorldCenter", &b2Body::GetWorldCenter, return_value_policy<copy_const_reference>())
		.def("GetLocalCenter", &b2Body::GetLocalCenter, return_value_policy<copy_const_reference>())
		.def("SetLinearVelocity", &b2Body::SetLinearVelocity)
		.def("GetLinearVelocity", &b2Body::GetLinearVelocity)
		.def("SetAngularVelocity", &b2Body::SetAngularVelocity)
		.def("GetAngularVelocity", &b2Body::GetAngularVelocity)
		.def("ApplyForce", &b2Body::ApplyForce)
		.def("ApplyTorque", &b2Body::ApplyTorque)
		// .def("ApplyImpulse", &b2Body::ApplyImpulse)
		.def("GetMass", &b2Body::GetMass)
		.def("GetInertia", &b2Body::GetInertia)
		.def("GetMassData", &b2Body::GetMassData)
		.def("SetMassData", &b2Body::SetMassData)
		.def("ResetMassData", &b2Body::ResetMassData)
		.def("GetWorldPoint", &b2Body::GetWorldPoint)
		.def("GetWorldVector", &b2Body::GetWorldVector)
		.def("GetLocalPoint", &b2Body::GetLocalPoint)
		.def("GetLocalVector", &b2Body::GetLocalVector)
		.def("GetLinearVelocityFromWorldPoint", &b2Body::GetLinearVelocityFromWorldPoint)
		.def("GetLinearVelocityFromLocalPoint", &b2Body::GetLinearVelocityFromLocalPoint)
		.def("GetLinearDamping", &b2Body::GetLinearDamping)
		.def("SetLinearDamping", &b2Body::SetLinearDamping)
		.def("GetAngularDamping", &b2Body::GetAngularDamping)
		.def("SetAngularDamping", &b2Body::SetAngularDamping)
		.def("SetType", &b2Body::SetType)
		.def("GetType", &b2Body::GetType)
		.def("SetBullet", &b2Body::SetBullet)
		.def("IsBullet", &b2Body::IsBullet)
		.def("SetSleepingAllowed", &b2Body::SetSleepingAllowed)
		.def("IsSleepingAllowed", &b2Body::IsSleepingAllowed)
		.def("SetAwake", &b2Body::SetAwake)
		.def("IsAwake", &b2Body::IsAwake)
		.def("SetActive", &b2Body::SetActive)
		.def("IsActive", &b2Body::IsActive)
		.def("SetFixedRotation", &b2Body::SetFixedRotation)
		.def("IsFixedRotation", &b2Body::IsFixedRotation)
		.def("GetFixtureList", b2BodyGetFixtureList, return_internal_reference<>())
		.def("GetJointList", b2BodyGetJointList, return_internal_reference<>())
		.def("GetContactList", b2BodyGetContactList, return_internal_reference<>())
		.def("GetNext", b2BodyGetNext, return_internal_reference<>())
		// .def("GetUserData", &b2Body::GetUserData)
		// .def("SetUserData", &b2Body::SetUserData)
		.def("GetWorld", b2BodyGetWorld, return_internal_reference<>())
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
