#include <boost/noncopyable.hpp>
#include <boost/python.hpp>

#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2LoopShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>

using namespace boost::python;

float32 vec_2_getitem(const b2Vec2 &v, int32 index)
{
    if (index >= 0 && index < 2) {
        return v(index);
    } else {
        throw std::out_of_range("Vec2 index out of range");
    }
}

void vec_2_setitem(b2Vec2 &v, int32 index, float32 value)
{
    if (index >= 0 && index < 2) {
        v(index) = value;
    } else {
        throw std::out_of_range("Vec2 index out of range");
    }
}

int vec_2_len(b2Vec2 &v)
{
    return 2;
}

float32 *vec_2_begin(b2Vec2 &v)
{
    return &v.x;
}

float32 *vec_2_end(b2Vec2 &v)
{
    return &v.x + 2;
}

std::string vec_2_repr(const b2Vec2 &v)
{
    std::ostringstream out;
    out << "Vec2(" << v.x << ", " << v.y << ")";
    return out.str();
}

void wrap_vec_2()
{
    class_<b2Vec2>("Vec2", init<float32, float32>())
        .def("set_zero", &b2Vec2::SetZero)
        .def("set", &b2Vec2::Set)
        .def(-self)
        .def("__getitem__", &vec_2_getitem)
        .def("__setitem__", &vec_2_setitem)
        .def(self += self)
        .def(self -= self)
        .def(self *= float32())
        .add_property("length", &b2Vec2::Length)
        .add_property("length_squared", &b2Vec2::LengthSquared)
        .def("normalize", &b2Vec2::Normalize)
        .add_property("valid", &b2Vec2::IsValid)
        .def_readwrite("x", &b2Vec2::x)
        .def_readwrite("y", &b2Vec2::y)
        .def("__len__", &vec_2_len)
        .def("__iter__", range(&vec_2_begin, &vec_2_end))
        .def("__repr__", &vec_2_repr)
        .def(self + self)
        .def(self - self)
        .def(float32() * self)
        .def(self == self)
    ;
}

void wrap_math()
{
    float32 (*dot)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;
    float32 (*cross_1)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;
    float32 (*cross_2)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;
    float32 (*cross_3)(const b2Vec2 &, const b2Vec2 &) = &b2Dot;

    def("dot", dot);
    def("cross", cross_1);
    def("cross", cross_2);
    def("cross", cross_3);
    def("distance", &b2Distance);
    def("distance_squared", &b2DistanceSquared);
}

void wrap_world()
{
    class_<b2World>("World", init<const b2Vec2&, bool>())
        .def("set_destruction_listener", &b2World::SetDestructionListener)
        .def("set_contact_filter", &b2World::SetContactFilter)
        .def("set_contact_listener", &b2World::SetContactListener)
        .def("set_debug_draw", &b2World::SetDebugDraw)
        .def("create_body", &b2World::CreateBody, return_internal_reference<>())
        .def("destroy_body", &b2World::DestroyBody)
        .def("create_joint", &b2World::CreateJoint, return_internal_reference<>())
        .def("destroy_joint", &b2World::DestroyJoint)
        .def("step", &b2World::Step)
        .def("clear_forces", &b2World::ClearForces)
        .def("draw_debug_data", &b2World::DrawDebugData)
        .def("query_aabb", &b2World::QueryAABB)
        .def("ray_cast", &b2World::RayCast)
        .add_property("body_list", make_function(&b2World::GetBodyList, return_internal_reference<>()))
        .add_property("joint_list", make_function(&b2World::GetJointList, return_internal_reference<>()))
        .add_property("contact_list", make_function(&b2World::GetContactList, return_internal_reference<>()))
        .def("set_warm_starting", &b2World::SetWarmStarting)
        .def("set_continuous_physics", &b2World::SetContinuousPhysics)
        .add_property("proxy_count", &b2World::GetProxyCount)
        .add_property("body_count", &b2World::GetBodyCount)
        .add_property("joint_count", &b2World::GetJointCount)
        .add_property("contact_count", &b2World::GetContactCount)
        .add_property("gravity", &b2World::GetGravity, &b2World::SetGravity)
        .add_property("locked", &b2World::IsLocked)
        .add_property("auto_clear_forces", &b2World::GetAutoClearForces, &b2World::SetAutoClearForces)
        .add_property("contact_manager", make_function(&b2World::GetContactManager, return_internal_reference<>()))
    ;
}

void wrap_body_type()
{
    enum_<b2BodyType>("BodyType")
        .value("STATIC_BODY", b2_staticBody)
        .value("KINEMATIC_BODY", b2_kinematicBody)
        .value("DYNAMIC_BODY", b2_dynamicBody)
        .export_values()
    ;
}

void wrap_body_def()
{
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
}

void wrap_body()
{
    b2Fixture *(b2Body::*create_fixture_1)(const b2FixtureDef *) = &b2Body::CreateFixture;
    b2Fixture *(b2Body::*create_fixture_2)(const b2Shape *, float32) = &b2Body::CreateFixture;
    b2Fixture *(b2Body::*get_fixture_list)() = &b2Body::GetFixtureList;
    b2JointEdge *(b2Body::*get_joint_list)() = &b2Body::GetJointList;
    b2ContactEdge *(b2Body::*get_contact_list)() = &b2Body::GetContactList;
    b2Body *(b2Body::*get_next)() = &b2Body::GetNext;
    b2World *(b2Body::*get_world)() = &b2Body::GetWorld;

    class_<b2Body, boost::noncopyable>("Body", no_init)
        .def("create_fixture", create_fixture_1, return_internal_reference<>())
        .def("create_fixture", create_fixture_2, return_internal_reference<>())
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

void wrap_mass_data()
{
    class_<b2MassData>("MassData")
        .def_readwrite("mass", &b2MassData::mass)
        .def_readwrite("center", &b2MassData::center)
        .def_readwrite("inertia", &b2MassData::I)
    ;
}

void wrap_shape_type()
{
    enum_<b2Shape::Type>("ShapeType")
        .value("UNKNOWN_SHAPE", b2Shape::e_unknown)
        .value("CIRCLE_SHAPE", b2Shape::e_circle)
        .value("EDGE_SHAPE", b2Shape::e_edge)
        .value("POLYGON_SHAPE", b2Shape::e_polygon)
        .value("LOOP_SHAPE", b2Shape::e_loop)
        .export_values()
    ;
}

void wrap_shape()
{
    class_<b2Shape, boost::noncopyable>("Shape", no_init)
        .def("clone", pure_virtual(&b2Shape::Clone), return_value_policy<manage_new_object>())
        .add_property("type", &b2Shape::GetType)
        // .def("get_child_count", pure_virtual(&b2Shape::GetChildCount))
        .def("test_point", pure_virtual(&b2Shape::TestPoint))
        .def("ray_cast", pure_virtual(&b2Shape::RayCast))
        .def("compute_aabb", pure_virtual(&b2Shape::ComputeAABB))
        .def("compute_mass", pure_virtual(&b2Shape::ComputeMass))
        .def_readwrite("radius", &b2Shape::m_radius)
    ;
}

b2CircleShape *construct_circle_shape(const b2Vec2 &position, float32 radius)
{
    b2CircleShape *circle_shape = new b2CircleShape;
    circle_shape->m_p = position;
    circle_shape->m_radius = radius;
    return circle_shape;
}

void wrap_circle_shape()
{
    class_<b2CircleShape, bases<b2Shape> >("CircleShape")
        .def("__init__", make_constructor(construct_circle_shape))
        .add_property("child_count", &b2CircleShape::GetChildCount)
        .def("get_support", &b2CircleShape::GetSupport)
        .def("get_support_vertex", &b2CircleShape::GetSupportVertex, return_value_policy<copy_const_reference>())
        .add_property("vertex_count", &b2CircleShape::GetVertexCount)
        .def("get_vertex", &b2CircleShape::GetVertex, return_value_policy<copy_const_reference>())
        .def_readwrite("position", &b2CircleShape::m_p)
    ;
}

void wrap_edge_shape()
{
    class_<b2EdgeShape, bases<b2Shape> >("EdgeShape")
        .add_property("child_count", &b2EdgeShape::GetChildCount)
        .def_readwrite("vertex_1", &b2EdgeShape::m_vertex1)
        .def_readwrite("vertex_2", &b2EdgeShape::m_vertex2)
    ;
}

list get_vertices(const b2PolygonShape *polygon_shape)
{
    list vertices;
    int32 n = polygon_shape->GetVertexCount();
    for (int32 i = 0; i != n; ++i) {
        vertices.append(polygon_shape->GetVertex(i));
    }
    return vertices;
}

void set_vertices(b2PolygonShape *polygon_shape, const list &vertices)
{
    b2Vec2 arr[b2_maxPolygonVertices];
    long n = len(vertices);
    for (long i = 0; i != n; ++i) {
        arr[i] = extract<const b2Vec2 &>(vertices[i]);
    }
    polygon_shape->Set(arr, n);
}

void wrap_polygon_shape()
{
    class_<b2PolygonShape, bases<b2Shape> >("PolygonShape")
        .add_property("child_count", &b2PolygonShape::GetChildCount)
        .add_property("vertex_count", &b2PolygonShape::GetVertexCount)
        .def("get_vertex", &b2PolygonShape::GetVertex, return_value_policy<copy_const_reference>())
        .add_property("vertices", &get_vertices, &set_vertices)
    ;
}

void wrap_loop_shape()
{
    class_<b2LoopShape, bases<b2Shape> >("LoopShape")
        .add_property("child_count", &b2LoopShape::GetChildCount)
        .def("get_child_edge", &b2LoopShape::GetChildEdge)
    ;
}

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
