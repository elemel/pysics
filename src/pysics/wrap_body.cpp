#include "wrap_body.hpp"

#include "wrap_vertex_array.hpp"

#include <boost/python.hpp>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2LoopShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>

using namespace boost::python;

namespace pysics {
    namespace {
        b2Fixture *create_fixture(b2Body *body,
                                  b2Shape *shape,
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

        b2Fixture *create_circle_fixture(b2Body *body,
                                         b2Vec2 position,
                                         float32 radius,
                                         b2UserData user_data,
                                         float32 friction,
                                         float32 restitution,
                                         float32 density,
                                         bool sensor,
                                         uint16 category_bits,
                                         uint16 mask_bits,
                                         uint16 group_index)
        {
            b2CircleShape circle_shape;
            circle_shape.m_p = position;
            circle_shape.m_radius = radius;

            b2FixtureDef fixture_def;
            fixture_def.shape = &circle_shape;
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

        b2Fixture *create_edge_fixture(b2Body *body,
                                       b2Vec2 vertex_1,
                                       b2Vec2 vertex_2,
                                       b2UserData user_data,
                                       float32 friction,
                                       float32 restitution,
                                       float32 density,
                                       bool sensor,
                                       uint16 category_bits,
                                       uint16 mask_bits,
                                       uint16 group_index)
        {
            b2EdgeShape edge_shape;
            edge_shape.m_vertex1 = vertex_1;
            edge_shape.m_vertex2 = vertex_2;

            b2FixtureDef fixture_def;
            fixture_def.shape = &edge_shape;
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

        void set_polygon_shape_vertices(b2PolygonShape *polygon_shape, list vertices)
        {
            b2Vec2 arr[b2_maxPolygonVertices];
            long n = len(vertices);
            for (long i = 0; i != n; ++i) {
                arr[i] = extract<b2Vec2>(vertices[i]);
            }
            polygon_shape->Set(arr, n);
        }

        b2Fixture *create_polygon_fixture(b2Body *body,
                                          list vertices,
                                          b2UserData user_data,
                                          float32 friction,
                                          float32 restitution,
                                          float32 density,
                                          bool sensor,
                                          uint16 category_bits,
                                          uint16 mask_bits,
                                          uint16 group_index)
        {
            b2PolygonShape polygon_shape;
            if (len(vertices)) {
                set_polygon_shape_vertices(&polygon_shape, vertices);
            } else {
                polygon_shape.SetAsBox(0.5f, 0.5f);
            }

            b2FixtureDef fixture_def;
            fixture_def.shape = &polygon_shape;
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

        b2Fixture *create_loop_fixture(b2Body *body,
                                       VertexArray vertices,
                                       b2UserData user_data,
                                       float32 friction,
                                       float32 restitution,
                                       float32 density,
                                       bool sensor,
                                       uint16 category_bits,
                                       uint16 mask_bits,
                                       uint16 group_index)
        {
            b2LoopShape loop_shape;
            loop_shape.m_vertices = vertices.ptr();
            loop_shape.m_count = vertices.size();

            b2FixtureDef fixture_def;
            fixture_def.shape = &loop_shape;
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

        b2Fixture *create_box_fixture(b2Body *body,
                                      float32 half_width,
                                      float32 half_height,
                                      b2Vec2 center,
                                      float32 angle,
                                      b2UserData user_data,
                                      float32 friction,
                                      float32 restitution,
                                      float32 density,
                                      bool sensor,
                                      uint16 category_bits,
                                      uint16 mask_bits,
                                      uint16 group_index)
        {
            b2PolygonShape polygon_shape;
            polygon_shape.SetAsBox(half_width, half_height, center, angle);

            b2FixtureDef fixture_def;
            fixture_def.shape = &polygon_shape;
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

        bool body_eq(b2Body *left, b2Body *right)
        {
            return left == right;
        }

        bool body_ne(b2Body *left, b2Body *right)
        {
            return left != right;
        }

        std::size_t hash_body(b2Body *body)
        {
            return reinterpret_cast<std::size_t>(body);
        }
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

    void wrap_body()
    {
        b2Fixture *(b2Body::*get_fixture_list)() = &b2Body::GetFixtureList;
        b2JointEdge *(b2Body::*get_joint_list)() = &b2Body::GetJointList;
        b2ContactEdge *(b2Body::*get_contact_list)() = &b2Body::GetContactList;
        b2Body *(b2Body::*get_next)() = &b2Body::GetNext;
        b2World *(b2Body::*get_world)() = &b2Body::GetWorld;

        class_<b2Body, boost::noncopyable>("Body", no_init)
            .def("__eq__", &body_eq)
            .def("__ne__", &body_ne)
            .def("__hash__", &hash_body)

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
            .def("create_circle_fixture", &create_circle_fixture, return_internal_reference<>(),
                 (arg("self"),
                  arg("position")=b2Vec2(0.0f, 0.0f),
                  arg("radius")=0.5f,
                  arg("user_data")=object(),
                  arg("friction")=0.2f,
                  arg("restitution")=0.0f,
                  arg("density")=0.0f,
                  arg("sensor")=false,
                  arg("category_bits")=0x0001,
                  arg("mask_bits")=0xffff,
                  arg("group_index")=0))
            .def("create_edge_fixture", &create_edge_fixture, return_internal_reference<>(),
                 (arg("self"),
                  arg("vertex_1")=b2Vec2(0.0f, 0.0f),
                  arg("vertex_2")=b2Vec2(0.0f, 0.0f),
                  arg("user_data")=object(),
                  arg("friction")=0.2f,
                  arg("restitution")=0.0f,
                  arg("density")=0.0f,
                  arg("sensor")=false,
                  arg("category_bits")=0x0001,
                  arg("mask_bits")=0xffff,
                  arg("group_index")=0))
            .def("create_polygon_fixture", &create_polygon_fixture, return_internal_reference<>(),
                 (arg("self"),
                  arg("vertices")=list(),
                  arg("user_data")=object(),
                  arg("friction")=0.2f,
                  arg("restitution")=0.0f,
                  arg("density")=0.0f,
                  arg("sensor")=false,
                  arg("category_bits")=0x0001,
                  arg("mask_bits")=0xffff,
                  arg("group_index")=0))
            .def("create_loop_fixture", &create_loop_fixture, return_internal_reference<>(),
                 (arg("self"),
                  arg("vertices"),
                  arg("user_data")=object(),
                  arg("friction")=0.2f,
                  arg("restitution")=0.0f,
                  arg("density")=0.0f,
                  arg("sensor")=false,
                  arg("category_bits")=0x0001,
                  arg("mask_bits")=0xffff,
                  arg("group_index")=0))
            .def("create_box_fixture", &create_box_fixture, return_internal_reference<>(),
                 (arg("self"),
                  arg("half_width")=0.5f,
                  arg("half_height")=0.5f,
                  arg("center")=b2Vec2(0.0f, 0.0f),
                  arg("angle")=0.0f,
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
