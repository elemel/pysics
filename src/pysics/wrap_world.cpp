#include "wrap_world.hpp"

#include <boost/python.hpp>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2DistanceJoint.h>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>
#include <Box2D/Dynamics/Joints/b2GearJoint.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>
#include <Box2D/Dynamics/Joints/b2LineJoint.h>
#include <Box2D/Dynamics/Joints/b2MouseJoint.h>
#include <Box2D/Dynamics/Joints/b2PrismaticJoint.h>
#include <Box2D/Dynamics/Joints/b2PulleyJoint.h>
#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <Box2D/Dynamics/Joints/b2WeldJoint.h>

using namespace boost::python;

namespace pysics {
    b2Body *create_body_1(b2World *world,
                          b2BodyType type,
                          b2Vec2 position,
                          float32 angle,
                          b2Vec2 linear_velocity,
                          float32 angular_velocity,
                          float32 linear_damping,
                          float32 angular_damping,
                          bool allow_sleep,
                          bool awake,
                          bool fixed_rotation,
                          bool bullet,
                          bool active,
                          b2UserData user_data,
                          float32 inertia_scale)
    {
        b2BodyDef body_def;
        body_def.type = type;
        body_def.position = position;
        body_def.angle = angle;
        body_def.linearVelocity = linear_velocity;
        body_def.angularVelocity = angular_velocity;
        body_def.linearDamping = linear_damping;
        body_def.angularDamping = angular_damping;
        body_def.allowSleep = allow_sleep;
        body_def.awake = awake;
        body_def.fixedRotation = fixed_rotation;
        body_def.bullet = bullet;
        body_def.active = active;
        body_def.userData = user_data;
        body_def.inertiaScale = inertia_scale;
        return world->CreateBody(&body_def);
    }

    template <b2BodyType T>
    b2Body *create_body_2(b2World *world,
                          b2Vec2 position,
                          float32 angle,
                          b2Vec2 linear_velocity,
                          float32 angular_velocity,
                          float32 linear_damping,
                          float32 angular_damping,
                          bool allow_sleep,
                          bool awake,
                          bool fixed_rotation,
                          bool bullet,
                          bool active,
                          b2UserData user_data,
                          float32 inertia_scale)
    {
        b2BodyDef body_def;
        body_def.type = T;
        body_def.position = position;
        body_def.angle = angle;
        body_def.linearVelocity = linear_velocity;
        body_def.angularVelocity = angular_velocity;
        body_def.linearDamping = linear_damping;
        body_def.angularDamping = angular_damping;
        body_def.allowSleep = allow_sleep;
        body_def.awake = awake;
        body_def.fixedRotation = fixed_rotation;
        body_def.bullet = bullet;
        body_def.active = active;
        body_def.userData = user_data;
        body_def.inertiaScale = inertia_scale;
        return world->CreateBody(&body_def);
    }

    b2Joint *create_revolute_joint(b2World *world,
                                   b2Body *body_a,
                                   b2Body *body_b,
                                   b2Vec2 anchor,
                                   bool limit_enabled,
                                   float32 lower_angle,
                                   float32 upper_angle,
                                   bool motor_enabled,
                                   float32 motor_speed,
                                   float32 max_motor_torque,
                                   b2UserData user_data,
                                   bool collide_connected)
    {
        b2RevoluteJointDef revolute_joint_def;
        revolute_joint_def.Initialize(body_a, body_b, anchor);
        revolute_joint_def.enableLimit = limit_enabled;
        revolute_joint_def.lowerAngle = lower_angle;
        revolute_joint_def.upperAngle = upper_angle;
        revolute_joint_def.enableMotor = motor_enabled;
        revolute_joint_def.motorSpeed = motor_speed;
        revolute_joint_def.maxMotorTorque = max_motor_torque;
        revolute_joint_def.userData = user_data;
        revolute_joint_def.collideConnected = collide_connected;
        return world->CreateJoint(&revolute_joint_def);
    }

    b2Joint *create_prismatic_joint(b2World *world,
                                    b2Body *body_a,
                                    b2Body *body_b,
                                    b2Vec2 anchor,
                                    b2Vec2 axis,
                                    bool limit_enabled,
                                    float32 lower_translation,
                                    float32 upper_translation,
                                    bool motor_enabled,
                                    float32 max_motor_force,
                                    float32 motor_speed,
                                    b2UserData user_data,
                                    bool collide_connected)
    {
        b2PrismaticJointDef prismatic_joint_def;
        prismatic_joint_def.Initialize(body_a, body_b, anchor, axis);
        prismatic_joint_def.enableLimit = limit_enabled;
        prismatic_joint_def.lowerTranslation = lower_translation;
        prismatic_joint_def.upperTranslation = upper_translation;
        prismatic_joint_def.enableMotor = motor_enabled;
        prismatic_joint_def.maxMotorForce = max_motor_force;
        prismatic_joint_def.motorSpeed = motor_speed;
        prismatic_joint_def.userData = user_data;
        prismatic_joint_def.collideConnected = collide_connected;
        return world->CreateJoint(&prismatic_joint_def);
    }

    b2Joint *create_distance_joint(b2World *world)
    {
        b2DistanceJointDef distance_joint_def;
        return world->CreateJoint(&distance_joint_def);
    }

    b2Joint *create_pulley_joint(b2World *world)
    {
        b2PulleyJointDef pulley_joint_def;
        return world->CreateJoint(&pulley_joint_def);
    }

    b2Joint *create_mouse_joint(b2World *world)
    {
        b2MouseJointDef mouse_joint_def;
        return world->CreateJoint(&mouse_joint_def);
    }

    b2Joint *create_gear_joint(b2World *world)
    {
        b2GearJointDef gear_joint_def;
        return world->CreateJoint(&gear_joint_def);
    }

    b2Joint *create_line_joint(b2World *world)
    {
        b2LineJointDef line_joint_def;
        return world->CreateJoint(&line_joint_def);
    }

    b2Joint *create_weld_joint(b2World *world)
    {
        b2WeldJointDef weld_joint_def;
        return world->CreateJoint(&weld_joint_def);
    }

    b2Joint *create_friction_joint(b2World *world)
    {
        b2FrictionJointDef friction_joint_def;
        return world->CreateJoint(&friction_joint_def);
    }

    void wrap_world()
    {
        class_<b2World>("World", init<b2Vec2, bool>())
            .add_property("body_list", make_function(&b2World::GetBodyList, return_internal_reference<>()))
            .add_property("joint_list", make_function(&b2World::GetJointList, return_internal_reference<>()))
            .add_property("contact_list", make_function(&b2World::GetContactList, return_internal_reference<>()))
            .add_property("proxy_count", &b2World::GetProxyCount)
            .add_property("body_count", &b2World::GetBodyCount)
            .add_property("joint_count", &b2World::GetJointCount)
            .add_property("contact_count", &b2World::GetContactCount)
            .add_property("gravity", &b2World::GetGravity, &b2World::SetGravity)
            .add_property("locked", &b2World::IsLocked)
            .add_property("auto_clear_forces", &b2World::GetAutoClearForces, &b2World::SetAutoClearForces)
            .add_property("contact_manager", make_function(&b2World::GetContactManager, return_internal_reference<>()))

            .def("set_destruction_listener", &b2World::SetDestructionListener)
            .def("set_contact_filter", &b2World::SetContactFilter)
            .def("set_contact_listener", &b2World::SetContactListener)
            .def("set_debug_draw", &b2World::SetDebugDraw)
            .def("create_body", &create_body_1, return_internal_reference<>(),
                 (arg("self"),
                  arg("type"),
                  arg("position")=b2Vec2(0.0f, 0.0f),
                  arg("angle")=0.0f,
                  arg("linear_velocity")=b2Vec2(0.0f, 0.0f),
                  arg("angular_velocity")=0.0f,
                  arg("linear_damping")=0.0f,
                  arg("angular_damping")=0.0f,
                  arg("allow_sleep")=true,
                  arg("awake")=true,
                  arg("fixed_rotation")=false,
                  arg("bullet")=false,
                  arg("active")=true,
                  arg("user_data")=object(),
                  arg("inertia_scale")=1.0f))
            .def("create_static_body", &create_body_2<b2_staticBody>, return_internal_reference<>(),
                 (arg("self"),
                  arg("position")=b2Vec2(0.0f, 0.0f),
                  arg("angle")=0.0f,
                  arg("linear_velocity")=b2Vec2(0.0f, 0.0f),
                  arg("angular_velocity")=0.0f,
                  arg("linear_damping")=0.0f,
                  arg("angular_damping")=0.0f,
                  arg("allow_sleep")=true,
                  arg("awake")=true,
                  arg("fixed_rotation")=false,
                  arg("bullet")=false,
                  arg("active")=true,
                  arg("user_data")=object(),
                  arg("inertia_scale")=1.0f))
            .def("create_kinematic_body", &create_body_2<b2_kinematicBody>, return_internal_reference<>(),
                 (arg("self"),
                  arg("position")=b2Vec2(0.0f, 0.0f),
                  arg("angle")=0.0f,
                  arg("linear_velocity")=b2Vec2(0.0f, 0.0f),
                  arg("angular_velocity")=0.0f,
                  arg("linear_damping")=0.0f,
                  arg("angular_damping")=0.0f,
                  arg("allow_sleep")=true,
                  arg("awake")=true,
                  arg("fixed_rotation")=false,
                  arg("bullet")=false,
                  arg("active")=true,
                  arg("user_data")=object(),
                  arg("inertia_scale")=1.0f))
            .def("create_dynamic_body", &create_body_2<b2_dynamicBody>, return_internal_reference<>(),
                 (arg("self"),
                  arg("position")=b2Vec2(0.0f, 0.0f),
                  arg("angle")=0.0f,
                  arg("linear_velocity")=b2Vec2(0.0f, 0.0f),
                  arg("angular_velocity")=0.0f,
                  arg("linear_damping")=0.0f,
                  arg("angular_damping")=0.0f,
                  arg("allow_sleep")=true,
                  arg("awake")=true,
                  arg("fixed_rotation")=false,
                  arg("bullet")=false,
                  arg("active")=true,
                  arg("user_data")=object(),
                  arg("inertia_scale")=1.0f))
            .def("destroy_body", &b2World::DestroyBody)
            .def("create_revolute_joint", &create_revolute_joint, return_internal_reference<>(),
                 (arg("self"),
                  arg("body_a"),
                  arg("body_b"),
                  arg("anchor"),
                  arg("limit_enabled")=false,
                  arg("lower_angle")=0.0f,
                  arg("upper_angle")=0.0f,
                  arg("motor_enabled")=false,
                  arg("motor_speed")=0.0f,
                  arg("max_motor_torque")=0.0f,
                  arg("user_data")=object(),
                  arg("collide_connected")=false))
            .def("create_prismatic_joint", &create_prismatic_joint, return_internal_reference<>(),
                 (arg("self"),
                  arg("body_a"),
                  arg("body_b"),
                  arg("anchor"),
                  arg("axis"),
                  arg("limit_enabled")=false,
                  arg("lower_translation")=0.0f,
                  arg("upper_translation")=0.0f,
                  arg("motor_enabled")=false,
                  arg("max_motor_force")=0.0f,
                  arg("motor_speed")=0.0f,
                  arg("user_data")=object(),
                  arg("collide_connected")=false))
            .def("create_distance_joint", &create_distance_joint, return_internal_reference<>())
            .def("create_pulley_joint", &create_pulley_joint, return_internal_reference<>())
            .def("create_mouse_joint", &create_mouse_joint, return_internal_reference<>())
            .def("create_gear_joint", &create_gear_joint, return_internal_reference<>())
            .def("create_line_joint", &create_line_joint, return_internal_reference<>())
            .def("create_weld_joint", &create_weld_joint, return_internal_reference<>())
            .def("create_friction_joint", &create_friction_joint, return_internal_reference<>())
            .def("destroy_joint", &b2World::DestroyJoint)
            .def("step", &b2World::Step)
            .def("clear_forces", &b2World::ClearForces)
            .def("draw_debug_data", &b2World::DrawDebugData)
            .def("query_aabb", &b2World::QueryAABB)
            .def("ray_cast", &b2World::RayCast)
            .def("get_body_list", &b2World::GetBodyList, return_internal_reference<>())
            .def("get_joint_list", &b2World::GetJointList, return_internal_reference<>())
            .def("get_contact_list", &b2World::GetContactList, return_internal_reference<>())
            .def("set_warm_starting", &b2World::SetWarmStarting)
            .def("set_continuous_physics", &b2World::SetContinuousPhysics)
            .def("get_proxy_count", &b2World::GetProxyCount)
            .def("get_body_count", &b2World::GetBodyCount)
            .def("get_joint_count", &b2World::GetJointCount)
            .def("get_contact_count", &b2World::GetContactCount)
            .def("get_gravity", &b2World::GetGravity)
            .def("set_gravity", &b2World::SetGravity)
            .def("is_locked", &b2World::IsLocked)
            .def("get_auto_clear_forces", &b2World::GetAutoClearForces)
            .def("set_auto_clear_forces", &b2World::SetAutoClearForces)
            .def("get_contact_manager", &b2World::GetContactManager, return_internal_reference<>())
        ;
    }
}
