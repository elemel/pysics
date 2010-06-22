#include "wrap_joint.hpp"

#include <boost/python.hpp>
#include <Box2D/Dynamics/b2Body.h>
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
    void wrap_joint_type()
    {
        enum_<b2JointType>("JointType")
            .value("UNKNOWN_JOINT", e_unknownJoint)
            .value("REVOLUTE_JOINT", e_revoluteJoint)
            .value("PRISMATIC_JOINT", e_prismaticJoint)
            .value("DISTANCE_JOINT", e_distanceJoint)
            .value("PULLEY_JOINT", e_pulleyJoint)
            .value("MOUSE_JOINT", e_mouseJoint)
            .value("GEAR_JOINT", e_gearJoint)
            .value("LINE_JOINT", e_lineJoint)
            .value("WELD_JOINT", e_weldJoint)
            .value("FRICTION_JOINT", e_frictionJoint)
            .export_values()
        ;
    }

    void wrap_joint()
    {
        class_<b2Joint, boost::noncopyable>("Joint", no_init)
            .add_property("type", &b2Joint::GetType)
            .add_property("body_a", make_function(&b2Joint::GetBodyA, return_internal_reference<>()))
            .add_property("body_b", make_function(&b2Joint::GetBodyB, return_internal_reference<>()))
            .add_property("next", make_function(&b2Joint::GetNext, return_internal_reference<>()))
            .add_property("user_data", &b2Joint::GetUserData, &b2Joint::SetUserData)
            .add_property("active", &b2Joint::IsActive)

            .def("get_type", &b2Joint::GetType)
            .def("get_body_a", &b2Joint::GetBodyA, return_internal_reference<>())
            .def("get_body_b", &b2Joint::GetBodyB, return_internal_reference<>())
            .def("get_anchor_a", pure_virtual(&b2Joint::GetAnchorA))
            .def("get_anchor_b", pure_virtual(&b2Joint::GetAnchorB))
            .def("get_reaction_force", pure_virtual(&b2Joint::GetReactionForce))
            .def("get_reaction_torque", pure_virtual(&b2Joint::GetReactionTorque))
            .def("get_next", &b2Joint::GetNext, return_internal_reference<>())
            .def("get_user_data", &b2Joint::GetUserData)
            .def("set_user_data", &b2Joint::SetUserData)
            .def("is_active", &b2Joint::IsActive)
        ;
    }

    void wrap_distance_joint()
    {
    }

    void wrap_friction_joint()
    {
    }
    
    void wrap_gear_joint()
    {
    }
    
    void wrap_line_joint()
    {
    }
    
    void wrap_mouse_joint()
    {
    }
    
    void wrap_prismatic_joint()
    {
    }
    
    void wrap_pulley_joint()
    {
    }
    
    void wrap_revolute_joint()
    {
    }
    
    void wrap_weld_joint()
    {
    }
}
