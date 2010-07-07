#include "operator.hpp"

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
            .def("__eq__", &eq_ptr<b2Joint>)
            .def("__ne__", &ne_ptr<b2Joint>)
            .def("__hash__", &hash_ptr<b2Joint>)

            .add_property("type", &b2Joint::GetType)
            .add_property("body_a", make_function(&b2Joint::GetBodyA, return_internal_reference<>()))
            .add_property("body_b", make_function(&b2Joint::GetBodyB, return_internal_reference<>()))
            .add_property("user_data", &b2Joint::GetUserData, &b2Joint::SetUserData)
            .add_property("active", &b2Joint::IsActive)
        ;
    }

    void wrap_revolute_joint()
    {
        class_<b2RevoluteJoint, bases<b2Joint> >("RevoluteJoint", no_init)
            .add_property("anchor_a", &b2RevoluteJoint::GetAnchorA)
            .add_property("anchor_b", &b2RevoluteJoint::GetAnchorB)
            .add_property("reaction_force", &b2RevoluteJoint::GetReactionForce)
            .add_property("reaction_torque", &b2RevoluteJoint::GetReactionTorque)
            .add_property("joint_angle", &b2RevoluteJoint::GetJointAngle)
            .add_property("joint_speed", &b2RevoluteJoint::GetJointSpeed)
            .add_property("limit_enabled", &b2RevoluteJoint::IsLimitEnabled, &b2RevoluteJoint::EnableLimit)
            .add_property("lower_limit", &b2RevoluteJoint::GetLowerLimit)
            .add_property("upper_limit", &b2RevoluteJoint::GetUpperLimit)
            .add_property("motor_enabled", &b2RevoluteJoint::IsMotorEnabled, &b2RevoluteJoint::EnableMotor)
            .add_property("motor_speed", &b2RevoluteJoint::GetMotorSpeed, &b2RevoluteJoint::SetMotorSpeed)
            .add_property("motor_torque", &b2RevoluteJoint::GetMotorTorque)
            .add_property("max_motor_torque", object(), &b2RevoluteJoint::SetMaxMotorTorque)

            .def("set_limits", &b2RevoluteJoint::SetLimits)
        ;
    }

    void wrap_prismatic_joint()
    {
        class_<b2PrismaticJoint, bases<b2Joint> >("PrismaticJoint", no_init)
            .add_property("anchor_a", &b2PrismaticJoint::GetAnchorA)
            .add_property("anchor_b", &b2PrismaticJoint::GetAnchorB)
            .add_property("reaction_force", &b2PrismaticJoint::GetReactionForce)
            .add_property("reaction_torque", &b2PrismaticJoint::GetReactionTorque)
            .add_property("joint_translation", &b2PrismaticJoint::GetJointTranslation)
            .add_property("joint_speed", &b2PrismaticJoint::GetJointSpeed)
            .add_property("limit_enabled", &b2PrismaticJoint::IsLimitEnabled, &b2PrismaticJoint::EnableLimit)
            .add_property("lower_limit", &b2PrismaticJoint::GetLowerLimit)
            .add_property("upper_limit", &b2PrismaticJoint::GetUpperLimit)
            .add_property("motor_enabled", &b2PrismaticJoint::IsMotorEnabled, &b2PrismaticJoint::EnableMotor)
            .add_property("motor_speed", &b2PrismaticJoint::GetMotorSpeed, &b2PrismaticJoint::SetMotorSpeed)
            .add_property("motor_force", &b2PrismaticJoint::GetMotorForce)
            .add_property("max_motor_force", object(), &b2PrismaticJoint::SetMaxMotorForce)

            .def("set_limits", &b2PrismaticJoint::SetLimits)
        ;
    }

    void wrap_distance_joint()
    {
        class_<b2DistanceJoint, bases<b2Joint> >("DistanceJoint", no_init)
            .add_property("anchor_a", &b2DistanceJoint::GetAnchorA)
            .add_property("anchor_b", &b2DistanceJoint::GetAnchorB)
            .add_property("reaction_force", &b2PrismaticJoint::GetReactionForce)
            .add_property("reaction_torque", &b2PrismaticJoint::GetReactionTorque)
            .add_property("length", &b2DistanceJoint::GetLength, &b2DistanceJoint::SetLength)
            .add_property("frequency", &b2DistanceJoint::GetFrequency, &b2DistanceJoint::SetFrequency)
            .add_property("damping_ratio", &b2DistanceJoint::GetDampingRatio, &b2DistanceJoint::SetDampingRatio)
        ;
    }

    void wrap_pulley_joint()
    {
        class_<b2PulleyJoint, bases<b2Joint> >("PulleyJoint", no_init)
            .add_property("anchor_a", &b2PulleyJoint::GetAnchorA)
            .add_property("anchor_b", &b2PulleyJoint::GetAnchorB)
            .add_property("reaction_force", &b2PrismaticJoint::GetReactionForce)
            .add_property("reaction_torque", &b2PrismaticJoint::GetReactionTorque)
            .add_property("ground_anchor_a", &b2PulleyJoint::GetGroundAnchorA)
            .add_property("ground_anchor_b", &b2PulleyJoint::GetGroundAnchorB)
            .add_property("length_1", &b2PulleyJoint::GetLength1)
            .add_property("length_2", &b2PulleyJoint::GetLength2)
            .add_property("ratio", &b2PulleyJoint::GetRatio)
        ;
    }

    void wrap_mouse_joint()
    {
        class_<b2MouseJoint, bases<b2Joint> >("MouseJoint", no_init)
            .add_property("anchor_a", &b2MouseJoint::GetAnchorA)
            .add_property("anchor_b", &b2MouseJoint::GetAnchorB)
            .add_property("reaction_force", &b2MouseJoint::GetReactionForce)
            .add_property("reaction_torque", &b2MouseJoint::GetReactionTorque)
            .add_property("target", make_function(&b2MouseJoint::GetTarget, return_value_policy<copy_const_reference>()), &b2MouseJoint::SetTarget)
            .add_property("max_force", &b2MouseJoint::GetMaxForce, &b2MouseJoint::SetMaxForce)
            .add_property("frequency", &b2MouseJoint::GetFrequency, &b2MouseJoint::SetFrequency)
            .add_property("damping_ratio", &b2MouseJoint::GetDampingRatio, &b2MouseJoint::SetDampingRatio)
        ;
    }

    void wrap_gear_joint()
    {
        class_<b2GearJoint, bases<b2Joint> >("GearJoint", no_init)
            .add_property("anchor_a", &b2GearJoint::GetAnchorA)
            .add_property("anchor_b", &b2GearJoint::GetAnchorB)
            .add_property("reaction_force", &b2GearJoint::GetReactionForce)
            .add_property("reaction_torque", &b2GearJoint::GetReactionTorque)
            .add_property("ratio", &b2GearJoint::GetRatio, &b2GearJoint::SetRatio)
        ;
    }

    void wrap_line_joint()
    {
        class_<b2LineJoint, bases<b2Joint> >("LineJoint", no_init)
            .add_property("anchor_a", &b2LineJoint::GetAnchorA)
            .add_property("anchor_b", &b2LineJoint::GetAnchorB)
            .add_property("reaction_force", &b2LineJoint::GetReactionForce)
            .add_property("reaction_torque", &b2LineJoint::GetReactionTorque)
            .add_property("joint_translation", &b2LineJoint::GetJointTranslation)
            .add_property("joint_speed", &b2LineJoint::GetJointSpeed)
            .add_property("limit_enabled", &b2LineJoint::IsLimitEnabled, &b2LineJoint::EnableLimit)
            .add_property("lower_limit", &b2LineJoint::GetLowerLimit)
            .add_property("upper_limit", &b2LineJoint::GetUpperLimit)
            .add_property("motor_enabled", &b2LineJoint::IsMotorEnabled, &b2LineJoint::EnableMotor)
            .add_property("motor_speed", &b2LineJoint::GetMotorSpeed, &b2LineJoint::SetMotorSpeed)
            .add_property("motor_force", &b2LineJoint::GetMotorForce)
            .add_property("max_motor_force", object(), &b2LineJoint::SetMaxMotorForce)

            .def("set_limits", &b2LineJoint::SetLimits)
        ;
    }

    void wrap_weld_joint()
    {
        class_<b2WeldJoint, bases<b2Joint> >("WeldJoint", no_init)
            .add_property("anchor_a", &b2WeldJoint::GetAnchorA)
            .add_property("anchor_b", &b2WeldJoint::GetAnchorB)
            .add_property("reaction_force", &b2WeldJoint::GetReactionForce)
            .add_property("reaction_torque", &b2WeldJoint::GetReactionTorque)
        ;
    }

    void wrap_friction_joint()
    {
        class_<b2FrictionJoint, bases<b2Joint> >("FrictionJoint", no_init)
            .add_property("anchor_a", &b2FrictionJoint::GetAnchorA)
            .add_property("anchor_b", &b2FrictionJoint::GetAnchorB)
            .add_property("reaction_force", &b2FrictionJoint::GetReactionForce)
            .add_property("reaction_torque", &b2FrictionJoint::GetReactionTorque)
            .add_property("max_force", &b2FrictionJoint::GetMaxForce, &b2FrictionJoint::SetMaxForce)
            .add_property("max_torque", &b2FrictionJoint::GetMaxTorque, &b2FrictionJoint::SetMaxTorque)
        ;
    }
}
