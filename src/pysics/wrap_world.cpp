#include <boost/python.hpp>

#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>

using namespace boost::python;

namespace pysics {
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
}
