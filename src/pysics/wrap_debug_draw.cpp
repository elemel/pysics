#include "wrap_debug_draw.hpp"

#include <boost/python.hpp>
#include <Box2D/Common/b2Math.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>

using namespace boost::python;

namespace pysics {
    namespace {
        struct DebugDrawWrapper : b2DebugDraw, wrapper<b2DebugDraw> {
            explicit DebugDrawWrapper(uint32 flags = 0)
            {
                m_drawFlags = flags;
            }
    
            void DrawPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color)
            {
                list vertex_list;
                for (int32 i = 0; i != vertexCount; ++i) {
                    vertex_list.append(vertices[i]);
                }
                this->get_override("draw_polygon")(vertex_list, make_tuple(color.r, color.g, color.b));
            }
    
            void DrawSolidPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color)
            {
                list vertex_list;
                for (int32 i = 0; i != vertexCount; ++i) {
                    vertex_list.append(vertices[i]);
                }
                this->get_override("draw_solid_polygon")(vertex_list, make_tuple(color.r, color.g, color.b));
            }
    
            void DrawCircle(const b2Vec2 &center, float32 radius, const b2Color &color)
            {
                this->get_override("draw_circle")(center, radius, make_tuple(color.r, color.g, color.b));
            }
    
            void DrawSolidCircle(const b2Vec2 &center, float32 radius, const b2Vec2 &axis, const b2Color &color)
            {
                this->get_override("draw_solid_circle")(center, radius, axis, make_tuple(color.r, color.g, color.b));
            }
    
            void DrawSegment(const b2Vec2 &p1, const b2Vec2 &p2, const b2Color &color)
            {
                this->get_override("draw_segment")(p1, p2, make_tuple(color.r, color.g, color.b));
            }
    
            void DrawTransform(const b2Transform &xf)
            {
                this->get_override("draw_transform")(xf.position, xf.GetAngle());
            }
        };
    }

    void wrap_debug_draw()
    {
        scope().attr("SHAPE_BIT") = uint32(b2DebugDraw::e_shapeBit);
        scope().attr("JOINT_BIT") = uint32(b2DebugDraw::e_jointBit);
        scope().attr("AABB_BIT") = uint32(b2DebugDraw::e_aabbBit);
        scope().attr("PAIR_BIT") = uint32(b2DebugDraw::e_pairBit);
        scope().attr("CENTER_OF_MASS_BIT") = uint32(b2DebugDraw::e_centerOfMassBit);

        class_<DebugDrawWrapper, boost::noncopyable>("DebugDraw")
            .def(init<uint32>())

            .add_property("flags", &b2DebugDraw::GetFlags, &b2DebugDraw::SetFlags)

            .def("get_flags", &b2DebugDraw::GetFlags)
            .def("set_flags", &b2DebugDraw::SetFlags)
            .def("append_flags", &b2DebugDraw::AppendFlags)
            .def("clear_flags", &b2DebugDraw::ClearFlags)
            .def("draw_polygon", pure_virtual(&b2DebugDraw::DrawPolygon))
            .def("draw_solid_polygon", pure_virtual(&b2DebugDraw::DrawSolidPolygon))
            .def("draw_circle", pure_virtual(&b2DebugDraw::DrawCircle))
            .def("draw_solid_circle", pure_virtual(&b2DebugDraw::DrawSolidCircle))
            .def("draw_segment", pure_virtual(&b2DebugDraw::DrawSegment))
            .def("draw_transform", pure_virtual(&b2DebugDraw::DrawTransform))
        ;
    }
}
