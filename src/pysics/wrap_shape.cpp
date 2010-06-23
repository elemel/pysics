#include "wrap_shape.hpp"

#include "wrap_vertex_array.hpp"

#include <boost/python.hpp>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2LoopShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

using namespace boost::python;

namespace pysics {
    namespace {
        std::auto_ptr<b2CircleShape> construct_circle_shape(b2Vec2 position, float32 radius)
        {
            std::auto_ptr<b2CircleShape> circle_shape(new b2CircleShape);
            circle_shape->m_p = position;
            circle_shape->m_radius = radius;
            return circle_shape;
        }

        list get_polygon_shape_vertices(b2PolygonShape *polygon_shape)
        {
            list vertices;
            int32 n = polygon_shape->GetVertexCount();
            for (int32 i = 0; i != n; ++i) {
                vertices.append(polygon_shape->GetVertex(i));
            }
            return vertices;
        }

        void set_polygon_shape_vertices(b2PolygonShape *polygon_shape, list vertices)
        {
            b2Vec2 arr[b2_maxPolygonVertices];
            long n = len(vertices);
            for (long i = 0; i != n; ++i) {
                arr[i] = extract<b2Vec2 &>(vertices[i]);
            }
            polygon_shape->Set(arr, n);
        }

        std::auto_ptr<b2PolygonShape> construct_polygon_shape(list vertices)
        {
            std::auto_ptr<b2PolygonShape> polygon_shape(new b2PolygonShape);
            set_polygon_shape_vertices(polygon_shape.get(), vertices);
            return polygon_shape;
        }

        list get_loop_shape_vertices(b2LoopShape *loop_shape)
        {
            list vertices;
            for (int32 i = 0; i != loop_shape->m_count; ++i) {
                vertices.append(loop_shape->m_vertices[i]);
            }
            return vertices;
        }

        void set_loop_shape_vertices(b2LoopShape *loop_shape, VertexArray *vertices)
        {
            if (vertices->empty()) {
                loop_shape->m_vertices = 0;
                loop_shape->m_count = 0;
            } else {
                loop_shape->m_vertices = &(*vertices)[0];
                loop_shape->m_count = vertices->size();
            }
        }
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
            .add_property("type", &b2Shape::GetType)
            .def_readwrite("radius", &b2Shape::m_radius)

            .def("clone", pure_virtual(&b2Shape::Clone), return_value_policy<manage_new_object>())
            .def("get_type", &b2Shape::GetType)
            .def("get_child_count", pure_virtual(&b2Shape::GetChildCount))
            .def("test_point", pure_virtual(&b2Shape::TestPoint))
            .def("ray_cast", pure_virtual(&b2Shape::RayCast))
            .def("compute_aabb", pure_virtual(&b2Shape::ComputeAABB))
            .def("compute_mass", pure_virtual(&b2Shape::ComputeMass))
            .def("get_radius", make_getter(&b2Shape::m_radius))
            .def("set_radius", make_setter(&b2Shape::m_radius))
        ;
    }

    void wrap_circle_shape()
    {
        class_<b2CircleShape, bases<b2Shape> >("CircleShape")
            .add_property("child_count", &b2CircleShape::GetChildCount)
            .add_property("vertex_count", &b2CircleShape::GetVertexCount)
            .def_readwrite("position", &b2CircleShape::m_p)

            .def("__init__", make_constructor(construct_circle_shape))
            .def("get_child_count", &b2CircleShape::GetChildCount)
            .def("get_support", &b2CircleShape::GetSupport)
            .def("get_support_vertex", &b2CircleShape::GetSupportVertex, return_value_policy<copy_const_reference>())
            .def("get_vertex_count", &b2CircleShape::GetVertexCount)
            .def("get_vertex", &b2CircleShape::GetVertex, return_value_policy<copy_const_reference>())
            .def("get_position", make_getter(&b2CircleShape::m_p))
            .def("set_position", make_setter(&b2CircleShape::m_p))
        ;
    }

    void wrap_edge_shape()
    {
        class_<b2EdgeShape, bases<b2Shape> >("EdgeShape")
            .add_property("child_count", &b2EdgeShape::GetChildCount)
            .def_readwrite("vertex_1", &b2EdgeShape::m_vertex1)
            .def_readwrite("vertex_2", &b2EdgeShape::m_vertex2)

            .def("get_child_count", &b2EdgeShape::GetChildCount)
            .def("get_vertex_1", make_getter(&b2EdgeShape::m_vertex1))
            .def("set_vertex_1", make_setter(&b2EdgeShape::m_vertex1))
            .def("get_vertex_2", make_getter(&b2EdgeShape::m_vertex2))
            .def("set_vertex_2", make_setter(&b2EdgeShape::m_vertex2))
        ;
    }

    void wrap_polygon_shape()
    {
        class_<b2PolygonShape, bases<b2Shape> >("PolygonShape")
            .add_property("child_count", &b2PolygonShape::GetChildCount)
            .add_property("vertex_count", &b2PolygonShape::GetVertexCount)
            .add_property("vertices", &get_polygon_shape_vertices, &set_polygon_shape_vertices)

            .def("__init__", make_constructor(construct_polygon_shape))
            .def("get_child_count", &b2PolygonShape::GetChildCount)
            .def("get_vertex_count", &b2PolygonShape::GetVertexCount)
            .def("get_vertex", &b2PolygonShape::GetVertex, return_value_policy<copy_const_reference>())
            .def("get_vertices", &get_polygon_shape_vertices)
            .def("set_vertices", &set_polygon_shape_vertices)
        ;
    }

    void wrap_loop_shape()
    {
        class_<b2LoopShape, bases<b2Shape> >("LoopShape")
            .add_property("vertices", &get_loop_shape_vertices, &set_loop_shape_vertices)
            .add_property("child_count", &b2LoopShape::GetChildCount)

            .def("get_vertices", &get_loop_shape_vertices)
            .def("set_vertices", &set_loop_shape_vertices)
            .def("get_child_count", &b2LoopShape::GetChildCount)
            .def("get_child_edge", &b2LoopShape::GetChildEdge)
        ;
    }
}
