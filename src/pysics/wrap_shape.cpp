#include <boost/noncopyable.hpp>
#include <boost/python.hpp>

#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2LoopShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

using namespace boost::python;

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
