from pysics_box2d import *

__all__ = ['Body', 'Circle', 'Polygon', 'World']

class World(object):
    def __init__(self, gravity, do_sleep):
        self._b2_world = b2World(b2Vec2(*gravity), do_sleep)
        # Body(world=self, ground=True)

    def delete(self):
        for joint in self.joints:
            joint._delete()
        for body in self.bodies:
            body._delete()
        # TODO: destroy Box2D world
        self._b2_world = None

    @property
    def physical(self):
        return self._b2_world is not None

    @property
    def bodies(self):
        assert self.physical
        return [b2_body.userData for b2_body in self._b2_world.bodyList]

    @property
    def joints(self):
        assert self.physical
        return [b2_joint.userData for b2_joint in self._b2_world.jointList]

class Body(object):
    def __init__(self, world, shapes=(), ground=False):
        assert isinstance(world, World)
        self._world = world
        if ground:
            b2_body = self._world._b2_world.groundBody
            assert b2_body.userData is None
        else:
            b2_body_def = b2BodyDef()
            b2_body = self._world._b2_world.CreateBody(b2_body_def)
        self._b2_body = b2_body
        self._b2_body.userData = self
        for shape in shapes:
            assert isinstance(shape, Shape)
            shape.body = self

    def delete(self):
        if self.physical:
            self._delete()
            self._world._b2_world.DestroyBody(self._b2_body)

    def _delete(self):
        assert self.physical
        for shape in self.shapes:
            shape._delete()
        self._world = None
        self._b2_body = None

    @property
    def physical(self):
        return self._b2_body is not None

    @property
    def shapes(self):
        assert self.physical
        return [b2_shape.userData for b2_shape in self._b2_body.shapeList]

class Shape(object):
    density = 0.0
    friction = 0.5
    restitution = 0.2
    category_bits = 1
    mask_bits = -1
    group_index = 0

    def __init__(self, body=None, **kwargs):
        self._body = None
        for key, value in kwargs.iteritems():
            assert hasattr(self, key)
            setattr(self, key, value)
        self.body = body

    def delete(self):
        if self.physical:
            self._delete()
            self._body._b2_body.DestroyShape(self)

    def _delete(self):
        assert self.physical
        self._b2_shape = None
        self._body = None

    @property
    def physical(self):
        return self._b2_shape is not None

    def _get_body(self):
        return self._body

    def _set_body(self, body):
        assert body is None or isinstance(body, Body) and body.physical
        if self._body is not None:
            self._body._b2_body.DestroyShape(self._b2_shape)
            self._b2_shape = None
            self._body._b2_body.SetMassFromShapes()
        self._body = body
        if self._body is not None:
            b2_shape_def = self._create_b2_shape_def()
            b2_shape_def.density = self.density
            b2_shape_def.friction = self.friction
            b2_shape_def.restitution = self.restitution
            self._b2_shape = self._body._b2_body.CreateShape(b2_shape_def)
            self._b2_shape.userData = self
            self._body._b2_body.SetMassFromShapes()

    body = property(_get_body, _set_body)

class Circle(Shape):
    def __init__(self, center, radius, **kwargs):
        self.center = center
        self.radius = radius
        Shape.__init__(self, **kwargs)

    def _create_b2_shape_def(self):
        b2_shape_def = b2CircleDef()
        b2_shape_def.localPosition = self.center
        b2_shape_def.radius = self.radius
        return b2_shape_def

class Polygon(Shape):
    def __init__(self, vertices, **kwargs):
        self.vertices = vertices
        Shape.__init__(self, **kwargs)

    def _create_b2_shape_def(self):
        b2_shape_def = b2PolygonDef()
        b2_shape_def.vertices = self.vertices
        return b2_shape_def
