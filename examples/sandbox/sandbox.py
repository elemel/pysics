from contextlib import contextmanager
import math
import pinky
import pyglet
from pyglet.gl import *
import pysics
import sys
from xml.dom import minidom

@contextmanager
def manage_matrix():
    glPushMatrix()
    yield
    glPopMatrix()

@contextmanager
def manage_mode(mode):
    glBegin(mode)
    yield
    glEnd()

@contextmanager
def manage_screen_transform(width, height, scale=1.0, x=0.0, y=0.0):
    with manage_matrix():
        glTranslatef(float(width // 2), float(height // 2), 0.0)
        scale = scale * float(min(width, height))
        glScalef(scale, scale, scale)
        glTranslatef(-x, -y, 0.0)
        yield

def generate_circle_vertices(x, y, radius, vertex_count):
    for i in xrange(vertex_count):
        angle = 2.0 * math.pi * float(i) / float(vertex_count)
        yield x + radius * math.cos(angle), y + radius * math.sin(angle)

def draw_vertices(vertices, mode):
    with manage_mode(mode):
        for x, y in vertices:
            glVertex2f(x, y)

class AttributeChain(object):
    def __init__(self, attributes, next=None):
        self.attributes = attributes
        self.next = next

    def get(self, name, default=None):
        chain = self
        while True:
            if name in chain.attributes:
                return chain.attributes[name]
            chain = chain.next
            if chain is None:
                return default

def parse_bool(arg):
    if arg == 'false':
        return False
    elif arg == 'true':
        return True
    else:
        raise ValueError('invalid boolean: ' + arg)

def parse_float_tuple(arg):
    return tuple(float(s) for s in arg.replace(',', ' ').split())

def parse_id_url(arg):
    if arg == 'none':
        return None
    elif arg.startswith('url(#') and arg.endswith(')'):
        return arg[5:-1]
    else:
        raise ValueError('invalid ID URL: ' + arg)

class Loader(object):
    def add_shape(self, matrix, attribute_chain, shape):
        raise NotImplementedError()

    def load(self):
        raise NotImplementedError()

class BodyLoader(Loader):
    def __init__(self, world, bodies, body_type, attribute_chain):
        body_attributes = self.parse_body_attributes(attribute_chain)
        self.body = world.create_body(body_type, **body_attributes)
        id_ = attribute_chain.attributes.get('id')
        if id_ is not None:
            bodies[id_] = self.body

    def parse_body_attributes(self, attribute_chain):
        get = attribute_chain.get
        return dict(linear_velocity=parse_float_tuple(get('linear-velocity', '0 0')),
                    angular_velocity = float(get('angular-velocity', '0')),
                    linear_damping = float(get('linear-damping', '0')),
                    angular_damping = float(get('angular-damping', '0')),
                    allow_sleep = parse_bool(get('allow-sleep', 'true')),
                    awake = parse_bool(get('awake', 'true')),
                    fixed_rotation = parse_bool(get('fixed-rotation', 'false')),
                    bullet = parse_bool(get('bullet', 'false')),
                    active = parse_bool(get('active', 'true')),
                    inertia_scale = float(get('inertia-scale', '1')))

    def parse_fixture_attributes(self, attribute_chain):
        get = attribute_chain.get
        return dict(friction=float(get('friction', '0.2')),
                    restitution=float(get('restitution', '0')),
                    density=float(get('density', '1')),
                    sensor=parse_bool(get('sensor', 'false')),
                    category_bits=int(get('category-bits', '0001'), 16),
                    mask_bits=int(get('mask-bits', 'ffff'), 16),
                    group_index=int(get('group-index', '0')))

    def add_shape(self, matrix, attribute_chain, shape):
        shape = shape.transform(matrix)
        fixture_attributes = self.parse_fixture_attributes(attribute_chain)
        if isinstance(shape, pinky.Circle):
            self.body.create_circle_fixture(position=(shape.cx, shape.cy),
                                            radius=shape.r,
                                            **fixture_attributes)
        elif isinstance(shape, pinky.Polygon):
            if shape.area >= 0.0:
                vertices = shape.points
            else:
                vertices = list(reversed(shape.points))
            self.body.create_polygon_fixture(vertices=vertices,
                                             **fixture_attributes)

    def load(self):
        pass

class JointLoader(Loader):
    def __init__(self, world, bodies):
        self.world = world
        self.bodies = bodies
        self.shapes = []

    def add_shape(self, matrix, attribute_chain, shape):
        if isinstance(shape, pinky.Path):
            for basic_shape in shape.basic_shapes:
                self.shapes.append(basic_shape.transform(matrix))
        else:
            self.shapes.append(shape.transform(matrix))

class RevoluteJointLoader(JointLoader):
    def __init__(self, world, bodies, attribute_chain):
        super(RevoluteJointLoader, self).__init__(world, bodies)
        self.body_a_id = parse_id_url(attribute_chain.get('body-a', 'none'))
        self.body_b_id = parse_id_url(attribute_chain.get('body-b', 'none'))
        self.joint_attributes = self.parse_joint_attributes(attribute_chain)

    def load(self):
        if len(self.shapes) != 1 or not isinstance(self.shapes[0], pinky.Circle):
            raise TypeError('invalid shapes for revolute joint')
        anchor = self.shapes[0].cx, self.shapes[0].cy
        bodies = [self.bodies[id_]
                  for id_ in (self.body_a_id, self.body_b_id)
                  if id_ is not None]
        if len(bodies) < 2:
            for fixture in self.world.query_aabb(anchor, anchor):
                if fixture.body not in bodies:
                    bodies.append(fixture.body)
            if len(bodies) < 2:
                raise ValueError('not enough bodies at anchor')
            elif len(bodies) > 2:
                raise ValueError('too many bodies at anchor')
            bodies = bodies[:2]
        body_a, body_b = bodies
        self.world.create_revolute_joint(body_a=body_a,
                                         body_b=body_b,
                                         anchor=anchor,
                                         **self.joint_attributes)

    def parse_joint_attributes(self, attribute_chain):
        get = attribute_chain.get
        return dict(limit_enabled=parse_bool(get('limit-enabled', 'false')),
                    lower_angle=float(get('lower-angle', '0')),
                    upper_angle=float(get('upper-angle', '0')),
                    motor_enabled=parse_bool(get('motor-enabled', 'false')),
                    motor_speed=float(get('motor-speed', '0')),
                    max_motor_torque=float(get('max-motor-torque', '0')),
                    collide_connected=parse_bool(get('collide-connected', 'false')))

def create_static_body_loader(world, bodies, attribute_chain):
    return BodyLoader(world, bodies, pysics.STATIC_BODY, attribute_chain)

def create_kinematic_body_loader(world, bodies, attribute_chain):
    return BodyLoader(world, bodies, pysics.KINEMATIC_BODY, attribute_chain)

def create_dynamic_body_loader(world, bodies, attribute_chain):
    return BodyLoader(world, bodies, pysics.DYNAMIC_BODY, attribute_chain)

class DocumentLoader(object):
    loader_factories = {
        'static-body': create_static_body_loader,
        'kinematic-body': create_kinematic_body_loader,
        'dynamic-body': create_dynamic_body_loader,

        'revolute-joint': RevoluteJointLoader,
        'prismatic-joint': None,
        'distance-joint': None,
        'pulley-joint': None,
        'mouse-joint': None,
        'gear-joint': None,
        'line-joint': None,
        'weld-joint': None,
        'friction-joint': None,
    }

    def __init__(self, path, world):
        self.path = path
        self.world = world
        self.bodies = {}
        self.loader = None
        self.loaders = []

    def load(self):
        document = minidom.parse(self.path)
        root = document.documentElement
        width = float(root.getAttribute('width'))
        height = float(root.getAttribute('height'))
        matrix = (pinky.Matrix.create_scale(0.01, -0.01) *
                  pinky.Matrix.create_translate(-0.5 * width, -0.5 * height))
        self.load_element(root, matrix, AttributeChain({}))
        for loader in self.loaders:
            loader.load()

    def load_element(self, element, matrix, attribute_chain):
        local_matrix = pinky.Matrix.from_string(element.getAttribute('transform'))
        matrix = matrix * local_matrix
        attributes = self.get_attributes(element)
        attribute_chain = AttributeChain(attributes, attribute_chain)
        with self.manage_loader(attribute_chain):
            self.load_shape(element, matrix, attribute_chain)
            for child in element.childNodes:
                if child.nodeType == child.ELEMENT_NODE:
                    self.load_element(child, matrix, attribute_chain)

    def load_shape(self, element, matrix, attribute_chain):
        shape = pinky.parse_shape(element)
        if shape is not None:
            loader = self.loader
            if loader is None:
                loader = BodyLoader(self.world, self.bodies,
                                    pysics.STATIC_BODY, attribute_chain)
                self.loaders.append(loader)
            loader.add_shape(matrix, attribute_chain, shape)

    def get_attributes(self, element):
        attributes = pinky.parse_style(element.getAttribute('style'))
        attribute_nodes = element.attributes
        for i in xrange(attribute_nodes.length):
            attribute_node = attribute_nodes.item(i)
            attributes[attribute_node.localName] = attribute_node.nodeValue
        for child in element.childNodes:
            if (child.nodeType == child.ELEMENT_NODE and
                child.namespaceURI == pinky.SVG_NAMESPACE and
                child.localName == 'desc'):
                if child.firstChild.nodeType == child.TEXT_NODE:
                    desc_attributes = pinky.parse_style(child.firstChild.nodeValue)
                    attributes.update(desc_attributes)
                break
        return attributes

    @contextmanager
    def manage_loader(self, attribute_chain):
        type_ = attribute_chain.attributes.get('type')
        if type_ in self.loader_factories:
            if self.loader is not None:
                raise Exception('body or joint nested within other body or joint')
            loader_factory = self.loader_factories[type_]
            loader = loader_factory(self.world, self.bodies, attribute_chain)
            self.loaders.append(loader)
            self.loader = loader
            yield
            self.loader = None
        else:
            yield

class MyDebugDraw(pysics.DebugDraw):
    def draw_polygon(self, vertices, color):
        draw_vertices(vertices, GL_LINE_LOOP)

    def draw_solid_polygon(self, vertices, color):
        draw_vertices(vertices, GL_LINE_LOOP)

    def draw_circle(self, center, radius, color):
        x, y = center
        vertices = generate_circle_vertices(x, y, radius, 16)
        draw_vertices(vertices, GL_LINE_LOOP)

    def draw_solid_circle(self, center, radius, axis, color):
        x, y = center
        ax, ay = axis
        vertices = generate_circle_vertices(x, y, radius, 16)
        draw_vertices(vertices, GL_LINE_LOOP)
        draw_vertices([(x, y), (x + radius * ax, y + radius * ay)], GL_LINES)

    def draw_segment(self, p1, p2, color):
        draw_vertices([p1, p2], GL_LINES)

    def draw_transform(self, position, angle):
        x, y = position
        glPopMatrix()
        glPushMatrix()
        glTranslatef(x, y, 0.0)
        glRotatef(angle * 180.0 / math.pi, 0.0, 0.0, 1.0)

class MyWindow(pyglet.window.Window):
    def __init__(self, paths, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        self.screen_time = 0.0
        self.world_time = 0.0
        self.world_dt = 1.0 / 60.0
        self.world = pysics.World((0.0, -10.0), True)
        self.debug_draw = MyDebugDraw(pysics.SHAPE_BIT)
        self.world.debug_draw = self.debug_draw
        for path in paths:
            DocumentLoader(path, self.world).load()
        self.clock_display = pyglet.clock.ClockDisplay()
        pyglet.clock.schedule_interval(self.step, 0.1 * self.world_dt)

    def close(self):
        pyglet.clock.unschedule(self.step)
        super(MyWindow, self).close()

    def step(self, dt):
        self.screen_time += dt
        while self.world_time + self.world_dt <= self.screen_time:
            self.world_time += self.world_dt
            self.world.step(self.world_dt, 10, 10)

    def on_draw(self):
        self.clear()
        with manage_screen_transform(self.width, self.height, 0.05):
            glPushMatrix()
            self.world.draw_debug_data()
            glPopMatrix()
        self.clock_display.draw()

def main():
    config = pyglet.gl.Config(double_buffer=True, sample_buffers=1, samples=4,
                              depth_size=8)
    window = MyWindow(sys.argv[1:], fullscreen=True, config=config)
    pyglet.app.run()

if __name__ == '__main__':
    main()
