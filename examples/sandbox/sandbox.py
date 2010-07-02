from contextlib import contextmanager
import math
import pinky
import pyglet
from pyglet.gl import *
import pysics
import sys
from xml.dom import minidom

def iter_links(link):
    while link is not None:
        next = link.next
        yield link
        link = next

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

@contextmanager
def manage_body_transform(body):
    with manage_matrix():
        x, y = body.position
        glTranslatef(x, y, 0.0)
        glRotatef(body.angle * 180.0 / math.pi, 0.0, 0.0, 1.0)
        yield

def draw_body(body):
    with manage_body_transform(body):
        for fixture in iter_links(body.fixture_list):
            draw_shape(fixture.shape)

def draw_shape(shape):
    if shape.type == pysics.CIRCLE_SHAPE:
        x, y = shape.position
        vertices = generate_circle_vertices(x, y, shape.radius, 16)
        draw_vertices(vertices, GL_LINE_LOOP)
        draw_vertices([(x, y), (x + shape.radius, y)], GL_LINES)
    elif shape.type == pysics.EDGE_SHAPE:
        pass
    elif shape.type == pysics.POLYGON_SHAPE:
        draw_vertices(shape.vertices, GL_LINE_LOOP)
    elif shape.type == pysics.LOOP_SHAPE:
        pass

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

class BodyLoader(object):
    def __init__(self, world, bodies, body_type, attribute_chain):
        body_attributes = self.parse_body_attributes(attribute_chain)
        self.body = world.create_body(body_type, **body_attributes)
        id_ = attribute_chain.attributes.get('id')
        if id_ is not None:
            bodies[id_] = self.body

    def parse_body_attributes(self, attribute_chain):
        get = attribute_chain.get
        return dict(linear_velocity=self.parse_float_tuple(get('linear-velocity', '0 0')),
                    angular_velocity = float(get('angular-velocity', '0')),
                    linear_damping = float(get('linear-damping', '0')),
                    angular_damping = float(get('angular-damping', '0')),
                    allow_sleep = self.parse_bool(get('allow-sleep', 'true')),
                    awake = self.parse_bool(get('awake', 'true')),
                    fixed_rotation = self.parse_bool(get('fixed-rotation', 'false')),
                    bullet = self.parse_bool(get('bullet', 'false')),
                    active = self.parse_bool(get('active', 'true')),
                    inertia_scale = float(get('inertia-scale', '1')))

    def parse_fixture_attributes(self, attribute_chain):
        get = attribute_chain.get
        return dict(friction=float(get('friction', '0.2')),
                    restitution=float(get('restitution', '0')),
                    density=float(get('density', '1')),
                    sensor=self.parse_bool(get('sensor', 'false')),
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

    def parse_float_tuple(self, arg):
        return tuple(float(s) for s in arg.replace(',', ' ').split())

    def parse_bool(self, arg):
        if arg == 'false':
            return False
        elif arg == 'true':
            return True
        else:
            raise ValueError('invalid boolean: %s' % bool_str)


class JointLoader(object):
    def __init__(self, world, bodies):
        self.world = world
        self.bodies = bodies
        self.shapes = []

    def parse_id_url(self, arg):
        if arg.startswith('url(#') and arg.endswith(')'):
            return arg[5:-1]
        else:
            raise ValueError('invalid ID URL: ' + arg)

    def add_shape(self, matrix, shape):
        if isinstance(shape, pinky.Path):
            for basic_shape in shape.basic_shapes:
                self.shapes.append(basic_shape.transform(matrix))
        else:
            self.shapes.append(shape.transform(matrix))

class RevoluteJointLoader(JointLoader):
    def __init__(self, world, bodies, attribute_chain):
        super(RevoluteJointLoader, self).__init__(world, bodies)
        self.body_a_id = self.parse_id_url(attribute_chain.get('body-a'))
        self.body_b_id = self.parse_id_url(attribute_chain.get('body-b'))

    def create(self):
        if len(self.shapes) != 1 or not isinstance(self.shapes[0], pinky.Circle):
            raise TypeError('invalid shapes for revolute joint')
        body_a = self.bodies[self.body_a_id]
        body_b = self.bodies[self.body_b_id]
        anchor = self.shapes[0].cx, self.shapes[0].cy
        self.world.create_revolute_joint(body_a=body_a,
                                         body_b=body_b,
                                         anchor=anchor)

    def parse_joint_attributes(self):
        pass

class DocumentLoader(object):
    body_types = {
        'static-body': pysics.STATIC_BODY,
        'kinematic-body': pysics.KINEMATIC_BODY,
        'dynamic-body': pysics.DYNAMIC_BODY,
    }

    joint_types = {
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
        self.body_loader = None
        self.bodies = {}
        self.joint_loader = None
        self.joint_loaders = []

    def load(self):
        document = minidom.parse(self.path)
        root = document.documentElement
        width = float(root.getAttribute('width'))
        height = float(root.getAttribute('height'))
        matrix = (pinky.Matrix.create_scale(0.01, -0.01) *
                  pinky.Matrix.create_translate(-0.5 * width, -0.5 * height))
        self.load_element(root, matrix, AttributeChain({}))
        for joint_loader in self.joint_loaders:
            joint_loader.create()

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
            if self.joint_loader is None:
                body_loader = self.body_loader
                if body_loader is None:
                    body_loader = BodyLoader(self.world, self.bodies,
                                             pysics.STATIC_BODY,
                                             attribute_chain)
                body_loader.add_shape(matrix, attribute_chain, shape)
            else:
                self.joint_loader.add_shape(matrix, shape)

    def get_attributes(self, element):
        attributes = pinky.parse_style(element.getAttribute('style'))
        attribute_nodes = element.attributes
        for i in xrange(attribute_nodes.length):
            attribute_node = attribute_nodes.item(i)
            attributes[attribute_node.localName] = attribute_node.nodeValue
        for child in element.childNodes:
            if child.nodeType == child.ELEMENT_NODE and child.namespaceURI == pinky.SVG_NAMESPACE and child.localName == 'desc':
                if child.firstChild.nodeType == child.TEXT_NODE:
                    desc_attributes = pinky.parse_style(child.firstChild.nodeValue)
                    attributes.update(desc_attributes)
                break
        return attributes

    @contextmanager
    def manage_loader(self, attribute_chain):
        type_ = attribute_chain.attributes.get('type')
        if type_ in self.body_types:
            if self.body_loader is not None or self.joint_loader is not None:
                raise Exception('body nested within body or joint')
            body_type = self.body_types[type_]
            self.body_loader = BodyLoader(self.world, self.bodies, body_type,
                                          attribute_chain)
            yield
            self.body_loader = None
        elif type_ in self.joint_types:
            if self.body_loader is not None or self.joint_loader is not None:
                raise Exception('joint nested within body or joint')
            joint_loader_factory = self.joint_types[type_]
            self.joint_loader = joint_loader_factory(self.world,
                                                     self.bodies,
                                                     attribute_chain)
            yield
            self.joint_loaders.append(self.joint_loader)
            self.joint_loader = None
        else:
            yield

class MyWindow(pyglet.window.Window):
    def __init__(self, paths, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        self.screen_time = 0.0
        self.world_time = 0.0
        self.world_dt = 1.0 / 60.0
        self.world = pysics.World((0.0, -10.0), True)
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
            for body in iter_links(self.world.body_list):
                draw_body(body)
        self.clock_display.draw()

def main():
    config = pyglet.gl.Config(double_buffer=True, sample_buffers=1, samples=4,
                              depth_size=8)
    window = MyWindow(sys.argv[1:], fullscreen=True, config=config)
    pyglet.app.run()

if __name__ == '__main__':
    main()
