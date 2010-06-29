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

class AttributeStack(object):
    def __init__(self):
        self.stack = [{}]

    def push(self, attributes):
        self.stack.append(attributes)

    def pop(self):
        return self.stack.pop()

    def peek(self):
        return self.stack[-1]

    def get(self, name, default=None):
        for attributes in reversed(self.stack):
            if name in attributes:
                return attributes[name]
        return default

class DocumentLoader(object):
    body_types = {'static-body': pysics.STATIC_BODY,
                  'kinematic-body': pysics.KINEMATIC_BODY,
                  'dynamic-body': pysics.DYNAMIC_BODY}

    def __init__(self, path, world):
        self.path = path
        self.world = world
        self.helper = pinky.DOMHelper()
        self.attribute_stack = AttributeStack()
        self.body = None
        self.joint_creator = None
        self.joint_creators = []

    def load(self):
        root = self.helper.load(self.path)
        width = float(self.helper.get_attribute(root, pinky.SVG_NAMESPACE, 'width'))
        height = float(self.helper.get_attribute(root, pinky.SVG_NAMESPACE, 'height'))
        matrix = pinky.Matrix.create_scale(0.01, -0.01) * pinky.Matrix.create_translate(-0.5 * width, -0.5 * height)
        self.load_element(root, matrix)

    def load_element(self, element, matrix):
        local_matrix = pinky.Matrix.from_string(self.helper.get_attribute(element, pinky.SVG_NAMESPACE, 'transform', ''))
        matrix = matrix * local_matrix
        with self.manage_attributes(element):
            with self.manage_body_or_joint_creator():
                self.load_shape(element, matrix)
                for child in self.helper.get_children(element):
                    self.load_element(child, matrix)

    def load_shape(self, element, matrix):
        shape = self.helper.parse_shape(element)
        if shape is not None:
            if self.joint_creator is None:
                body = self.body
                if body is None:
                    body = self.world.create_static_body()
                self.load_body_shape(matrix, body, shape)
            else:
                pass

    def load_body_shape(self, matrix, body, shape):
        shape = shape.transform(matrix)
        fixture_attributes = self.parse_fixture_attributes()
        if isinstance(shape, pinky.Circle):
            body.create_circle_fixture(position=(shape.cx, shape.cy),
                                       radius=shape.r,
                                       **fixture_attributes)
        elif isinstance(shape, pinky.Polygon):
            if shape.area >= 0.0:
                vertices = shape.points
            else:
                vertices = list(reversed(shape.points))
            body.create_polygon_fixture(vertices=vertices,
                                        **fixture_attributes)

    @contextmanager
    def manage_attributes(self, element):
        attributes = {}
        for child in self.helper.get_children(element):
            if self.helper.get_namespace_and_name(child) == (pinky.SVG_NAMESPACE, 'desc'):
                desc_attributes = pinky.parse_css_attributes(self.helper.get_text(child))
                attributes.update(desc_attributes)
        self.attribute_stack.push(attributes)
        yield
        self.attribute_stack.pop()

    @contextmanager
    def manage_body_or_joint_creator(self):
        type_ = self.attribute_stack.peek().get('type')
        if type_ in self.body_types:
            if self.body is not None or self.joint_creator is not None:
                raise Exception('nested bodies or joints')
            body_type = self.body_types[type_]
            body_attributes = self.parse_body_attributes()
            self.body = self.world.create_body(body_type, **body_attributes)
            yield
            self.body = None
        else:
            yield

    def parse_body_attributes(self):
        get = self.attribute_stack.get
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

    def parse_fixture_attributes(self):
        get = self.attribute_stack.get
        return dict(friction=float(get('friction', '0.2')),
                    restitution=float(get('restitution', '0')),
                    density=float(get('density', '1')),
                    sensor=self.parse_bool(get('sensor', 'false')),
                    category_bits=int(get('category-bits', '0001'), 16),
                    mask_bits=int(get('mask-bits', 'ffff'), 16),
                    group_index=int(get('group-index', '0')))

    def parse_float_tuple(self, arg):
        return tuple(float(s) for s in arg.replace(',', ' ').split())

    def parse_bool(self, arg):
        if arg == 'false':
            return False
        elif arg == 'true':
            return True
        else:
            raise ValueError('invalid boolean: %s' % bool_str)

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
