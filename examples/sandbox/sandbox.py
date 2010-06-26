from contextlib import contextmanager
import math
import pinky
import pyglet
from pyglet.gl import *
import pysics
import sys

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

def load_document(document, world):
    width = float(document.root.attributes['width'])
    height = float(document.root.attributes['height'])
    matrix = pinky.Matrix.create_scale(0.01, -0.01) * pinky.Matrix.create_translate(-0.5 * width, -0.5 * height)
    load_element(document.root, matrix, {}, world, None)

def parse_float_tuple(float_tuple_str):
    return tuple(float(s) for s in float_tuple_str.replace(',', ' ').split())

def parse_bool(bool_str):
    if bool_str == 'false':
        return False
    elif bool_str == 'true':
        return True
    else:
        raise ValueError('invalid boolean: %s' % bool_str)

def parse_body_attributes(attributes):
    return dict(linear_velocity=parse_float_tuple(attributes.get('linear-velocity', '0 0')),
                angular_velocity = float(attributes.get('angular-velocity', '0')),
                linear_damping = float(attributes.get('linear-damping', '0')),
                angular_damping = float(attributes.get('angular-damping', '0')),
                allow_sleep = parse_bool(attributes.get('allow-sleep', 'true')),
                awake = parse_bool(attributes.get('awake', 'true')),
                fixed_rotation = parse_bool(attributes.get('fixed-rotation', 'false')),
                bullet = parse_bool(attributes.get('bullet', 'false')),
                active = parse_bool(attributes.get('active', 'true')),
                inertia_scale = float(attributes.get('inertia-scale', '1')))

def parse_shape_attributes(attributes):
    return dict(friction=float(attributes.get('friction', '0.2')),
                restitution=float(attributes.get('restitution', '0')),
                density=float(attributes.get('density', '1')),
                sensor=parse_bool(attributes.get('sensor', 'false')),
                category_bits=int(attributes.get('category-bits', '0001'), 16),
                mask_bits=int(attributes.get('mask-bits', 'ffff'), 16),
                group_index=int(attributes.get('group-index', '0')))

BODY_TYPES = {'static-body': pysics.STATIC_BODY,
              'kinematic-body': pysics.KINEMATIC_BODY,
              'dynamic-body': pysics.DYNAMIC_BODY}

def load_element(element, matrix, attributes, world, body):
    matrix = matrix * element.matrix
    local_attributes = dict(element.attributes)
    for attribute in ('style', 'desc'):
        local_attributes.update(pinky.parse_style(local_attributes.pop(attribute, '')))
    attributes = dict(attributes)
    attributes.update(local_attributes)
    type_ = local_attributes.get('type')
    if type_ in BODY_TYPES or type_ is None and body is None and element.shape is not None:
        if type_ is None:
            body_type = pysics.STATIC_BODY
        else:
            body_type = BODY_TYPES[type_]
        body_attributes = parse_body_attributes(attributes)
        body = world.create_body(type=body_type, **body_attributes)
    if body is not None and element.shape is not None:
        transformed_shape = element.shape.transform(matrix)
        shape_attributes = parse_shape_attributes(attributes)
        if isinstance(transformed_shape, pinky.Circle):
            body.create_circle_fixture(position=(transformed_shape.cx,
                                                 transformed_shape.cy),
                                       radius=transformed_shape.r,
                                       **shape_attributes)
        elif isinstance(transformed_shape, pinky.Polygon):
            if transformed_shape.area >= 0.0:
                vertices = transformed_shape.points
            else:
                vertices = list(reversed(transformed_shape.points))
            body.create_polygon_fixture(vertices=vertices, **shape_attributes)
    for child in element.children:
        load_element(child, matrix, attributes, world, body)

class MyWindow(pyglet.window.Window):
    def __init__(self, documents, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        self.screen_time = 0.0
        self.world_time = 0.0
        self.world_dt = 1.0 / 60.0
        self.world = pysics.World((0.0, -10.0), True)
        for document in documents:
            load_document(document, self.world)
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
    documents = [pinky.Document(arg) for arg in sys.argv[1:]]
    config = pyglet.gl.Config(double_buffer=True, sample_buffers=1, samples=4,
                              depth_size=8)
    window = MyWindow(documents, fullscreen=True, config=config)
    pyglet.app.run()

if __name__ == '__main__':
    main()
