from contextlib import contextmanager
import math
import pyglet
from pyglet.gl import *
import pysics

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
        draw_circle(x, y, shape.radius)
    elif shape.type == pysics.EDGE_SHAPE:
        pass
    elif shape.type == pysics.POLYGON_SHAPE:
        draw_polygon(shape.vertices)
    elif shape.type == pysics.LOOP_SHAPE:
        pass

def generate_circle_vertices(x, y, radius, vertex_count=16):
    for i in xrange(vertex_count):
        angle = 2.0 * math.pi * float(i) / float(vertex_count)
        yield math.cos(angle), math.sin(angle)

def draw_circle(x, y, radius, vertex_count=16, mode=GL_LINE_LOOP):
    vertices = generate_circle_vertices(x, y, radius, vertex_count)
    draw_polygon(vertices, mode)

def draw_polygon(vertices, mode=GL_LINE_LOOP):
    with manage_mode(mode):
        for x, y in vertices:
            glVertex2f(x, y)

class MyWindow(pyglet.window.Window):
    def __init__(self, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        self.screen_time = 0.0
        self.world_time = 0.0
        self.world_dt = 1.0 / 60.0
        self.world = pysics.World((0.0, -10.0), True)
        body = self.world.create_dynamic_body(angular_velocity=5.0)
        body.create_box_fixture()
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
        with manage_screen_transform(self.width, self.height, 0.1):
            for body in iter_links(self.world.body_list):
                draw_body(body)
        self.clock_display.draw()

def main():
    window = MyWindow(fullscreen=True)
    pyglet.app.run()

if __name__ == '__main__':
    main()
