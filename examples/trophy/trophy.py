import math
import pyglet
from pyglet.gl import *
import pysics

def iter_links(link):
    while link is not None:
        yield link
        link = link.next

class MyWindow(pyglet.window.Window):
    def __init__(self, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        self.screen_time = 0.0
        self.world_time = 0.0
        self.world_dt = 1.0 / 60.0
        self.scale = 20.0
        gravity = pysics.Vec2(0.0, -10.0)
        self.world = pysics.World(gravity, True)
        self.create_circle_body()
        pyglet.clock.schedule_interval(self.step, self.world_dt)

    def create_circle_body(self, position=pysics.Vec2(0.0, 0.0), radius=1.0):
        body_def = pysics.BodyDef()
        body_def.type = pysics.DYNAMIC_BODY
        body = self.world.create_body(body_def)
        fixture_def = pysics.FixtureDef()
        circle_shape = pysics.CircleShape()
        circle_shape.position = position
        circle_shape.radius = radius
        fixture_def.shape = circle_shape
        fixture_def.density = 1.0
        body.create_fixture(fixture_def)
        return body

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
        glPushMatrix()
        glTranslatef(float(self.width // 2), float(self.height // 2), 0.0)
        glScalef(self.scale, self.scale, self.scale)
        self.debug_draw()
        glPopMatrix()

    def debug_draw(self):
        for body in iter_links(self.world.body_list):
            self.debug_draw_body(body)

    def debug_draw_body(self, body):
        glPushMatrix()
        x, y = body.position
        glTranslatef(x, y, 0.0)
        glRotatef(body.angle * 180.0 / math.pi, 0.0, 0.0, 1.0)
        for fixture in iter_links(body.fixture_list):
            self.debug_draw_fixture(fixture)
        glPopMatrix()

    def debug_draw_fixture(self, fixture):
        if fixture.type == pysics.CIRCLE_SHAPE:
            self.debug_draw_circle_shape(fixture.shape)
        elif fixture.type == pysics.POLYGON_SHAPE:
            self.debug_draw_polygon_shape(fixture.shape)
        else:
            print 'debug draw %r' % fixture

    def debug_draw_circle_shape(self, circle_shape):
        glPushMatrix()
        x, y = circle_shape.position
        glTranslatef(x, y, 0.0)
        scale = circle_shape.radius
        glScalef(scale, scale, scale)
        vertex_count = 16
        glBegin(GL_LINE_LOOP)
        for i in xrange(vertex_count):
            angle = 2 * math.pi * float(i) / float(vertex_count)
            vx = math.cos(angle)
            vy = math.sin(angle)
            glVertex2f(vx, vy)
        glEnd()
        glPopMatrix()

    def debug_draw_polygon_shape(self, polygon_shape):
        print 'debug draw %r' % polygon_shape

def main():
    window = MyWindow()
    pyglet.app.run()

if __name__ == '__main__':
    main()
