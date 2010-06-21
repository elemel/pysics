from contextlib import contextmanager
import nose
from pysics import *

@contextmanager
def manage_world():
    world = World(Vec2(0, -10), True)
    yield world

@contextmanager
def manage_body():
    with manage_world() as world:
        body = world.create_body()
        yield body
        world.destroy_body(body)

@contextmanager
def manage_circle_fixture():
    with manage_body() as body:
        circle_fixture = body.create_circle_fixture()
        yield circle_fixture
        body.destroy_fixture(circle_fixture)

@contextmanager
def manage_edge_fixture():
    with manage_body() as body:
        edge_fixture = body.create_edge_fixture()
        yield edge_fixture
        body.destroy_fixture(edge_fixture)

@contextmanager
def manage_polygon_fixture():
    with manage_body() as body:
        polygon_fixture = body.create_polygon_fixture()
        yield polygon_fixture
        body.destroy_fixture(polygon_fixture)

@contextmanager
def manage_loop_fixture():
    with manage_body() as body:
        loop_fixture = body.create_loop_fixture(None)
        yield loop_fixture
        body.destroy_fixture(loop_fixture)

def test_body_types_are_distinct():
    body_types = [STATIC_BODY, KINEMATIC_BODY, DYNAMIC_BODY]
    assert len(body_types) == len(set(body_types))

def test_user_data_of_body_keeps_identity():
    with manage_body() as body:
        obj = object()
        obj_repr = repr(obj)
        body.user_data = obj
        del obj
        assert repr(body.user_data) == obj_repr

def test_body_can_have_none_as_user_data():
    with manage_body() as body:
        assert body.user_data is None
        body.user_data = object()
        assert body.user_data is not None
        body.user_data = None
        assert body.user_data is None

def test_default_initialized_vec_2_is_zero():
    x, y = Vec2()
    assert not x and not y

def test_vec_2_has_length_2():
    assert len(Vec2(0, 0)) == 2

def test_exercise():
    with manage_circle_fixture() as fixture:
        pass
    with manage_edge_fixture() as fixture:
        pass
    with manage_polygon_fixture() as fixture:
        pass
    with manage_loop_fixture() as fixture:
        pass

def test_vec_2_unpacks_with_length_2():
    x, y = Vec2(0, 0)

if __name__ == '__main__':
    nose.main()
