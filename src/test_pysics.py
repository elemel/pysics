from pysics import *

def test_create_world():
    world = World(Vec2(0.0, -10.0), True)

def test_body_types_are_distinct():
    body_types = [STATIC_BODY, KINEMATIC_BODY, DYNAMIC_BODY]
    assert len(body_types) == len(set(body_types))

def test_user_data_of_body_keeps_identity():
    world = World(Vec2(0.0, -10.0), True)
    body_def = BodyDef()
    body = world.create_body(body_def)
    obj = object()
    obj_repr = repr(obj)
    body.user_data = obj
    del obj
    assert repr(body.user_data) == obj_repr

def test_body_can_have_none_as_user_data():
    world = World(Vec2(0.0, -10.0), True)
    body_def = BodyDef()
    body = world.create_body(body_def)
    assert body.user_data is None
    body.user_data = object()
    assert body.user_data is not None
    body.user_data = None
    assert body.user_data is None

def main():
    test_create_world()
    test_body_types_are_distinct()
    test_user_data_of_body_keeps_identity()
    test_body_can_have_none_as_user_data()

if __name__ == '__main__':
    main()
