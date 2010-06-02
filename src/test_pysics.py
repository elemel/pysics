from pysics import *

def main():
    world = World((0, -10), True)
    # polygon = Polygon([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)])
    # body = Body(world=world, shapes=[polygon])
    # circle = Circle((0., 0.), 1., body=body)
    # world.delete()
    # assert not world.physical
    # assert not polygon.physical

if __name__ == '__main__':
    main()
