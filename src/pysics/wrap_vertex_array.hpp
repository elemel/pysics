#ifndef PYSICS_WRAP_VERTEX_ARRAY_HPP
#define PYSICS_WRAP_VERTEX_ARRAY_HPP

#include "shared_array_range.hpp"

#include <Box2D/Common/b2Math.h>

namespace pysics {
    typedef shared_array_range<b2Vec2> vertex_array;

    void wrap_vertex_array();
}

#endif
