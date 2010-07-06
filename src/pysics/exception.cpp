#include <boost/python.hpp>
#include <Box2D/Common/b2Settings.h>

using namespace boost::python;

namespace pysics {
    namespace {
        void translate_assertion_failed(const pysics::assertion_failed &e)
        {
            std::string what("assertion failed: ");
            what += e.what();
            PyErr_SetString(PyExc_AssertionError, what.c_str());
        }
    }

    void wrap_exception()
    {
        register_exception_translator<assertion_failed>(&translate_assertion_failed);
    }
}
