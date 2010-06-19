#include "shared_slice.hpp"

#include <algorithm>
#include <cassert>

const int int_array[] = { 234, 36, 24, 465, 325, 52};
const std::size_t int_array_size = sizeof(int_array) / sizeof(int_array[0]);

void test_default_constructor()
{
    pysics::shared_slice<int> int_slice;
    assert(int_slice.begin() == int_slice.end());
    assert(int_slice.size() == 0);
    assert(int_slice.empty());
}

void test_input_range_constructor()
{
    pysics::shared_slice<int> int_slice(int_array,
                                        int_array + int_array_size,
                                        int_array_size);
    assert(int_slice.end() - int_slice.begin() == int_array_size);
    assert(!int_slice.empty());
    assert(int_slice.size() == int_array_size);
    assert(std::equal(int_array, int_array + int_array_size,
                      int_slice.begin()));
}

void test_random_access_range_constructor()
{
    pysics::shared_slice<int> int_slice(int_array,
                                        int_array + int_array_size);
    assert(int_slice.end() - int_slice.begin() == int_array_size);
    assert(!int_slice.empty());
    assert(int_slice.size() == int_array_size);
    assert(std::equal(int_array, int_array + int_array_size,
                      int_slice.begin()));
}

int main()
{
    test_default_constructor();
    test_input_range_constructor();
    test_random_access_range_constructor();
    return 0;
}
