// The single translation unit that provides doctest's main(). Keeping it alone in its
// own file means the (fairly heavy) implementation is compiled once, and adding a new
// test file never triggers a rebuild of it.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
