#include <vector>
#include <cstdint>
#include <cstdlib>

namespace TestFramework {
    using TestFn = void(*)();
    std::vector<TestFn> test_fns;
    std::size_t assertion_count = 0;
    std::size_t failure_count = 0;

    struct TestPushBacker {
        explicit TestPushBacker(TestFn test_fn) {
            test_fns.push_back(test_fn);
        }
    };
}

#define EXPECT_TRUE(bool_expr) \
++TestFramework::assertion_count; \
if (!(bool_expr)) { \
    std::printf("Test failed '%s' line %d, '%s' evaluated to false.\n", __func__, __LINE__, #bool_expr); \
    ++TestFramework::failure_count; \
}

#define EXPECT_EQ(x, y) \
++TestFramework::assertion_count; \
if ((x) != (y)) { \
    std::printf("Test failed '%s' line %d, '%s' and '%s' are not equal.\n", __func__, __LINE__, #x, #y); \
    ++TestFramework::failure_count; \
}

#define TEST(fn) \
static void test_##fn(); \
namespace TestFramework { \
    static const TestPushBacker test_##fn##_push_backer{test_##fn}; \
} \
static void test_##fn()

int main() {
    TestFramework::assertion_count = 0;
    TestFramework::failure_count = 0;
    for (const TestFramework::TestFn test_fn : TestFramework::test_fns) {
        test_fn();
    }

    if (TestFramework::failure_count == 0) {
        std::printf("All tests passed, %llu assertion(s) in %llu test(s).\n", TestFramework::assertion_count, TestFramework::test_fns.size());
    }

    return 0;
}
