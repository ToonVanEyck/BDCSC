#include "rbuff.h"

#include "unity.h"

void setUp(void)
{
    // This function is called before each test
}

void tearDown(void)
{
    // This function is called after each test
}

void test_rbuff_init(void)
{
    uint8_t buffer[10];
    rbuff_t rbuff;
    rbuff_init(&rbuff, buffer, 10, sizeof(buffer[0]));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_rbuff_init);

    return UNITY_END();
}