#ifdef TEST

#include "unity.h"
#include <string.h>
#include "fifo.h"

char test_string_0[]  = "Hello there. \r\n";
char test_string_1[]  = "That is a test string. 123123\n";
char test_string_2[]  = "Another test string";
char test_string_3[]  = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.";
char test_string_4[]  = "Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat.";
char test_string_5[]  = "Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.";
char test_string_6[]  = "Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.";
char test_string_7[]  = "Lorem Ipsum is simply dummy text of the\n\n printing and typesetting industry.";
char test_string_8[]  = "It has survived not only five centuries, but also the leap into electronic typesetting, remaining essentially unchanged.";
char test_string_9[]  = "Contrary to popular belief, Lorem Ipsum \n is not simply random text.";
char test_string_10[] = "Lorem Ipsum comes from sections 1.10.32 and 1.10.33 of de Finibus Bonorum et Malorum by Cicero, written in 45 BC.";
char test_string_11[] = "This book is a treatise on the\t\n\r theory of ethics, very popular during the Renaissance.";
char test_string_12[] = "The first line of Lorem Ipsum, Lorem ipsum dolor sit amet.., comes from a line in section 1.10.32.";
char test_string_13[] = "The standard chunk of Lorem Ipsum used since the 1500s is reproduced below for those interested.";
char test_string_14[] = "It is a long established fact that a reader will be distracted by the readable content of a page when looking at its layout.";
char test_string_15[] = "The generated Lorem Ipsum is therefore always free from repetition, injected humour, or non-characteristic words etc.";

void setUp(void)
{
}

void tearDown(void)
{
}

void test_fifo_init(void)
{
    fifo_t fifo;
    fifo_init(&fifo);
    TEST_ASSERT_EQUAL(0, fifo.r);
    TEST_ASSERT_EQUAL(0, fifo.w);
    TEST_ASSERT_EQUAL(0, fifo.count);

    for (uint32_t i = 0; i < FIFO_DEPTH; i++) {
        for (uint32_t j = 0; j < FIFO_ELEMENT_MAX_SIZE; j++) {
            TEST_ASSERT_EQUAL(0, fifo.data[i].element[j]); 
        }
        TEST_ASSERT_EQUAL(0, fifo.size[i]);
    }
}

void test_fifo_push(void)
{
    fifo_t fifo;
    fifo_init(&fifo);

    TEST_ASSERT(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(0, fifo_count(&fifo));

    fifo_push(&fifo, test_string_0, (uint32_t)strlen(test_string_0));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(1, fifo_count(&fifo));

    fifo_push(&fifo, test_string_1, (uint32_t)strlen(test_string_1));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(2, fifo_count(&fifo));

    fifo_push(&fifo, test_string_2, (uint32_t)strlen(test_string_2));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(3, fifo_count(&fifo));

    fifo_push(&fifo, test_string_3, (uint32_t)strlen(test_string_3));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(4, fifo_count(&fifo));

    fifo_push(&fifo, test_string_4, (uint32_t)strlen(test_string_4));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(5, fifo_count(&fifo));

    fifo_push(&fifo, test_string_5, (uint32_t)strlen(test_string_5));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(6, fifo_count(&fifo));

    fifo_push(&fifo, test_string_6, (uint32_t)strlen(test_string_6));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(7, fifo_count(&fifo));

    fifo_push(&fifo, test_string_7, (uint32_t)strlen(test_string_7));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(8, fifo_count(&fifo));

    fifo_push(&fifo, test_string_8, (uint32_t)strlen(test_string_8));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(9, fifo_count(&fifo));

    fifo_push(&fifo, test_string_9, (uint32_t)strlen(test_string_9));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(10, fifo_count(&fifo));

    fifo_push(&fifo, test_string_10, (uint32_t)strlen(test_string_10));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(11, fifo_count(&fifo));

    fifo_push(&fifo, test_string_11, (uint32_t)strlen(test_string_11));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(12, fifo_count(&fifo));

    fifo_push(&fifo, test_string_12, (uint32_t)strlen(test_string_12));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(13, fifo_count(&fifo));

    fifo_push(&fifo, test_string_13, (uint32_t)strlen(test_string_13));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(14, fifo_count(&fifo));

    fifo_push(&fifo, test_string_14, (uint32_t)strlen(test_string_14));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(15, fifo_count(&fifo));

    fifo_push(&fifo, test_string_15, (uint32_t)strlen(test_string_15));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(16, fifo_count(&fifo));
}

void test_fifo_pull(void)
{
    fifo_t fifo;
    fifo_init(&fifo);

    fifo_push(&fifo, test_string_0, (uint32_t)strlen(test_string_0));
    fifo_push(&fifo, test_string_1, (uint32_t)strlen(test_string_1));
    fifo_push(&fifo, test_string_2, (uint32_t)strlen(test_string_2));
    fifo_push(&fifo, test_string_3, (uint32_t)strlen(test_string_3));
    fifo_push(&fifo, test_string_4, (uint32_t)strlen(test_string_4));
    fifo_push(&fifo, test_string_5, (uint32_t)strlen(test_string_5));
    fifo_push(&fifo, test_string_6, (uint32_t)strlen(test_string_6));
    fifo_push(&fifo, test_string_7, (uint32_t)strlen(test_string_7));
    fifo_push(&fifo, test_string_8, (uint32_t)strlen(test_string_8));
    fifo_push(&fifo, test_string_9, (uint32_t)strlen(test_string_9));
    fifo_push(&fifo, test_string_10, (uint32_t)strlen(test_string_10));
    fifo_push(&fifo, test_string_11, (uint32_t)strlen(test_string_11));
    fifo_push(&fifo, test_string_12, (uint32_t)strlen(test_string_12));
    fifo_push(&fifo, test_string_13, (uint32_t)strlen(test_string_13));
    fifo_push(&fifo, test_string_14, (uint32_t)strlen(test_string_14));
    fifo_push(&fifo, test_string_15, (uint32_t)strlen(test_string_15));
    
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(16, fifo_count(&fifo));

    uint8_t data0[128] = {0};
    uint32_t len0 = fifo_pull(&fifo, data0);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(15, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_0, data0);
    TEST_ASSERT_EQUAL(strlen(test_string_0), len0);

    uint8_t data1[128] = {0};
    uint32_t len1 = fifo_pull(&fifo, data1);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(14, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_1, data1);
    TEST_ASSERT_EQUAL(strlen(test_string_1), len1);

    uint8_t data2[128] = {0};
    uint32_t len2 = fifo_pull(&fifo, data2);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(13, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_2, data2);
    TEST_ASSERT_EQUAL(strlen(test_string_2), len2);

    uint8_t data3[128] = {0};
    uint32_t len3 = fifo_pull(&fifo, data3);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(12, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_3, data3);
    TEST_ASSERT_EQUAL(strlen(test_string_3), len3);

    uint8_t data4[128] = {0};
    uint32_t len4 = fifo_pull(&fifo, data4);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(11, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_4, data4);
    TEST_ASSERT_EQUAL(strlen(test_string_4), len4);

    uint8_t data5[128] = {0};
    uint32_t len5 = fifo_pull(&fifo, data5);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(10, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_5, data5);
    TEST_ASSERT_EQUAL(strlen(test_string_5), len5);

    uint8_t data6[128] = {0};
    uint32_t len6 = fifo_pull(&fifo, data6);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(9, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_6, data6);
    TEST_ASSERT_EQUAL(strlen(test_string_6), len6);

    uint8_t data7[128] = {0};
    uint32_t len7 = fifo_pull(&fifo, data7);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(8, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_7, data7);
    TEST_ASSERT_EQUAL(strlen(test_string_7), len7);

    uint8_t data8[128] = {0};
    uint32_t len8 = fifo_pull(&fifo, data8);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(7, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_8, data8);
    TEST_ASSERT_EQUAL(strlen(test_string_8), len8);

    uint8_t data9[128] = {0};
    uint32_t len9 = fifo_pull(&fifo, data9);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(6, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_9, data9);
    TEST_ASSERT_EQUAL(strlen(test_string_9), len9);

    uint8_t data10[128] = {0};
    uint32_t len10 = fifo_pull(&fifo, data10);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(5, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_10, data10);
    TEST_ASSERT_EQUAL(strlen(test_string_10), len10);

    uint8_t data11[128] = {0};
    uint32_t len11 = fifo_pull(&fifo, data11);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(4, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_11, data11);
    TEST_ASSERT_EQUAL(strlen(test_string_11), len11);

    uint8_t data12[128] = {0};
    uint32_t len12 = fifo_pull(&fifo, data12);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(3, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_12, data12);
    TEST_ASSERT_EQUAL(strlen(test_string_12), len12);

    uint8_t data13[128] = {0};
    uint32_t len13 = fifo_pull(&fifo, data13);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(2, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_13, data13);
    TEST_ASSERT_EQUAL(strlen(test_string_13), len13);

    uint8_t data14[128] = {0};
    uint32_t len14 = fifo_pull(&fifo, data14);
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(1, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_14, data14);
    TEST_ASSERT_EQUAL(strlen(test_string_14), len14);

    uint8_t data15[128] = {0};
    uint32_t len15 = fifo_pull(&fifo, data15);
    TEST_ASSERT(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));
    TEST_ASSERT_EQUAL(0, fifo_count(&fifo));
    TEST_ASSERT_EQUAL_STRING(test_string_15, data15);
    TEST_ASSERT_EQUAL(strlen(test_string_15), len15);
}

void test_mixed_push_pull_usage(void)
{
    uint32_t len;
    uint8_t pdata[128] = {0};
    fifo_t fifo;
    fifo_init(&fifo);

    TEST_ASSERT(fifo_is_empty(&fifo));

    // 5 3 0 13 1 15 11 12 4 2 10 14 9 6 8 7
    fifo_push(&fifo, test_string_5, (uint32_t)strlen(test_string_5));
    fifo_push(&fifo, test_string_3, (uint32_t)strlen(test_string_3));
    fifo_push(&fifo, test_string_0, (uint32_t)strlen(test_string_0));
    fifo_push(&fifo, test_string_13, (uint32_t)strlen(test_string_13));
    fifo_push(&fifo, test_string_1, (uint32_t)strlen(test_string_1));
    fifo_push(&fifo, test_string_15, (uint32_t)strlen(test_string_15));
    fifo_push(&fifo, test_string_11, (uint32_t)strlen(test_string_11));
    fifo_push(&fifo, test_string_12, (uint32_t)strlen(test_string_12));
    fifo_push(&fifo, test_string_4, (uint32_t)strlen(test_string_4));
    fifo_push(&fifo, test_string_2, (uint32_t)strlen(test_string_2));
    fifo_push(&fifo, test_string_10, (uint32_t)strlen(test_string_10));
    fifo_push(&fifo, test_string_14, (uint32_t)strlen(test_string_14));
    TEST_ASSERT_EQUAL(12, fifo_count(&fifo));

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_5, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_3, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_0, pdata);
    memset(pdata, 0, 128);

    TEST_ASSERT_EQUAL(9, fifo_count(&fifo));

    fifo_push(&fifo, test_string_9, (uint32_t)strlen(test_string_9));
    fifo_push(&fifo, test_string_6, (uint32_t)strlen(test_string_6));
    fifo_push(&fifo, test_string_8, (uint32_t)strlen(test_string_8));
    fifo_push(&fifo, test_string_7, (uint32_t)strlen(test_string_7));
    fifo_push(&fifo, test_string_1, (uint32_t)strlen(test_string_1));
    TEST_ASSERT_EQUAL(14, fifo_count(&fifo));
    TEST_ASSERT_FALSE(fifo_is_empty(&fifo));
    TEST_ASSERT_FALSE(fifo_is_full(&fifo));

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_13, pdata);
    memset(pdata, 0, 128);

    fifo_push(&fifo, test_string_12, (uint32_t)strlen(test_string_12));
    fifo_push(&fifo, test_string_2, (uint32_t)strlen(test_string_2));
    fifo_push(&fifo, test_string_14, (uint32_t)strlen(test_string_14));
    TEST_ASSERT_EQUAL(16, fifo_count(&fifo));
    TEST_ASSERT(fifo_is_full(&fifo));

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_1, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_15, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_11, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_12, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_4, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_2, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_10, pdata);
    memset(pdata, 0, 128);

    len = fifo_pull(&fifo, pdata);
    TEST_ASSERT_EQUAL_STRING(test_string_14, pdata);
    memset(pdata, 0, 128);

    TEST_ASSERT_EQUAL(8, fifo_count(&fifo));
}

#endif // TEST
