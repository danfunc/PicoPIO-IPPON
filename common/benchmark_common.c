#include "benchmark_common.h"
#include <string.h>

// プロジェクト唯一の test_lens 定義実体 (M-5)
const uint8_t g_poc_test_lens[POC_NUM_TEST_LENS] = {1, 8, 32, 64, 128};
const size_t g_poc_num_test_lens = POC_NUM_TEST_LENS;

void poc_stats_reset(poc_stats_t *stats) {
    if (stats) {
        memset(stats, 0, sizeof(*stats));
    }
}
