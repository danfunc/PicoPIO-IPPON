#include "build_info.h"
#include "build_info_gen.h"
#include <stdio.h>

const char *build_info_get_time_utc(void) {
    return BUILD_INFO_TIME_UTC;
}

const char *build_info_get_git_commit(void) {
    return BUILD_INFO_GIT_COMMIT;
}

const char *build_info_get_source_sha256(void) {
    return BUILD_INFO_SOURCE_SHA256;
}

void build_info_print(const char *target_name) {
    (void)target_name;
    printf("[BUILD] %s | git: %s | src-sha256: %s\n",
           BUILD_INFO_TIME_UTC,
           BUILD_INFO_GIT_COMMIT,
           BUILD_INFO_SOURCE_SHA256);
}
