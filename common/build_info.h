#ifndef BUILD_INFO_H
#define BUILD_INFO_H

#ifdef __cplusplus
extern "C" {
#endif

const char *build_info_get_time_utc(void);
const char *build_info_get_git_commit(void);
const char *build_info_get_source_sha256(void);
void build_info_print(const char *target_name);

#ifdef __cplusplus
}
#endif

#endif // BUILD_INFO_H
