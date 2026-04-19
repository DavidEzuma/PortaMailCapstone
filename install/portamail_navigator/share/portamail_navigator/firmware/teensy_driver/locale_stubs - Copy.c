#include <sys/reent.h>

// Stub implementations for missing locale functions
const char * __locale_charset(void) {
    return "UTF-8";
}

struct __locale_t;
const struct __locale_t * __get_current_locale(void) {
    return 0;
}

// Provide the missing __locale_ctype_ptr function
char **__locale_ctype_ptr(void) {
    static char *ctype_ptr = 0;
    return &ctype_ptr;
}