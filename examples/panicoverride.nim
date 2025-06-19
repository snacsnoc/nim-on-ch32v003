proc nimPanic(msg: string) {.noconv, noreturn.} =
  while true:
    discard

proc panic*(s: string) {.noconv, noreturn, noinline.} =
  # this is to stub out an internal 'panic' that fatal.nim might be trying to call
  # behavior should be same as nimPanic
  while true:
    discard

proc rawoutput*(s: string) {.noconv, noinline.} =
  discard

# Provide implementations for missing C library functions
{.emit: """
#include <stddef.h>

void *memset(void *s, int c, size_t n) {
    unsigned char *p = s;
    while (n-- > 0) {
        *p++ = (unsigned char)c;
    }
    return s;
}

void *memcpy(void *dest, const void *src, size_t n) {
    char *dp = dest;
    const char *sp = src;
    while (n--)
        *dp++ = *sp++;
    return dest;
}

void *memmove(void *dest, const void *src, size_t n)
{
    unsigned char *d = dest;
    const unsigned char *s = src;
    if (d < s) {
        while (n--)
            *d++ = *s++;
    } else {
        const unsigned char *lasts = s + (n-1);
        unsigned char *lastd = d + (n-1);
        while (n--)
            *lastd-- = *lasts--;
    }
    return dest;
}
""".}
