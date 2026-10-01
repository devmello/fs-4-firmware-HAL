// Minimal newlib-nano system calls. _write lives in console.c.

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

#undef errno
extern int errno;

// From the linker script
extern uint8_t end;
extern uint8_t _estack;
extern uint8_t _Min_Stack_Size;

// Heap grows up from the end of .bss and stops _Min_Stack_Size below the top of RAM
void *_sbrk(ptrdiff_t incr) {
    static uint8_t *heap_end = NULL;
    const uint8_t *limit = &_estack - (uint32_t)&_Min_Stack_Size;

    if (heap_end == NULL) {
        heap_end = &end;
    }
    if (heap_end + incr > limit) {
        errno = ENOMEM;
        return (void *)-1;
    }

    uint8_t *prev = heap_end;
    heap_end += incr;
    return prev;
}

int _close(int file) {
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st) {
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir) {
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _getpid(void) {
    return 1;
}

int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status) {
    (void)status;
    while (1) {
    }
}
