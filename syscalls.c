/**
 ******************************************************************************
 * @file           : syscalls.c
 * @brief          : System calls implementation for STM32 with FreeRTOS
 * @details        : Implements newlib syscalls for printf redirection via UART
 *                   and memory management with FreeRTOS heap
 ******************************************************************************
 * @attention
 *
 * This file provides:
 * - _write() for printf redirection to UART
 * - _read() for scanf redirection from UART
 * - Memory allocation stubs (malloc/free use FreeRTOS heap)
 * - File system stubs (minimal implementation)
 *
 * Compatible with:
 * - STM32CubeIDE
 * - ARM GCC toolchain with newlib
 * - FreeRTOS RTOS
 *
 ******************************************************************************
 */

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include "usart.h"

/* Variables */
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

#undef errno
extern int errno;

/*
 * Pointer to the current high watermark of the heap usage
 */
register char *stack_ptr asm("sp");

char *__env[1] = { 0 };
char **environ = __env;

/**
 * @brief  Initialize the heap memory
 * @param  None
 * @retval None
 */
void initialise_monitor_handles()
{
}

/**
 * @brief  Get the process ID
 * @retval Process ID (always 1)
 */
int _getpid(void)
{
    return 1;
}

/**
 * @brief  Kill a process
 * @param  pid: Process ID
 * @param  sig: Signal number
 * @retval -1 with errno set to EINVAL
 */
int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

/**
 * @brief  Exit the program
 * @param  status: Exit status code
 * @retval None (infinite loop)
 */
void _exit(int status)
{
    _kill(status, -1);
    while (1) {} /* Hang here forever */
}

/**
 * @brief  Write to a file descriptor
 * @param  file: File descriptor (1=stdout, 2=stderr)
 * @param  ptr: Pointer to data buffer
 * @param  len: Number of bytes to write
 * @retval Number of bytes written, or -1 on error
 *
 * @details Redirects stdout/stderr to UART for printf support
 */
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    int DataIdx;

    /* Write to UART if file is stdout or stderr */
    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        for (DataIdx = 0; DataIdx < len; DataIdx++)
        {
            __io_putchar(*ptr++);
        }
        return len;
    }

    errno = EBADF;
    return -1;
}

/**
 * @brief  Close a file descriptor
 * @param  file: File descriptor
 * @retval -1 with errno set to EBADF
 */
int _close(int file)
{
    errno = EBADF;
    return -1;
}

/**
 * @brief  Set position in a file
 * @param  file: File descriptor
 * @param  ptr: Offset
 * @param  dir: Direction (SEEK_SET, SEEK_CUR, SEEK_END)
 * @retval 0 (always successful in stub)
 */
int _lseek(int file, int ptr, int dir)
{
    return 0;
}

/**
 * @brief  Read from a file descriptor
 * @param  file: File descriptor (0=stdin)
 * @param  ptr: Pointer to buffer
 * @param  len: Number of bytes to read
 * @retval Number of bytes read, or -1 on error
 *
 * @details Redirects stdin from UART for scanf support
 */
__attribute__((weak)) int _read(int file, char *ptr, int len)
{
    int DataIdx;

    /* Read from UART if file is stdin */
    if (file == STDIN_FILENO)
    {
        for (DataIdx = 0; DataIdx < len; DataIdx++)
        {
            *ptr++ = __io_getchar();
        }
        return len;
    }

    errno = EBADF;
    return -1;
}

/**
 * @brief  Get file status
 * @param  file: File descriptor
 * @param  st: Pointer to stat structure
 * @retval 0 if character device, -1 otherwise
 */
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

/**
 * @brief  Check if file descriptor is a terminal
 * @param  file: File descriptor
 * @retval 1 (always true in stub)
 */
int _isatty(int file)
{
    return 1;
}

/**
 * @brief  Increase program data space (heap allocation)
 * @param  incr: Number of bytes to increase
 * @retval Pointer to allocated memory, or (caddr_t)-1 on error
 *
 * @details This is called by malloc/free. When using FreeRTOS, prefer
 *          pvPortMalloc/vPortFree instead of malloc/free
 */
caddr_t _sbrk(int incr)
{
    extern char end asm("end"); /* Defined by the linker */
    static char *heap_end = 0;
    char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = &end;
    }

    prev_heap_end = heap_end;

    /* Check if there's enough memory */
    if (heap_end + incr > stack_ptr)
    {
        errno = ENOMEM;
        return (caddr_t) -1;
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}

/**
 * @brief  Get process times
 * @param  buf: Pointer to tms structure
 * @retval -1 (not implemented)
 */
int _times(struct tms *buf)
{
    return -1;
}

/**
 * @brief  Remove a file
 * @param  name: File name
 * @retval -1 with errno set to ENOENT
 */
int _unlink(char *name)
{
    errno = ENOENT;
    return -1;
}

/**
 * @brief  Wait for child process
 * @param  status: Pointer to status variable
 * @retval -1 with errno set to ECHILD
 */
int _wait(int *status)
{
    errno = ECHILD;
    return -1;
}

/**
 * @brief  Create a link
 * @param  old: Old path
 * @param  new: New path
 * @retval -1 with errno set to EMLINK
 */
int _link(char *old, char *new)
{
    errno = EMLINK;
    return -1;
}

/**
 * @brief  Fork process
 * @retval -1 with errno set to EAGAIN
 */
int _fork(void)
{
    errno = EAGAIN;
    return -1;
}

/**
 * @brief  Execute program
 * @param  name: Program name
 * @param  argv: Argument vector
 * @param  env: Environment variables
 * @retval -1 with errno set to ENOMEM
 */
int _execve(char *name, char **argv, char **env)
{
    errno = ENOMEM;
    return -1;
}
