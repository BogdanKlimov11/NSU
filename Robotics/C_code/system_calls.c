/**
  ******************************************************************************
  * @file    system_calls.c
  * @brief   Minimal system calls for STM32CubeIDE.
  * @author  STMicroelectronics
  * @date    2025
  *
  * @attention
  * Copyright (c) 2025 STMicroelectronics. All rights reserved.
  * Licensed under BSD 3-Clause license. See opensource.org/licenses/BSD-3-Clause.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>

/* Variables -----------------------------------------------------------------*/
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

char *s_env[1] = {0};
char **environ = s_env;

/* Functions -----------------------------------------------------------------*/
void initialise_monitor_handles(void)
{
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    _kill(status, -1);
    while (1) {}
}

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
    int idx;
    for (idx = 0; idx < len; idx++) {
        *ptr++ = __io_getchar();
    }
    return len;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    int idx;
    for (idx = 0; idx < len; idx++) {
        __io_putchar(*ptr++);
    }
    return len;
}

int _close(int file)
{
    return -1;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _open(char *path, int flags, ...)
{
    return -1;
}

int _wait(int *status)
{
    errno = ECHILD;
    return -1;
}

int _unlink(char *name)
{
    errno = ENOENT;
    return -1;
}

int _times(struct tms *buf)
{
    return -1;
}

int _stat(char *file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _link(char *old, char *new)
{
    errno = EMLINK;
    return -1;
}

int _fork(void)
{
    errno = EAGAIN;
    return -1;
}

int _execve(char *name, char **argv, char **env)
{
    errno = ENOMEM;
    return -1;
}
