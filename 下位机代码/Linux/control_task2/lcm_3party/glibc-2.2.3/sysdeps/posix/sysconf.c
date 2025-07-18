/* Copyright (C) 1991,93,95,96,97,99,2000,2001 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public License as
   published by the Free Software Foundation; either version 2 of the
   License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with the GNU C Library; see the file COPYING.LIB.  If not,
   write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */

#include <errno.h>
#include <limits.h>
#include <grp.h>
#include <pwd.h>
#include <stddef.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <regex.h>


/* Get the value of the system variable NAME.  */
long int
__sysconf (name)
     int name;
{
  switch (name)
    {
    default:
      __set_errno (EINVAL);
      return -1;

    case _SC_ARG_MAX:
#ifdef	ARG_MAX
      return ARG_MAX;
#else
      return -1;
#endif

    case _SC_CHILD_MAX:
#ifdef	CHILD_MAX
      return CHILD_MAX;
#else
      return -1;
#endif

    case _SC_CLK_TCK:
      return __getclktck ();

    case _SC_NGROUPS_MAX:
#ifdef	NGROUPS_MAX
      return NGROUPS_MAX;
#else
      return -1;
#endif

    case _SC_OPEN_MAX:
      return __getdtablesize ();

    case _SC_STREAM_MAX:
#ifdef	STREAM_MAX
      return STREAM_MAX;
#else
      return FOPEN_MAX;
#endif

    case _SC_TZNAME_MAX:
      return __tzname_max ();

    case _SC_JOB_CONTROL:
#ifdef	_POSIX_JOB_CONTROL
      return 1;
#else
      return -1;
#endif

    case _SC_SAVED_IDS:
#ifdef	_POSIX_SAVED_IDS
      return 1;
#else
      return -1;
#endif

    case _SC_REALTIME_SIGNALS:
#ifdef	_POSIX_REALTIME_SIGNALS
      return 1;
#else
      return -1;
#endif

    case _SC_PRIORITY_SCHEDULING:
#ifdef	_POSIX_PRIORITY_SCHEDULING
      return 1;
#else
      return -1;
#endif

    case _SC_TIMERS:
#ifdef	_POSIX_TIMERS
      return 1;
#else
      return -1;
#endif

    case _SC_ASYNCHRONOUS_IO:
#ifdef	_POSIX_ASYNCHRONOUS_IO
      return 1;
#else
      return -1;
#endif

    case _SC_PRIORITIZED_IO:
#ifdef	_POSIX_PRIORITIZED_IO
      return 1;
#else
      return -1;
#endif

    case _SC_SYNCHRONIZED_IO:
#ifdef	_POSIX_SYNCHRONIZED_IO
      return 1;
#else
      return -1;
#endif

    case _SC_FSYNC:
#ifdef	_POSIX_FSYNC
      return 1;
#else
      return -1;
#endif

    case _SC_MAPPED_FILES:
#ifdef	_POSIX_MAPPED_FILES
      return 1;
#else
      return -1;
#endif

    case _SC_MEMLOCK:
#ifdef	_POSIX_MEMLOCK
      return 1;
#else
      return -1;
#endif

    case _SC_MEMLOCK_RANGE:
#ifdef	_POSIX_MEMLOCK_RANGE
      return 1;
#else
      return -1;
#endif

    case _SC_MEMORY_PROTECTION:
#ifdef	_POSIX_MEMORY_PROTECTION
      return 1;
#else
      return -1;
#endif

    case _SC_MESSAGE_PASSING:
#ifdef	_POSIX_MESSAGE_PASSING
      return 1;
#else
      return -1;
#endif

    case _SC_SEMAPHORES:
#ifdef	_POSIX_SEMAPHORES
      return 1;
#else
      return -1;
#endif

    case _SC_SHARED_MEMORY_OBJECTS:
#ifdef	_POSIX_SHARED_MEMORY_OBJECTS
      return 1;
#else
      return -1;
#endif

    case _SC_VERSION:
      return _POSIX_VERSION;

    case _SC_PAGESIZE:
      return __getpagesize ();

    case _SC_AIO_LISTIO_MAX:
#ifdef	AIO_LISTIO_MAX
      return AIO_LISTIO_MAX;
#else
      return -1;
#endif

    case _SC_AIO_MAX:
#ifdef	AIO_MAX
      return AIO_MAX;
#else
      return -1;
#endif

    case _SC_AIO_PRIO_DELTA_MAX:
#ifdef	AIO_PRIO_DELTA_MAX
      return AIO_PRIO_DELTA_MAX;
#else
      return -1;
#endif

    case _SC_DELAYTIMER_MAX:
#ifdef	DELAYTIMER_MAX
      return DELAYTIMER_MAX;
#else
      return -1;
#endif

    case _SC_MQ_OPEN_MAX:
#ifdef	MQ_OPEN_MAX
      return MQ_OPEN_MAX;
#else
      return -1;
#endif

    case _SC_MQ_PRIO_MAX:
#ifdef	MQ_PRIO_MAX
      return MQ_PRIO_MAX;
#else
      return -1;
#endif

    case _SC_RTSIG_MAX:
#ifdef	RTSIG_MAX
      return RTSIG_MAX;
#else
      return -1;
#endif

    case _SC_SEM_NSEMS_MAX:
#ifdef	SEM_NSEMS_MAX
      return SEM_NSEMS_MAX;
#else
      return -1;
#endif

    case _SC_SEM_VALUE_MAX:
#ifdef	SEM_VALUE_MAX
      return SEM_VALUE_MAX;
#else
      return -1;
#endif

    case _SC_SIGQUEUE_MAX:
#ifdef	SIGQUEUE_MAX
      return SIGQUEUE_MAX;
#else
      return -1;
#endif

    case _SC_TIMER_MAX:
#ifdef	TIMER_MAX
      return TIMER_MAX;
#else
      return -1;
#endif

    case _SC_BC_BASE_MAX:
#ifdef	BC_BASE_MAX
      return BC_BASE_MAX;
#else
      return -1;
#endif

    case _SC_BC_DIM_MAX:
#ifdef	BC_DIM_MAX
      return BC_DIM_MAX;
#else
      return -1;
#endif

    case _SC_BC_SCALE_MAX:
#ifdef	BC_SCALE_MAX
      return BC_SCALE_MAX;
#else
      return -1;
#endif

    case _SC_BC_STRING_MAX:
#ifdef	BC_STRING_MAX
      return BC_STRING_MAX;
#else
      return -1;
#endif

    case _SC_COLL_WEIGHTS_MAX:
#ifdef	COLL_WEIGHTS_MAX
      return COLL_WEIGHTS_MAX;
#else
      return -1;
#endif

    case _SC_EXPR_NEST_MAX:
#ifdef	EXPR_NEST_MAX
      return EXPR_NEST_MAX;
#else
      return -1;
#endif

    case _SC_LINE_MAX:
#ifdef	LINE_MAX
      return LINE_MAX;
#else
      return -1;
#endif

    case _SC_RE_DUP_MAX:
#ifdef	RE_DUP_MAX
      return RE_DUP_MAX;
#else
      return -1;
#endif

    case _SC_CHARCLASS_NAME_MAX:
#ifdef	CHARCLASS_NAME_MAX
      return CHARCLASS_NAME_MAX;
#else
      return -1;
#endif

    case _SC_PII:
#ifdef	_POSIX_PII
      return 1;
#else
      return -1;
#endif

    case _SC_PII_XTI:
#ifdef	_POSIX_PII_XTI
      return 1;
#else
      return -1;
#endif

    case _SC_PII_SOCKET:
#ifdef	_POSIX_PII_SOCKET
      return 1;
#else
      return -1;
#endif

    case _SC_PII_INTERNET:
#ifdef	_POSIX_PII_INTERNET
      return 1;
#else
      return -1;
#endif

    case _SC_PII_OSI:
#ifdef	_POSIX_PII_OSI
      return 1;
#else
      return -1;
#endif

    case _SC_POLL:
#ifdef	_POSIX_POLL
      return 1;
#else
      return -1;
#endif

    case _SC_SELECT:
#ifdef	_POSIX_SELECT
      return 1;
#else
      return -1;
#endif

    case _SC_UIO_MAXIOV:
#ifdef	UIO_MAXIOV
      return UIO_MAXIOV;
#else
      return -1;
#endif

    case _SC_PII_INTERNET_STREAM:
#ifdef	_POSIX_PII_INTERNET_STREAM
      return 1;
#else
      return -1;
#endif

    case _SC_PII_INTERNET_DGRAM:
#ifdef	_POSIX_PII_INTERNET_DGRAM
      return 1;
#else
      return -1;
#endif

    case _SC_PII_OSI_COTS:
#ifdef	_POSIX_PII_OSI_COTS
      return 1;
#else
      return -1;
#endif

    case _SC_PII_OSI_CLTS:
#ifdef	_POSIX_PII_OSI_CLTS
      return 1;
#else
      return -1;
#endif

    case _SC_PII_OSI_M:
#ifdef	_POSIX_PII_OSI_M
      return 1;
#else
      return -1;
#endif

    case _SC_T_IOV_MAX:
#ifdef	_T_IOV_MAX
      return _T_IOV_MAX;
#else
      return -1;
#endif

    case _SC_2_VERSION:
      return _POSIX2_VERSION;

    case _SC_2_C_BIND:
#ifdef	_POSIX2_C_BIND
      return _POSIX2_C_BIND;
#else
      return -1;
#endif

    case _SC_2_C_DEV:
#ifdef	_POSIX2_C_DEV
      return _POSIX2_C_DEV;
#else
      return -1;
#endif

    case _SC_2_C_VERSION:
#ifdef	_POSIX2_C_VERSION
      return _POSIX2_C_VERSION;
#else
      return -1;
#endif

    case _SC_2_FORT_DEV:
#ifdef	_POSIX2_FORT_DEV
      return _POSIX2_FORT_DEV;
#else
      return -1;
#endif

    case _SC_2_LOCALEDEF:
#ifdef	_POSIX2_LOCALEDEF
      return _POSIX2_LOCALEDEF;
#else
      return -1;
#endif

    case _SC_2_SW_DEV:
#ifdef	_POSIX2_SW_DEV
      return _POSIX2_SW_DEV;
#else
      return -1;
#endif

    case _SC_2_CHAR_TERM:
#ifdef	_POSIX2_CHAR_TERM
      return _POSIX2_CHAR_TERM;
#else
      return -1;
#endif

    case _SC_2_UPE:
#ifdef	_POSIX2_UPE
      return _POSIX2_UPE;
#else
      return -1;
#endif

      /* POSIX 1003.1c (POSIX Threads).  */
    case _SC_THREADS:
#ifdef	_POSIX_THREADS
      return 1;
#else
      return -1;
#endif

    case _SC_THREAD_SAFE_FUNCTIONS:
#ifdef	_POSIX_THREAD_SAFE_FUNCTIONS
      return 1;
#else
      return -1;
#endif

    case _SC_GETGR_R_SIZE_MAX:
#ifdef	NSS_BUFLEN_GROUP
      return NSS_BUFLEN_GROUP;
#else
      return -1;
#endif

    case _SC_GETPW_R_SIZE_MAX:
#ifdef	NSS_BUFLEN_PASSWD
      return NSS_BUFLEN_PASSWD;
#else
      return -1;
#endif

    case _SC_LOGIN_NAME_MAX:
#ifdef	_POSIX_LOGIN_NAME_MAX
      return _POSIX_LOGIN_NAME_MAX;
#else
      return -1;
#endif

    case _SC_TTY_NAME_MAX:
#ifdef	_POSIX_TTY_NAME_MAX
      return _POSIX_TTY_NAME_MAX;
#else
      return -1;
#endif

    case _SC_THREAD_DESTRUCTOR_ITERATIONS:
#ifdef	_POSIX_THREAD_DESTRUCTOR_ITERATIONS
      return _POSIX_THREAD_DESTRUCTOR_ITERATIONS;
#else
      return -1;
#endif

    case _SC_THREAD_KEYS_MAX:
#ifdef	PTHREAD_KEYS_MAX
      return PTHREAD_KEYS_MAX;
#else
      return -1;
#endif

    case _SC_THREAD_STACK_MIN:
#ifdef	PTHREAD_STACK_MIN
      return PTHREAD_STACK_MIN;
#else
      return -1;
#endif

    case _SC_THREAD_THREADS_MAX:
#ifdef	PTHREAD_THREADS_MAX
      return PTHREAD_THREADS_MAX;
#else
      return -1;
#endif

    case _SC_THREAD_ATTR_STACKADDR:
#ifdef	_POSIX_THREAD_ATTR_STACKADDR
      return 1;
#else
      return -1;
#endif

    case _SC_THREAD_ATTR_STACKSIZE:
#ifdef	_POSIX_THREAD_ATTR_STACKSIZE
      return 1;
#else
      return -1;
#endif

    case _SC_THREAD_PRIORITY_SCHEDULING:
#ifdef	_POSIX_THREAD_PRIORITY_SCHEDULING
      return 1;
#else
      return -1;
#endif

    case _SC_THREAD_PRIO_INHERIT:
#ifdef	_POSIX_THREAD_PRIO_INHERIT
      return 1;
#else
      return -1;
#endif

    case _SC_THREAD_PRIO_PROTECT:
#ifdef	_POSIX_THREAD_PRIO_PROTECT
      return 1;
#else
      return -1;
#endif

    case _SC_THREAD_PROCESS_SHARED:
#ifdef	_POSIX_THREAD_PROCESS_SHARED
      return 1;
#else
      return -1;
#endif

    case _SC_NPROCESSORS_CONF:
      return __get_nprocs_conf ();

    case _SC_NPROCESSORS_ONLN:
      return __get_nprocs ();

    case _SC_PHYS_PAGES:
      return __get_phys_pages ();

    case _SC_AVPHYS_PAGES:
      return __get_avphys_pages ();

    case _SC_ATEXIT_MAX:
      /* We have no limit since we use lists.  */
      return INT_MAX;

    case _SC_PASS_MAX:
      /* We have no limit but since the return value might be used to
	 allocate a buffer we restrict the value.  */
      return BUFSIZ;

    case _SC_XOPEN_VERSION:
      return _XOPEN_VERSION;

    case _SC_XOPEN_XCU_VERSION:
      return _XOPEN_XCU_VERSION;

    case _SC_XOPEN_UNIX:
      return _XOPEN_UNIX;

    case _SC_XOPEN_CRYPT:
#ifdef	_XOPEN_CRYPT
      return _XOPEN_CRYPT;
#else
      return -1;
#endif

    case _SC_XOPEN_ENH_I18N:
#ifdef	_XOPEN_ENH_I18N
      return _XOPEN_ENH_I18N;
#else
      return -1;
#endif

    case _SC_XOPEN_SHM:
#ifdef	_XOPEN_SHM
      return _XOPEN_SHM;
#else
      return -1;
#endif

    case _SC_XOPEN_XPG2:
#ifdef	_XOPEN_XPG2
      return _XOPEN_XPG2;
#else
      return -1;
#endif

    case _SC_XOPEN_XPG3:
#ifdef	_XOPEN_XPG3
      return _XOPEN_XPG3;
#else
      return -1;
#endif

    case _SC_XOPEN_XPG4:
#ifdef	_XOPEN_XPG4
      return _XOPEN_XPG4;
#else
      return -1;
#endif

    case _SC_CHAR_BIT:
      return CHAR_BIT;

    case _SC_CHAR_MAX:
      return CHAR_MAX;

    case _SC_CHAR_MIN:
      return CHAR_MIN;

    case _SC_INT_MAX:
      return INT_MAX;

    case _SC_INT_MIN:
      return INT_MIN;

    case _SC_LONG_BIT:
      return sizeof (long int) * CHAR_BIT;

    case _SC_WORD_BIT:
      return sizeof (int) * CHAR_BIT;

    case _SC_MB_LEN_MAX:
      return MB_LEN_MAX;

    case _SC_NZERO:
      return NZERO;

    case _SC_SSIZE_MAX:
      return _POSIX_SSIZE_MAX;

    case _SC_SCHAR_MAX:
      return SCHAR_MAX;

    case _SC_SCHAR_MIN:
      return SCHAR_MIN;

    case _SC_SHRT_MAX:
      return SHRT_MAX;

    case _SC_SHRT_MIN:
      return SHRT_MIN;

    case _SC_UCHAR_MAX:
      return UCHAR_MAX;

    case _SC_UINT_MAX:
      return UINT_MAX;

    case _SC_ULONG_MAX:
      return ULONG_MAX;

    case _SC_USHRT_MAX:
      return USHRT_MAX;

    case _SC_NL_ARGMAX:
#ifdef	NL_ARGMAX
      return NL_ARGMAX;
#else
      return -1;
#endif

    case _SC_NL_LANGMAX:
#ifdef	NL_LANGMAX
      return NL_LANGMAX;
#else
      return -1;
#endif

    case _SC_NL_MSGMAX:
#ifdef	NL_MSGMAX
      return NL_MSGMAX;
#else
      return -1;
#endif

    case _SC_NL_NMAX:
#ifdef	NL_NMAX
      return NL_NMAX;
#else
      return -1;
#endif

    case _SC_NL_SETMAX:
#ifdef	NL_SETMAX
      return NL_SETMAX;
#else
      return -1;
#endif

    case _SC_NL_TEXTMAX:
#ifdef	NL_TEXTMAX
      return NL_TEXTMAX;
#else
      return -1;
#endif

    case _SC_XBS5_ILP32_OFF32:
#ifdef _XBS5_ILP32_OFF32
      return _XBS5_ILP32_OFF32;
#else
      return -1;
#endif
    case _SC_XBS5_ILP32_OFFBIG:
#ifdef _XBS5_ILP32_OFFBIG
      return _XBS5_ILP32_OFFBIG;
#else
      return -1;
#endif
    case _SC_XBS5_LP64_OFF64:
#ifdef _XBS5_LP64_OFF64
      return _XBS5_LP64_OFF64;
#else
      return -1;
#endif
    case _SC_XBS5_LPBIG_OFFBIG:
#ifdef _XBS5_LPBIG_OFFBIG
      return _XBS5_LPBIG_OFFBIG;
#else
      return -1;
#endif

    case _SC_V6_ILP32_OFF32:
#ifdef _POSIX_V6_ILP32_OFF32
      return _POSIX_V6_ILP32_OFF32;
#else
      return -1;
#endif
    case _SC_V6_ILP32_OFFBIG:
#ifdef _POSIX_V6_ILP32_OFFBIG
      return _POSIX_V6_ILP32_OFFBIG;
#else
      return -1;
#endif
    case _SC_V6_LP64_OFF64:
#ifdef _POSIX_V6_LP64_OFF64
      return _POSIX_V6_LP64_OFF64;
#else
      return -1;
#endif
    case _SC_V6_LPBIG_OFFBIG:
#ifdef _POSIX_V6_LPBIG_OFFBIG
      return _POSIX_V6_LPBIG_OFFBIG;
#else
      return -1;
#endif

    case _SC_XOPEN_LEGACY:
      return _XOPEN_LEGACY;

    case _SC_XOPEN_REALTIME:
#ifdef _XOPEN_REALTIME
      return _XOPEN_REALTIME;
#else
      return -1;
#endif
    case _SC_XOPEN_REALTIME_THREADS:
#ifdef _XOPEN_REALTIME_THREADS
      return _XOPEN_REALTIME_THREADS;
#else
      return -1;
#endif

    case _SC_ADVISORY_INFO:
#ifdef _POSIX_ADVISORY_INFO
      return _POSIX_ADVISORY_INFO;
#else
      return -1;
#endif

    case _SC_BARRIERS:
#ifdef _POSIX_BARRIERS
      return _POSIX_BARRIERS;
#else
      return -1;
#endif

    case _SC_BASE:
#ifdef _POSIX_BASE
      return _POSIX_BASE;
#else
      return -1;
#endif
    case _SC_C_LANG_SUPPORT:
#ifdef _POSIX_C_LANG_SUPPORT
      return _POSIX_C_LANG_SUPPORT;
#else
      return -1;
#endif
    case _SC_C_LANG_SUPPORT_R:
#ifdef _POSIX_C_LANG_SUPPORT_R
      return _POSIX_C_LANG_SUPPORT_R;
#else
      return -1;
#endif

    case _SC_CLOCK_SELECTION:
#ifdef _POSIX_CLOCK_SELECTION
      return _POSIX_CLOCK_SELECTION;
#else
      return -1;
#endif

    case _SC_CPUTIME:
#ifdef _POSIX_CPUTIME
      return _POSIX_CPUTIME;
#else
      return -1;
#endif

    case _SC_DEVICE_IO:
#ifdef _POSIX_DEVICE_IO
      return _POSIX_DEVICE_IO;
#else
      return -1;
#endif
    case _SC_DEVICE_SPECIFIC:
#ifdef _POSIX_DEVICE_SPCIFIC
      return _POSIX_DEVICE_SPECIFIC;
#else
      return -1;
#endif
    case _SC_DEVICE_SPECIFIC_R:
#ifdef _POSIX_DEVICE_SPCIFIC_R
      return _POSIX_DEVICE_SPECIFIC_R;
#else
      return -1;
#endif

    case _SC_FD_MGMT:
#ifdef _POSIX_FD_MGMT
      return _POSIX_FD_MGMT;
#else
      return -1;
#endif

    case _SC_FIFO:
#ifdef _POSIX_FIFO
      return _POSIX_FIFO;
#else
      return -1;
#endif
    case _SC_PIPE:
#ifdef _POSIX_PIPE
      return _POSIX_PIPE;
#else
      return -1;
#endif

    case _SC_FILE_ATTRIBUTES:
#ifdef _POSIX_FILE_ATTRIBUTES
      return _POSIX_FILE_ATTRIBUTES;
#else
      return -1;
#endif
    case _SC_FILE_LOCKING:
#ifdef _POSIX_FILE_LOCKING
      return _POSIX_FILE_LOCKING;
#else
      return -1;
#endif
    case _SC_FILE_SYSTEM:
#ifdef _POSIX_FILE_SYSTEM
      return _POSIX_FILE_SYSTEM;
#else
      return -1;
#endif

    case _SC_MONOTONIC_CLOCK:
#ifdef _POSIX_MONOTONIC_CLOCK
      return _POSIX_MONOTONIC_CLOCK;
#else
      return -1;
#endif

    case _SC_MULTI_PROCESS:
#ifdef _POSIX_MULTI_PROCESS
      return _POSIX_MULTI_PROCESS;
#else
      return -1;
#endif
    case _SC_SINGLE_PROCESS:
#ifdef _POSIX_SINGLE_PROCESS
      return _POSIX_SINGLE_PROCESS;
#else
      return -1;
#endif

    case _SC_NETWORKING:
#ifdef _POSIX_NETWORKING
      return _POSIX_NETWORKING;
#else
      return -1;
#endif

    case _SC_READER_WRITER_LOCKS:
#ifdef _POSIX_READER_WRITER_LOCKS
      return _POSIX_READER_WRITER_LOCKS;
#else
      return -1;
#endif
    case _SC_SPIN_LOCKS:
#ifdef _POSIX_SPIN_LOCKS
      return _POSIX_SPIN_LOCKS;
#else
      return -1;
#endif

    case _SC_REGEXP:
#ifdef _POSIX_REGEXP
      return _POSIX_REGEXP;
#else
      return -1;
#endif
    case _SC_REGEX_VERSION:
#ifdef _POSIX_REGEX_VERSION
      return _POSIX_REGEX_VERSION;
#else
      return -1;
#endif

    case _SC_SHELL:
#ifdef _POSIX_SHELL
      return _POSIX_SHELL;
#else
      return -1;
#endif

    case _SC_SIGNALS:
#ifdef _POSUX_SIGNALS
      return _POSIX_SIGNALS;
#else
      return -1;
#endif

    case _SC_SPAWN:
#ifdef _POSIX_SPAWN
      return _POSIX_SPAWN;
#else
      return -1;
#endif

    case _SC_SPORADIC_SERVER:
#ifdef _POSIX_SPORADIC_SERVER
      return _POSIX_SPORADIC_SERVER;
#else
      return -1;
#endif
    case _SC_THREAD_SPORADIC_SERVER:
#ifdef _POSIX_THREAD_SPORADIC_SERVER
      return _POSIX_THREAD_SPORADIC_SERVER;
#else
      return -1;
#endif

    case _SC_SYSTEM_DATABASE:
#ifdef _POSIX_SYSTEM_DATABASE
      return _POSIX_SYSTEM_DATABASE;
#else
      return -1;
#endif
    case _SC_SYSTEM_DATABASE_R:
#ifdef _POSIX_SYSTEM_DATABASE_R
      return _POSIX_SYSTEM_DATABASE_R;
#else
      return -1;
#endif

    case _SC_THREAD_CPUTIME:
#ifdef _POSIX_THREAD_CPUTIME
      return _POSIX_THREAD_CPUTIME;
#else
      return -1;
#endif

    case _SC_TIMEOUTS:
#ifdef _POSIX_TIMEOUTS
      return _POSIX_TIMEOUTS;
#else
      return -1;
#endif

    case _SC_TYPED_MEMORY_OBJECTS:
#ifdef _POSIX_TYPED_MEMORY_OBJECTS
      return _POSIX_TYPED_MEMORY_OBJECTS;
#else
      return -1;
#endif

    case _SC_USER_GROUPS:
#ifdef _POSIX_USER_GROUPS
      return _POSIX_USER_GROUPS;
#else
      return -1;
#endif
    case _SC_USER_GROUPS_R:
#ifdef _POSIX_USER_GROUPS_R
      return _POSIX_USER_GROUPS_R;
#else
      return -1;
#endif

    case _SC_2_PBS:
#ifdef _POSIX2_PBS
      return _POSIX2_PBS;
#else
      return -1;
#endif
    case _SC_2_PBS_ACCOUNTING:
#ifdef _POSIX2_PBS_ACCOUNTING
      return _POSIX2_PBS_ACCOUNTING;
#else
      return -1;
#endif
    case _SC_2_PBS_CHECKPOINT:
#ifdef _POSIX2_PBS_CHECKPOINT
      return _POSIX2_PBS_CHECKPOINT;
#else
      return -1;
#endif
    case _SC_2_PBS_LOCATE:
#ifdef _POSIX2_PBS_LOCATE
      return _POSIX2_PBS_LOCATE;
#else
      return -1;
#endif
    case _SC_2_PBS_TRACK:
#ifdef _POSIX2_PBS_TRACK
      return _POSIX2_PBS_TRACK;
#else
      return -1;
#endif

    case _SC_SYMLOOP_MAX:
#ifdef SYMLOOP_MAX
      return SYMLOOP_MAX;
#else
      return -1;
#endif

    case _SC_STREAMS:
#ifdef _XOPEN_STREAMS
      return _XOPEN_STREAMS;
#else
      return -1;
#endif
    }
}

#undef __sysconf
weak_alias (__sysconf, sysconf)
