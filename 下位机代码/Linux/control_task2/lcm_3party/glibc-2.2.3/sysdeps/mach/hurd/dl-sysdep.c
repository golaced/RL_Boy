/* Operating system support for run-time dynamic linker.  Hurd version.
   Copyright (C) 1995,96,97,98,99,2000,2001 Free Software Foundation, Inc.
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

#include <hurd.h>
#include <link.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <ldsodefs.h>
#include <sys/wait.h>
#include <assert.h>
#include <sysdep.h>
#include <mach/mig_support.h>
#include "hurdstartup.h"
#include <mach/host_info.h>
#include <stdio-common/_itoa.h>
#include <hurd/auth.h>
#include <hurd/term.h>
#include <stdarg.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/uio.h>

#include <entry.h>
#include <dl-machine.h>
#include <dl-procinfo.h>

extern void __mach_init (void);

extern int _dl_argc;
extern char **_dl_argv;
extern char **_environ;

int __libc_enable_secure;
int __libc_multiple_libcs = 0;	/* Defining this here avoids the inclusion
				   of init-first.  */
/* This variable containts the lowest stack address ever used.  */
void *__libc_stack_end;
unsigned long int _dl_hwcap_mask = HWCAP_IMPORTANT;


struct hurd_startup_data *_dl_hurd_data;

/* Defining these variables here avoids the inclusion of hurdsig.c.  */
unsigned long int __hurd_sigthread_stack_base;
unsigned long int __hurd_sigthread_stack_end;
unsigned long int *__hurd_sigthread_variables;

/* Defining these variables here avoids the inclusion of init-first.c.
   We need to provide temporary storage for the per-thread variables
   of the main user thread here, since it is used for storing the
   `errno' variable.  Note that this information is lost once we
   relocate the dynamic linker.  */
static unsigned long int threadvars[_HURD_THREADVAR_MAX];
unsigned long int __hurd_threadvar_stack_offset
  = (unsigned long int) &threadvars;
unsigned long int __hurd_threadvar_stack_mask;

/* XXX loser kludge for vm_map kernel bug */
static vm_address_t fmha;
static vm_size_t fmhs;
static void unfmh(void){
__vm_deallocate(__mach_task_self(),fmha,fmhs);}
static void fmh(void) {
    error_t err;int x;mach_port_t p;
    vm_address_t a=0x08000000U,max=VM_MAX_ADDRESS;
    while (!(err=__vm_region(__mach_task_self(),&a,&fmhs,&x,&x,&x,&x,&p,&x))){
      __mach_port_deallocate(__mach_task_self(),p);
      if (a+fmhs>=0x80000000U){
	max=a; break;}
      fmha=a+=fmhs;}
    if (err) assert(err==KERN_NO_SPACE);
    if (!fmha)fmhs=0;else{
    fmhs=max-fmha;
    err = __vm_map (__mach_task_self (),
		    &fmha, fmhs, 0, 0, MACH_PORT_NULL, 0, 1,
		    VM_PROT_NONE, VM_PROT_NONE, VM_INHERIT_COPY);
    assert_perror(err);}
  }
/* XXX loser kludge for vm_map kernel bug */



Elf32_Addr
_dl_sysdep_start (void **start_argptr,
		  void (*dl_main) (const Elf32_Phdr *phdr, Elf32_Word phent,
				   Elf32_Addr *user_entry))
{
  void go (int *argdata)
    {
      extern unsigned int _dl_skip_args; /* rtld.c */
      char **p;

      /* Cache the information in various global variables.  */
      _dl_argc = *argdata;
      _dl_argv = 1 + (char **) argdata;
      _environ = &_dl_argv[_dl_argc + 1];
      for (p = _environ; *p++;); /* Skip environ pointers and terminator.  */

      if ((void *) p == _dl_argv[0])
	{
	  static struct hurd_startup_data nodata;
	  _dl_hurd_data = &nodata;
	  nodata.user_entry = (vm_address_t) ENTRY_POINT;
	}
      else
	_dl_hurd_data = (void *) p;

      __libc_enable_secure = _dl_hurd_data->flags & EXEC_SECURE;

      if (_dl_hurd_data->flags & EXEC_STACK_ARGS &&
	  _dl_hurd_data->user_entry == 0)
	_dl_hurd_data->user_entry = (vm_address_t) ENTRY_POINT;

unfmh();			/* XXX */

#if 0				/* XXX make this work for real someday... */
      if (_dl_hurd_data->user_entry == (vm_address_t) ENTRY_POINT)
	/* We were invoked as a command, not as the program interpreter.
	   The generic ld.so code supports this: it will parse the args
	   as "ld.so PROGRAM [ARGS...]".  For booting the Hurd, we
	   support an additional special syntax:
	     ld.so [-LIBS...] PROGRAM [ARGS...]
	   Each LIBS word consists of "FILENAME=MEMOBJ";
	   for example "-/lib/libc.so=123" says that the contents of
	   /lib/libc.so are found in a memory object whose port name
	   in our task is 123.  */
	while (_dl_argc > 2 && _dl_argv[1][0] == '-' && _dl_argv[1][1] != '-')
	  {
	    char *lastslash, *memobjname, *p;
	    struct link_map *l;
	    mach_port_t memobj;
	    error_t err;

	    ++_dl_skip_args;
	    --_dl_argc;
	    p = _dl_argv++[1] + 1;

	    memobjname = strchr (p, '=');
	    if (! memobjname)
	      _dl_sysdep_fatal ("Bogus library spec: ", p, "\n", NULL);
	    *memobjname++ = '\0';
	    memobj = 0;
	    while (*memobjname != '\0')
	      memobj = (memobj * 10) + (*memobjname++ - '0');

	    /* Add a user reference on the memory object port, so we will
	       still have one after _dl_map_object_from_fd calls our
	       `close'.  */
	    err = __mach_port_mod_refs (__mach_task_self (), memobj,
					MACH_PORT_RIGHT_SEND, +1);
	    assert_perror (err);

	    lastslash = strrchr (p, '/');
	    l = _dl_map_object_from_fd (lastslash ? lastslash + 1 : p,
					memobj, strdup (p), 0);

	    /* Squirrel away the memory object port where it
	       can be retrieved by the program later.  */
	    l->l_info[DT_NULL] = (void *) memobj;
	  }
#endif

      /* Call elf/rtld.c's main program.  It will set everything
	 up and leave us to transfer control to USER_ENTRY.  */
      (*dl_main) ((const Elf32_Phdr *) _dl_hurd_data->phdr,
		  _dl_hurd_data->phdrsz / sizeof (Elf32_Phdr),
		  &_dl_hurd_data->user_entry);

      /* The call above might screw a few things up.

	 First of all, if _dl_skip_args is nonzero, we are ignoring
	 the first few arguments.  However, if we have no Hurd startup
	 data, it is the magical convention that ARGV[0] == P.  The
	 startup code in init-first.c will get confused if this is not
	 the case, so we must rearrange things to make it so.  We'll
	 overwrite the origional ARGV[0] at P with ARGV[_dl_skip_args].

	 Secondly, if we need to be secure, it removes some dangerous
	 environment variables.  If we have no Hurd startup date this
	 changes P (since that's the location after the terminating
	 NULL in the list of environment variables).  We do the same
	 thing as in the first case but make sure we recalculate P.
	 If we do have Hurd startup data, we have to move the data
	 such that it starts just after the terminating NULL in the
	 environment list.

	 We use memmove, since the locations might overlap.  */
      if (__libc_enable_secure || _dl_skip_args)
	{
	  char **newp;

	  for (newp = _environ; *newp++;);

	  if (_dl_argv[-_dl_skip_args] == (char *) p)
	    {
	      if ((char *) newp != _dl_argv[0])
		{
		  assert ((char *) newp < _dl_argv[0]);
		  _dl_argv[0] = memmove ((char *) newp, _dl_argv[0],
					 strlen (_dl_argv[0]) + 1);
		}
	    }
	  else
	    {
	      if ((void *) newp != _dl_hurd_data)
		memmove (newp, _dl_hurd_data, sizeof (*_dl_hurd_data));
	    }
	}

      {
	extern void _dl_start_user (void);
	/* Unwind the stack to ARGDATA and simulate a return from _dl_start
	   to the RTLD_START code which will run the user's entry point.  */
	RETURN_TO (argdata, &_dl_start_user, _dl_hurd_data->user_entry);
      }
    }

  /* Set up so we can do RPCs.  */
  __mach_init ();

  /* Initialize frequently used global variable.  */
  _dl_pagesize = __getpagesize ();

fmh();				/* XXX */

  /* See hurd/hurdstartup.c; this deals with getting information
     from the exec server and slicing up the arguments.
     Then it will call `go', above.  */
  _hurd_startup (start_argptr, &go);

  LOSE;
  abort ();
}

void
internal_function
_dl_sysdep_start_cleanup (void)
{
  /* Deallocate the reply port and task port rights acquired by
     __mach_init.  We are done with them now, and the user will
     reacquire them for himself when he wants them.  */
  __mig_dealloc_reply_port (MACH_PORT_NULL);
  __mach_port_deallocate (__mach_task_self (), __mach_task_self_);
}

/* Minimal open/close/mmap implementation sufficient for initial loading of
   shared libraries.  These are weak definitions so that when the
   dynamic linker re-relocates itself to be user-visible (for -ldl),
   it will get the user's definition (i.e. usually libc's).  */

/* Open FILE_NAME and return a Hurd I/O for it in *PORT, or
   return an error.  If STAT is non-zero, stat the file into that stat buffer.  */
static error_t
open_file (const char *file_name, int mode,
	   mach_port_t *port, struct stat *stat)
{
  enum retry_type doretry;
  char retryname[1024];		/* XXX string_t LOSES! */
  file_t startdir, newpt, fileport;
  int dealloc_dir;
  int nloops;
  error_t err;

  assert (!(mode & ~O_READ));

  startdir = _dl_hurd_data->portarray[file_name[0] == '/' ?
				      INIT_PORT_CRDIR : INIT_PORT_CWDIR];

  while (file_name[0] == '/')
    file_name++;

  if (err = __dir_lookup (startdir, (char *)file_name, mode, 0,
			  &doretry, retryname, &fileport))
    return err;

  dealloc_dir = 0;
  nloops = 0;

  while (1)
    {
      if (dealloc_dir)
	__mach_port_deallocate (__mach_task_self (), startdir);
      if (err)
	return err;

      switch (doretry)
	{
	case FS_RETRY_REAUTH:
	  {
	    mach_port_t ref = __mach_reply_port ();
	    err = __io_reauthenticate (fileport, ref, MACH_MSG_TYPE_MAKE_SEND);
	    if (! err)
	      err = __auth_user_authenticate
		(_dl_hurd_data->portarray[INIT_PORT_AUTH],
		 ref, MACH_MSG_TYPE_MAKE_SEND,
		 &newpt);
	    __mach_port_destroy (__mach_task_self (), ref);
	  }
	  __mach_port_deallocate (__mach_task_self (), fileport);
	  if (err)
	    return err;
	  fileport = newpt;
	  /* Fall through.  */

	case FS_RETRY_NORMAL:
#ifdef SYMLOOP_MAX
	  if (nloops++ >= SYMLOOP_MAX)
	    return ELOOP;
#endif

	  /* An empty RETRYNAME indicates we have the final port.  */
	  if (retryname[0] == '\0')
	    {
	      dealloc_dir = 1;
	    opened:
	      /* We have the file open.  Now map it.  */
	      if (stat)
		err = __io_stat (fileport, stat);

	      if (err)
		{
		  if (dealloc_dir)
		    __mach_port_deallocate (__mach_task_self (), fileport);
		}
	      else
		{
		  if (!dealloc_dir)
		    __mach_port_mod_refs (__mach_task_self (), fileport,
					  MACH_PORT_RIGHT_SEND, 1);
		  *port = fileport;
		}

	      return err;
	    }

	  startdir = fileport;
	  dealloc_dir = 1;
	  file_name = retryname;
	  break;

	case FS_RETRY_MAGICAL:
	  switch (retryname[0])
	    {
	    case '/':
	      startdir = _dl_hurd_data->portarray[INIT_PORT_CRDIR];
	      dealloc_dir = 0;
	      if (fileport != MACH_PORT_NULL)
		__mach_port_deallocate (__mach_task_self (), fileport);
	      file_name = &retryname[1];
	      break;

	    case 'f':
	      if (retryname[1] == 'd' && retryname[2] == '/' &&
		  isdigit (retryname[3]))
		{
		  /* We can't use strtol for the decoding here
		     because it brings in hairy locale bloat.  */
		  char *p;
		  int fd = 0;
		  for (p = &retryname[3]; isdigit (*p); ++p)
		    fd = (fd * 10) + (*p - '0');
		  /* Check for excess text after the number.  A slash is
		     valid; it ends the component.  Anything else does not
		     name a numeric file descriptor.  */
		  if (*p != '/' && *p != '\0')
		    return ENOENT;
		  if (fd < 0 || fd >= _dl_hurd_data->dtablesize ||
		      _dl_hurd_data->dtable[fd] == MACH_PORT_NULL)
		    /* If the name was a proper number, but the file
		       descriptor does not exist, we return EBADF instead
		       of ENOENT.  */
		    return EBADF;
		  fileport = _dl_hurd_data->dtable[fd];
		  if (*p == '\0')
		    {
		      /* This descriptor is the file port we want.  */
		      dealloc_dir = 0;
		      goto opened;
		    }
		  else
		    {
		      /* Do a normal retry on the remaining components.  */
		      startdir = fileport;
		      dealloc_dir = 1;
		      file_name = p + 1; /* Skip the slash.  */
		      break;
		    }
		}
	      else
		goto bad_magic;
	      break;

	    case 'm':
	      if (retryname[1] == 'a' && retryname[2] == 'c' &&
		  retryname[3] == 'h' && retryname[4] == 't' &&
		  retryname[5] == 'y' && retryname[6] == 'p' &&
		  retryname[7] == 'e')
		{
		  error_t err;
		  struct host_basic_info hostinfo;
		  mach_msg_type_number_t hostinfocnt = HOST_BASIC_INFO_COUNT;
		  char *p;
		  if (err = __host_info (__mach_host_self (), HOST_BASIC_INFO,
					 (natural_t *) &hostinfo,
					 &hostinfocnt))
		    return err;
		  if (hostinfocnt != HOST_BASIC_INFO_COUNT)
		    return EGRATUITOUS;
		  p = _itoa (hostinfo.cpu_subtype, &retryname[8], 10, 0);
		  *--p = '/';
		  p = _itoa (hostinfo.cpu_type, &retryname[8], 10, 0);
		  if (p < retryname)
		    abort ();	/* XXX write this right if this ever happens */
		  if (p > retryname)
		    strcpy (retryname, p);
		  startdir = fileport;
		  dealloc_dir = 1;
		}
	      else
		goto bad_magic;
	      break;

	    case 't':
	      if (retryname[1] == 't' && retryname[2] == 'y')
		switch (retryname[3])
		  {
		    error_t opentty (file_t *result)
		      {
			error_t err;
			file_t unauth;
			err = __termctty_open_terminal
			  (_dl_hurd_data->portarray[INIT_PORT_CTTYID],
			   mode, &unauth);
			if (! err)
			  {
			    mach_port_t ref = __mach_reply_port ();
			    err = __io_reauthenticate
			      (unauth, ref, MACH_MSG_TYPE_MAKE_SEND);
			    if (! err)
			      err = __auth_user_authenticate
				(_dl_hurd_data->portarray[INIT_PORT_AUTH],
				 ref, MACH_MSG_TYPE_MAKE_SEND,
				 result);
			    __mach_port_deallocate (__mach_task_self (),
						    unauth);
			    __mach_port_destroy (__mach_task_self (), ref);
			  }
			return err;
		      }

		  case '\0':
		    if (err = opentty (&fileport))
		      return err;
		    dealloc_dir = 1;
		    goto opened;
		  case '/':
		    if (err = opentty (&startdir))
		      return err;
		    dealloc_dir = 1;
		    strcpy (retryname, &retryname[4]);
		    break;
		  default:
		    goto bad_magic;
		  }
	      else
		goto bad_magic;
	      break;

	    default:
	    bad_magic:
	      return EGRATUITOUS;
	    }
	  break;

	default:
	  return EGRATUITOUS;
	}

      err = __dir_lookup (startdir, (char *)file_name, mode, 0,
			  &doretry, retryname, &fileport);
    }
}

int weak_function
__open (const char *file_name, int mode, ...)
{
  mach_port_t port;
  error_t err = open_file (file_name, mode, &port, 0);
  if (err)
    return __hurd_fail (err);
  else
    return (int)port;
}

int weak_function
__close (int fd)
{
  if (fd != (int) MACH_PORT_NULL)
    __mach_port_deallocate (__mach_task_self (), (mach_port_t) fd);
  return 0;
}

__ssize_t weak_function
__libc_read (int fd, void *buf, size_t nbytes)
{
  error_t err;
  char *data;
  mach_msg_type_number_t nread;

  data = buf;
  err = __io_read ((mach_port_t) fd, &data, &nread, -1, nbytes);
  if (err)
    return __hurd_fail (err);

  if (data != buf)
    {
      memcpy (buf, data, nread);
      __vm_deallocate (__mach_task_self (), (vm_address_t) data, nread);
    }

  return nread;
}

__ssize_t weak_function
__libc_write (int fd, const void *buf, size_t nbytes)
{
  error_t err;
  mach_msg_type_number_t nwrote;

  assert (fd < _hurd_init_dtablesize);

  err = __io_write (_hurd_init_dtable[fd], buf, nbytes, -1, &nwrote);
  if (err)
    return __hurd_fail (err);

  return nwrote;
}

/* This is only used for printing messages (see dl-misc.c).  */
__ssize_t weak_function
__writev (int fd, const struct iovec *iov, int niov)
{
  int i;
  size_t total = 0;
  for (i = 0; i < niov; ++i)
    total += iov[i].iov_len;

  assert (fd < _hurd_init_dtablesize);

  if (total != 0)
    {
      char buf[total], *bufp = buf;
      error_t err;
      mach_msg_type_number_t nwrote;

      for (i = 0; i < niov; ++i)
	bufp = (memcpy (bufp, iov[i].iov_base, iov[i].iov_len)
		+ iov[i].iov_len);

      err = __io_write (_hurd_init_dtable[fd], buf, total, -1, &nwrote);
      if (err)
	return __hurd_fail (err);

      return nwrote;
    }
  return 0;
}


off_t weak_function
__lseek (int fd, off_t offset, int whence)
{
  error_t err;

  err = __io_seek ((mach_port_t) fd, offset, whence, &offset);
  if (err)
    return __hurd_fail (err);

  return offset;
}

__ptr_t weak_function
__mmap (__ptr_t addr, size_t len, int prot, int flags, int fd, off_t offset)
{
  error_t err;
  vm_prot_t vmprot;
  vm_address_t mapaddr;
  mach_port_t memobj_rd, memobj_wr;

  vmprot = VM_PROT_NONE;
  if (prot & PROT_READ)
    vmprot |= VM_PROT_READ;
  if (prot & PROT_WRITE)
    vmprot |= VM_PROT_WRITE;
  if (prot & PROT_EXEC)
    vmprot |= VM_PROT_EXECUTE;

  if (flags & MAP_ANON)
    memobj_rd = MACH_PORT_NULL;
  else
    {
      assert (!(flags & MAP_SHARED));
      err = __io_map ((mach_port_t) fd, &memobj_rd, &memobj_wr);
      if (err)
	return (__ptr_t) __hurd_fail (err);
      __mach_port_deallocate (__mach_task_self (), memobj_wr);
    }

  mapaddr = (vm_address_t) addr;
  err = __vm_map (__mach_task_self (),
		  &mapaddr, (vm_size_t) len, 0 /*ELF_MACHINE_USER_ADDRESS_MASK*/,
		  !(flags & MAP_FIXED),
		  memobj_rd,
		  (vm_offset_t) offset,
		  flags & (MAP_COPY|MAP_PRIVATE),
		  vmprot, VM_PROT_ALL,
		  (flags & MAP_SHARED) ? VM_INHERIT_SHARE : VM_INHERIT_COPY);
  if (err == KERN_NO_SPACE && (flags & MAP_FIXED))
    {
      /* XXX this is not atomic as it is in unix! */
      /* The region is already allocated; deallocate it first.  */
      err = __vm_deallocate (__mach_task_self (), mapaddr, len);
      if (! err)
	err = __vm_map (__mach_task_self (),
			&mapaddr, (vm_size_t) len, 0 /*ELF_MACHINE_USER_ADDRESS_MASK*/,
			!(flags & MAP_FIXED),
			memobj_rd, (vm_offset_t) offset,
			flags & (MAP_COPY|MAP_PRIVATE),
			vmprot, VM_PROT_ALL,
			(flags & MAP_SHARED)
			? VM_INHERIT_SHARE : VM_INHERIT_COPY);
    }

  if ((flags & MAP_ANON) == 0)
    __mach_port_deallocate (__mach_task_self (), memobj_rd);

  return err ? (__ptr_t) __hurd_fail (err) : (__ptr_t) mapaddr;
}

int weak_function
__fxstat (int vers, int fd, struct stat *buf)
{
  error_t err;

  assert (vers == _STAT_VER);

  err = __io_stat ((mach_port_t) fd, buf);
  if (err)
    return __hurd_fail (err);

  return 0;
}

int weak_function
__xstat (int vers, const char *file, struct stat *buf)
{
  error_t err;
  mach_port_t port;

  assert (vers == _STAT_VER);

  err = open_file (file, 0, &port, buf);
  if (err)
    return __hurd_fail (err);

  __mach_port_deallocate (__mach_task_self (), port);

  return 0;
}

/* This function is called by the dynamic linker (rtld.c) to check
   whether debugging malloc is allowed even for SUID binaries.  This
   stub will always fail, which means that malloc-debugging is always
   disabled for SUID binaries.  */
int weak_function
__access (const char *file, int type)
{
  errno = ENOSYS;
  return -1;
}

pid_t weak_function
__getpid ()
{
  pid_t pid, ppid;
  int orphaned;

  if (__proc_getpids (_dl_hurd_data->portarray[INIT_PORT_PROC],
		      &pid, &ppid, &orphaned))
    return -1;

  return pid;
}

/* This is called only in some strange cases trying to guess a value
   for $ORIGIN for the executable.  The dynamic linker copes with
   getcwd failing (dl-object.c), and it's too much hassle to include
   the functionality here.  (We could, it just requires duplicating or
   reusing getcwd.c's code but using our special lookup function as in
   `open', above.)  */
char *
weak_function
__getcwd (char *buf, size_t size)
{
  errno = ENOSYS;
  return NULL;
}

void weak_function
_exit (int status)
{
  __proc_mark_exit (_dl_hurd_data->portarray[INIT_PORT_PROC],
		    W_EXITCODE (status, 0), 0);
  while (__task_terminate (__mach_task_self ()))
    __mach_task_self_ = (__mach_task_self) ();
}

/* Try to get a machine dependent instruction which will make the
   program crash.  This is used in case everything else fails.  */
#include <abort-instr.h>
#ifndef ABORT_INSTRUCTION
/* No such instruction is available.  */
# define ABORT_INSTRUCTION
#endif

void weak_function
abort (void)
{
  /* Try to abort using the system specific command.  */
  ABORT_INSTRUCTION;

  /* If the abort instruction failed, exit.  */
  _exit (127);

  /* If even this fails, make sure we never return.  */
  while (1)
    /* Try for ever and ever.  */
    ABORT_INSTRUCTION;
}

/* This function is called by interruptible RPC stubs.  For initial
   dynamic linking, just use the normal mach_msg.  Since this defn is
   weak, the real defn in libc.so will override it if we are linked into
   the user program (-ldl).  */

error_t weak_function
_hurd_intr_rpc_mach_msg (mach_msg_header_t *msg,
			 mach_msg_option_t option,
			 mach_msg_size_t send_size,
			 mach_msg_size_t rcv_size,
			 mach_port_t rcv_name,
			 mach_msg_timeout_t timeout,
			 mach_port_t notify)
{
  return __mach_msg (msg, option, send_size, rcv_size, rcv_name,
		     timeout, notify);
}


void
internal_function
_dl_show_auxv (void)
{
  /* There is nothing to print.  Hurd has no auxiliary vector.  */
}


/* Return an array of useful/necessary hardware capability names.  */
const struct r_strlenpair *
internal_function
_dl_important_hwcaps (const char *platform, size_t platform_len, size_t *sz,
		      size_t *max_capstrlen)
{
  struct r_strlenpair *result;

  /* Return an empty array.  Hurd has no hardware capabilities.  */
  result = (struct r_strlenpair *) malloc (sizeof (*result));
  if (result == NULL)
    _dl_signal_error (ENOMEM, NULL, "cannot create capability list");

  result[0].str = (char *) result;	/* Does not really matter.  */
  result[0].len = 0;

  *sz = 1;
  return result;
}

void weak_function
_dl_init_first (int argc, ...)
{
  /* This no-op definition only gets used if libc is not linked in.  */
}
