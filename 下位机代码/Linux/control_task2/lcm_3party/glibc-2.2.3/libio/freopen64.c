/* Copyright (C) 1993,1995,1996,1997,1998,2000,2001 Free Software Foundation, Inc.
   This file is part of the GNU IO Library.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 2, or (at
   your option) any later version.

   This library is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this library; see the file COPYING.  If not, write to
   the Free Software Foundation, 59 Temple Place - Suite 330, Boston,
   MA 02111-1307, USA.

   As a special exception, if you link this library with files
   compiled with a GNU compiler to produce an executable, this does
   not cause the resulting executable to be covered by the GNU General
   Public License.  This exception does not however invalidate any
   other reasons why the executable file might be covered by the GNU
   General Public License.  */

#include "libioP.h"
#include "stdio.h"

#include <fd_to_filename.h>

FILE *
freopen64 (filename, mode, fp)
     const char* filename;
     const char* mode;
     FILE *fp;
{
#ifdef _G_OPEN64
  FILE *result;
  int fd = -1;
  CHECK_FILE (fp, NULL);
  if (!(fp->_flags & _IO_IS_FILEBUF))
    return NULL;
  _IO_cleanup_region_start ((void (*) __P ((void *))) _IO_funlockfile, fp);
  _IO_flockfile (fp);
  if (filename == NULL && _IO_fileno (fp) >= 0)
    {
      fd = dup (_IO_fileno (fp));
      if (fd != -1)
	filename = fd_to_filename (fd);
    }
  result = _IO_freopen64 (filename, mode, fp);
  if (result != NULL)
    /* unbound stream orientation */
    result->_mode = 0;
  if (fd != -1)
    {
      close (fd);
      if (filename != NULL)
	free ((char *) filename);
    }
  _IO_funlockfile (fp);
  _IO_cleanup_region_end (0);
  return result;
#else
  __set_errno (ENOSYS);
  return NULL;
#endif
}
