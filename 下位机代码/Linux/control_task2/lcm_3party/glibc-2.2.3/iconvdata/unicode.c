/* Conversion module for Unicode
   Copyright (C) 1999, 2000 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Contributed by Ulrich Drepper <drepper@cygnus.com>, 1999.

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

#include <byteswap.h>
#include <dlfcn.h>
#include <gconv.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* This is the Byte Order Mark character (BOM).  */
#define BOM	0xfeff
/* And in the other endian format.  */
#define BOM_OE	0xfffe


/* Definitions used in the body of the `gconv' function.  */
#define FROM_LOOP		from_unicode_loop
#define TO_LOOP			to_unicode_loop
#define DEFINE_INIT		0
#define DEFINE_FINI		0
#define MIN_NEEDED_FROM		2
#define MIN_NEEDED_TO		4
#define FROM_DIRECTION		(dir == from_unicode)
#define PREPARE_LOOP \
  enum direction dir = ((struct unicode_data *) step->__data)->dir;	      \
  int swap;								      \
  if (FROM_DIRECTION)							      \
    {									      \
      if (data->__invocation_counter == 0)				      \
	{								      \
	  /* We have to find out which byte order the file is encoded in.  */ \
	  if (inptr + 2 > inend)					      \
	    return __GCONV_EMPTY_INPUT;					      \
									      \
	  if (get16u (inptr) == BOM)					      \
	    /* Simply ignore the BOM character.  */			      \
	    *inptrp = inptr += 2;					      \
	  else if (get16u (inptr) == BOM_OE)				      \
	    {								      \
	      ((struct unicode_data *) step->__data)->swap = 1;		      \
	      *inptrp = inptr += 2;					      \
	    }								      \
	}								      \
    }									      \
  else if (!data->__internal_use && data->__invocation_counter == 0)	      \
    {									      \
      /* Emit the Byte Order Mark.  */					      \
      if (__builtin_expect (outbuf + 2 > outend, 0))			      \
	return __GCONV_FULL_OUTPUT;					      \
									      \
      put16u (outbuf, BOM);						      \
      outbuf += 2;							      \
    }									      \
  swap = ((struct unicode_data *) step->__data)->swap;
#define EXTRA_LOOP_ARGS		, swap


/* Direction of the transformation.  */
enum direction
{
  illegal_dir,
  to_unicode,
  from_unicode
};

struct unicode_data
{
  enum direction dir;
  int swap;
};


extern int gconv_init (struct __gconv_step *step);
int
gconv_init (struct __gconv_step *step)
{
  /* Determine which direction.  */
  struct unicode_data *new_data;
  enum direction dir = illegal_dir;
  int result;

  if (strcmp (step->__from_name, "UNICODE//") == 0)
    dir = from_unicode;
  else
    dir = to_unicode;

  new_data = (struct unicode_data *) malloc (sizeof (struct unicode_data));

  result = __GCONV_NOMEM;
  if (new_data != NULL)
    {
      new_data->dir = dir;
      new_data->swap = 0;
      step->__data = new_data;

      if (dir == from_unicode)
	{
	  step->__min_needed_from = MIN_NEEDED_FROM;
	  step->__max_needed_from = MIN_NEEDED_FROM;
	  step->__min_needed_to = MIN_NEEDED_TO;
	  step->__max_needed_to = MIN_NEEDED_TO;
	}
      else
	{
	  step->__min_needed_from = MIN_NEEDED_TO;
	  step->__max_needed_from = MIN_NEEDED_TO;
	  step->__min_needed_to = MIN_NEEDED_FROM;
	  step->__max_needed_to = MIN_NEEDED_FROM;
	}

      step->__stateful = 0;

      result = __GCONV_OK;
    }

  return result;
}


extern void gconv_end (struct __gconv_step *data);
void
gconv_end (struct __gconv_step *data)
{
  free (data->__data);
}


/* Convert from the internal (UCS4-like) format to UCS2.  */
#define MIN_NEEDED_INPUT	MIN_NEEDED_TO
#define MIN_NEEDED_OUTPUT	MIN_NEEDED_FROM
#define LOOPFCT			TO_LOOP
#define BODY \
  {									      \
    uint32_t c = get32 (inptr);						      \
									      \
    if (__builtin_expect (c, 0) >= 0x10000)				      \
      {									      \
	STANDARD_ERR_HANDLER (4);					      \
      }									      \
    else if (__builtin_expect (c >= 0xd800 && c < 0xe000, 0))		      \
      {									      \
	/* Surrogate characters in UCS-4 input are not valid.		      \
	   We must catch this, because the UCS-2 output might be	      \
	   interpreted as UTF-16 by other programs.  If we let		      \
	   surrogates pass through, attackers could make a security	      \
	   hole exploit by synthesizing any desired plane 1-16		      \
	   character.  */						      \
	if (! ignore_errors_p ())					      \
	  {								      \
	    result = __GCONV_ILLEGAL_INPUT;				      \
	    break;							      \
	  }								      \
	inptr += 4;							      \
	++*irreversible;						      \
	continue;							      \
      }									      \
    else								      \
      {									      \
	put16 (outptr, c);						      \
	outptr += 2;							      \
      }									      \
									      \
    inptr += 4;								      \
  }
#define LOOP_NEED_FLAGS
#define EXTRA_LOOP_DECLS \
	, int swap
#include <iconv/loop.c>


/* Convert from UCS2 to the internal (UCS4-like) format.  */
#define MIN_NEEDED_INPUT	MIN_NEEDED_FROM
#define MIN_NEEDED_OUTPUT	MIN_NEEDED_TO
#define LOOPFCT			FROM_LOOP
#define BODY \
  {									      \
    uint16_t u1 = get16 (inptr);					      \
									      \
    if (swap)								      \
      u1 = bswap_16 (u1);						      \
									      \
    if (__builtin_expect (u1 >= 0xd800 && u1 < 0xe000, 0))		      \
      {									      \
	/* Surrogate characters in UCS-2 input are not valid.  Reject	      \
	   them.  (Catching this here is not security relevant.)  */	      \
	if (! ignore_errors_p ())					      \
	  {								      \
	    result = __GCONV_ILLEGAL_INPUT;				      \
	    break;							      \
	  }								      \
	inptr += 2;							      \
	++*irreversible;						      \
	continue;							      \
      }									      \
									      \
    put32 (outptr, u1);							      \
									      \
    inptr += 2;								      \
    outptr += 4;							      \
  }
#define LOOP_NEED_FLAGS
#define EXTRA_LOOP_DECLS \
	, int swap
#include <iconv/loop.c>


/* Now define the toplevel functions.  */
#include <iconv/skeleton.c>
