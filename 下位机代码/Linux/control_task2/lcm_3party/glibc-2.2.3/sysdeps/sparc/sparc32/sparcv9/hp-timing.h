/* High precision, low overhead timing functions.  sparcv9 version.
   Copyright (C) 2001 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Contributed by David S. Miller <davem@redhat.com>, 2001.

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

#ifndef _HP_TIMING_H
#define _HP_TIMING_H	1

#include <string.h>
#include <sys/param.h>
#include <stdio-common/_itoa.h>

#define HP_TIMING_AVAIL		(1)
#define HP_TIMING_INLINE	(1)

typedef unsigned long long int hp_timing_t;

extern hp_timing_t __libc_hp_timing_overhead;

#define HP_TIMING_ZERO(Var)	(Var) = (0)

#define HP_TIMING_NOW(Var) \
      __asm__ __volatile__ ("rd %%tick, %L0\n\t" \
			    "srlx %L0, 32, %H0" \
			    : "=r" (Var))

#define HP_TIMING_DIFF_INIT() \
  do {									      \
    int __cnt = 5;							      \
    __libc_hp_timing_overhead = ~0ull;					      \
    do									      \
      {									      \
	hp_timing_t __t1, __t2;						      \
	HP_TIMING_NOW (__t1);						      \
	HP_TIMING_NOW (__t2);						      \
	if (__t2 - __t1 < __libc_hp_timing_overhead)			      \
	  __libc_hp_timing_overhead = __t2 - __t1;			      \
      }									      \
    while (--__cnt > 0);						      \
  } while (0)

#define HP_TIMING_DIFF(Diff, Start, End)	(Diff) = ((End) - (Start))

#define HP_TIMING_ACCUM(Sum, Diff)				\
do {								\
  hp_timing_t __diff = (Diff) - __libc_hp_timing_overhead;	\
  __asm__ __volatile__("srl	%L0, 0, %%g1\n\t"		\
		       "sllx	%H0, 32, %%g7\n\t"		\
		       "or	%%g1, %%g7, %%g1\n\t"		\
		       "1: ldx	[%1], %%g5\n\t"			\
		       "add	%%g5, %%g1, %%g7\n\t"		\
		       "casx	[%1], %%g5,  %%g7\n\t"		\
		       "cmp	%%g5, %%g7\n\t"			\
		       "bne,pn	%%xcc, 1b\n\t"			\
		       " nop"					\
		       : /* no outputs */			\
		       : "r" (__diff), "r" (&(Sum))		\
		       : "memory", "g1", "g5", "g7");		\
} while(0)

#define HP_TIMING_ACCUM_NT(Sum, Diff)	(Sum) += (Diff)

#define HP_TIMING_PRINT(Buf, Len, Val) \
  do {									      \
    char __buf[20];							      \
    char *__cp = _itoa (Val, __buf + sizeof (__buf), 10, 0);		      \
    int __len = (Len);							      \
    char *__dest = (Buf);						      \
    while (__len-- > 0 && __cp < __buf + sizeof (__buf))		      \
      *__dest++ = *__cp++;						      \
    memcpy (__dest, " clock cycles", MIN (__len, sizeof (" clock cycles")));  \
  } while (0)

#endif	/* hp-timing.h */
