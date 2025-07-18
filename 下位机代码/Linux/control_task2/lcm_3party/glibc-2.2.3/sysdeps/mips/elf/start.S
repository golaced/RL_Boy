/* Startup code compliant to the ELF Mips ABI.
   Copyright (C) 1995, 1997, 2000, 2001 Free Software Foundation, Inc.
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

#define __ASSEMBLY__ 1
#include <entry.h>

#ifndef ENTRY_POINT
#error ENTRY_POINT needs to be defined for start.S on MIPS/ELF.
#endif

/* This is the canonical entry point, usually the first thing in the text
   segment.  The SVR4/Mips ABI (pages 3-31, 3-32) says that when the entry
   point runs, most registers' values are unspecified, except for:

   v0 ($2)	Contains a function pointer to be registered with `atexit'.
		This is how the dynamic linker arranges to have DT_FINI
		functions called for shared libraries that have been loaded
		before this code runs.

   sp ($29)	The stack contains the arguments and environment:
		0(%esp)			argc
		4(%esp)			argv[0]
		...
		(4*argc)(%esp)		NULL
		(4*(argc+1))(%esp)	envp[0]
		...
					NULL
   ra ($31)	The return address register is set to zero so that programs
		that search backword through stack frames recognize the last
		stack frame.
*/


/* We need to call:
   __libc_start_main (int (*main) (int, char **, char **), int argc,
		      char **argv, void (*init) (void), void (*fini) (void),
		      void (*rtld_fini) (void), void *stack_end)
*/
#ifdef __PIC__
/* A macro to (re)initialize gp. We can get the run time address of 0f in
   ra ($31) by blezal instruction. In this early phase, we can't save gp
   in stack and .cprestore doesn't work properly. So we set gp by using
   this macro. */
#define SET_GP \
	.set noreorder;	\
	bltzal $0,0f;	\
	nop;		\
0:	.cpload $31;	\
	.set reorder;
#endif

	.text
	.globl ENTRY_POINT
ENTRY_POINT:
#ifdef __PIC__
	SET_GP
#endif
	move $31, $0

	la $4, main		/* main */
	lw $5, 0($29)		/* argc */
	addu $6, $29, 4		/* argv  */
	/* Allocate space on the stack for seven arguments and make sure
	   the stack is aligned to double words (8 bytes).  */
	and $29, 0xfffffff8
	subu $29, 32
	la $7, _init		/* init */
	la $8, _fini
	sw $8, 16($29)		/* fini */
	sw $2, 20($29)		/* rtld_fini */
	sw $29, 24($29)		/* stack_end */
	jal __libc_start_main
hlt:	b hlt			/* Crash if somehow it does return.  */

/* Define a symbol for the first piece of initialized data.  */
	.data
	.globl __data_start
__data_start:
	.long 0
	.weak data_start
	data_start = __data_start
