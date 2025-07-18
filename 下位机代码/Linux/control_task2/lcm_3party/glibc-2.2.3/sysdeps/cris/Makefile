# Copyright (C) 1999, 2000, 2001 Free Software Foundation, Inc.
# This file is part of the GNU C Library.

# The GNU C Library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Library General Public License
# as published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.

# The GNU C Library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Library General Public License for more details.

# You should have received a copy of the GNU Library General Public
# License along with the GNU C Library; see the file COPYING.LIB.  If not,
# write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

# We don't support long doubles as a distinct type.  We don't need to set
# this variable; it's here mostly for documentational purposes.

long-double-fcts = no

# FIXME: Note that this is a machine-specific stub; profiling is not
# implemented.
ifeq ($(subdir),gmon)
sysdep_routines += _mcount
endif

# FIXME: This test seems generally bogus.  Wrong types in function calls
# and assumes FE_TONEAREST is defined.  Does it work somewhere?
# Presumably it does, so let's settle for filtering it out target-wise
# until it's agreed what should be done.
ifeq ($(subdir),math)
tests := $(filter-out test-fenv, $(tests))
endif

# PIC code must be assembled with special options, passed on by gcc when
# given the -fpic option.
ASFLAGS-.os = -fpic

# Overflow occurs at 2**15/4 (8192) symbols.  Glibc uses about 2000.
# So use -fpic: smaller-size relocs; smaller, faster code.
pic-ccflag = -fpic
