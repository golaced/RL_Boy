# Copyright (C) 1996, 1997, 1998, 2001 Free Software Foundation, Inc.
# This file is part of the GNU C Library.

# The GNU C Library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Library General Public License as
# published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.

# The GNU C Library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Library General Public License for more details.

# You should have received a copy of the GNU Library General Public
# License along with the GNU C Library; see the file COPYING.LIB.  If not,
# write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

#
#	Makefile for NIS/NIS+ part.
#
subdir	:= nis

headers			:= $(wildcard rpcsvc/*.[hx])
distribute		:= nss-nis.h nss-nisplus.h nis_intern.h Banner \
			nisplus-parser.h nis_xdr.h

# These are the databases available for the nis (and perhaps later nisplus)
# service.  This must be a superset of the services in nss.
databases		= proto service hosts network grp pwd rpc ethers \
			  spwd netgrp alias publickey

# Specify rules for the nss_* modules.
services		:= nis nisplus compat

extra-libs		= libnsl $(services:%=libnss_%)
# These libraries will be built in the `others' pass rather than
# the `lib' pass, because they depend on libc.so being built already.
extra-libs-others	= $(extra-libs)

# The sources are found in the appropriate subdir.
subdir-dirs = $(services:%=nss_%)
vpath %.c $(subdir-dirs)

libnsl-routines = yp_xdr ypclnt ypupdate_xdr \
                  nis_subr nis_local_names nis_free nis_file \
                  nis_print nis_error nis_call nis_lookup\
                  nis_table nis_xdr nis_server nis_ping \
		  nis_checkpoint nis_mkdir nis_rmdir nis_getservlist\
		  nis_verifygroup nis_ismember nis_addmember nis_util\
		  nis_removemember nis_creategroup nis_destroygroup\
		  nis_print_group_entry nis_domain_of nis_domain_of_r\
		  nis_modify nis_remove nis_add nis_defaults\
		  nis_findserv nis_callback nis_clone_dir nis_clone_obj\
		  nis_clone_res

libnss_compat-routines	:= $(addprefix compat-,grp pwd spwd initgroups) \
			   nisplus-parser nss-nis
libnss_compat-inhibit-o	= $(filter-out .os,$(object-suffixes))

libnss_nis-routines	:= $(addprefix nis-,$(databases)) nis-initgroups \
			   nss-nis
libnss_nis-inhibit-o	= $(filter-out .os,$(object-suffixes))

libnss_nisplus-routines	:= $(addprefix nisplus-,$(databases)) nisplus-parser \
			   nss-nisplus
libnss_nisplus-inhibit-o = $(filter-out .os,$(object-suffixes))

include ../Rules


$(objpfx)libnss_compat.so: $(objpfx)libnsl.so$(libnsl.so-version)
$(objpfx)libnss_nis.so: $(objpfx)libnsl.so$(libnsl.so-version) \
			$(common-objpfx)nss/libnss_files.so
$(objpfx)libnss_nisplus.so: $(objpfx)libnsl.so$(libnsl.so-version)

# Depend on libc.so so a DT_NEEDED is generated in the shared objects.
# This ensures they will load libc.so for needed symbols if loaded by
# a statically-linked program that hasn't already loaded it.
$(services:%=$(objpfx)libnss_%.so) $(objpfx)libnsl.so: $(common-objpfx)libc.so\
					       $(common-objpfx)libc_nonshared.a


ifeq ($(build-shared),yes)
$(others:%=$(objpfx)%): $(objpfx)libnsl.so$(libnsl.so-version)
else
$(others:%=$(objpfx)%): $(objpfx)libnsl.a
endif
