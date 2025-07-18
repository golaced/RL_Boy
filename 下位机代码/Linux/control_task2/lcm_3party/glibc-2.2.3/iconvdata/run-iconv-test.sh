#! /bin/sh -f
# Run available iconv(1) tests.
# Copyright (C) 1998, 1999, 2000, 2001 Free Software Foundation, Inc.
# This file is part of the GNU C Library.
# Contributed by Ulrich Drepper <drepper@cygnus.com>, 1998.
#
# The GNU C Library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Library General Public License as
# published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.
#
# The GNU C Library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Library General Public License for more details.
#
# You should have received a copy of the GNU Library General Public
# License along with the GNU C Library; see the file COPYING.LIB.  If not,
# write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

codir=$1

# We use always the same temporary file.
temp1=$codir/iconvdata/iconv-test.xxx
temp2=$codir/iconvdata/iconv-test.yyy

trap "rm -f $temp1 $temp2" 1 2 3 15

# We must tell the iconv(1) program where the modules we want to use can
# be found.
GCONV_PATH=$codir/iconvdata
export GCONV_PATH

# We have to have some directories in the library path.
LIBPATH=$codir:$codir/iconvdata

# How the start the iconv(1) program.
ICONV='$codir/elf/ld.so --library-path $LIBPATH --inhibit-rpath ${from}.so \
       $codir/iconv/iconv_prog'

# Which echo?
if (echo "testing\c"; echo 1,2,3) | grep c >/dev/null; then
  ac_n=-n ac_c= ac_t=
else
  ac_n= ac_c='\c' ac_t=
fi

# We read the file named TESTS.  All non-empty lines not starting with
# `#' are interpreted as commands.
failed=0
while read from to subset targets; do
  # Ignore empty and comment lines.
  if test -z "$subset" || test "$from" = '#'; then continue; fi

  # Expand the variables now.
  PROG=`eval echo $ICONV`

  if test -n "$targets"; then
    for t in $targets; do
      if test -f testdata/$from; then
	echo $ac_n "   test data: $from -> $t $ac_c"
	$PROG -f $from -t $t testdata/$from > $temp1 ||
	  { if test $? -gt 128; then exit 1; fi
	    echo "FAILED"; failed=1; continue; }
	echo $ac_n "OK$ac_c"
	if test -s testdata/$from..$t; then
	  cmp $temp1 testdata/$from..$t > /dev/null 2>&1 ||
	    { echo "/FAILED"; failed=1; continue; }
	  echo $ac_n "/OK$ac_c"
	fi
	echo $ac_n " -> $from $ac_c"
	$PROG -f $t -t $to -o $temp2 $temp1 ||
	  { if test $? -gt 128; then exit 1; fi
	    echo "FAILED"; failed=1; continue; }
	echo $ac_n "OK$ac_c"
	test -s $temp1 && cmp testdata/$from $temp2 > /dev/null 2>&1 ||
	  { echo "/FAILED"; failed=1; continue; }
	echo "/OK"
	rm -f $temp1 $temp2
      fi

      # Now test some bigger text, entirely in ASCII.  If ASCII is no subset
      # of the coded character set we convert the text to this coded character
      # set.  Otherwise we convert to all the TARGETS.
      if test $subset = Y; then
	echo $ac_n "      suntzu: $from -> $t -> $to $ac_c"
	$PROG -f $from -t $t testdata/suntzus |
	$PROG -f $t -t $to > $temp1 ||
	  { if test $? -gt 128; then exit 1; fi
	    echo "FAILED"; failed=1; continue; }
	echo $ac_n "OK$ac_c"
	cmp testdata/suntzus $temp1 ||
	  { echo "/FAILED"; failed=1; continue; }
	echo "/OK"
      fi
      rm -f $temp1

      # And tests where iconv(1) has to handle charmaps.
      if test "$t" = UTF8; then tc=UTF-8; else tc="$t"; fi
      if test -f ../localedata/charmaps/$from &&
         test -f ../localedata/charmaps/$tc &&
	 test -f testdata/$from; then
	echo $ac_n "test charmap: $from -> $t $ac_c"
	$PROG -f ../localedata/charmaps/$from -t ../localedata/charmaps/$tc \
	      testdata/$from > $temp1 ||
	  { if test $? -gt 128; then exit 1; fi
	    echo "FAILED"; failed=1; continue; }
	echo $ac_n "OK$ac_c"
	if test -s testdata/$from..$t; then
	  cmp $temp1 testdata/$from..$t > /dev/null 2>&1 ||
	    { echo "/FAILED"; failed=1; continue; }
	  echo $ac_n "/OK$ac_c"
	fi
	echo $ac_n " -> $from $ac_c"
	$PROG -t ../localedata/charmaps/$from -f ../localedata/charmaps/$tc \
	      -o $temp2 $temp1 ||
	  { if test $? -gt 128; then exit 1; fi
	    echo "FAILED"; failed=1; continue; }
	echo $ac_n "OK$ac_c"
	test -s $temp1 && cmp testdata/$from $temp2 > /dev/null 2>&1 ||
	  { echo "/FAILED"; failed=1; continue; }
	echo "/OK"
	rm -f $temp1 $temp2
      fi
    done
  fi

  if test "$subset" != Y; then
    echo $ac_n "      suntzu: ASCII -> $to -> ASCII $ac_c"
    $PROG -f ASCII -t $to testdata/suntzus |
    $PROG -f $to -t ASCII > $temp1 ||
      { if test $? -gt 128; then exit 1; fi
	echo "FAILED"; failed=1; continue; }
    echo $ac_n "OK$ac_c"
    cmp testdata/suntzus $temp1 ||
      { echo "/FAILED"; failed=1; continue; }
    echo "/OK"
  fi
done < TESTS

# We read the file named TESTS2.  All non-empty lines not starting with
# `#' are interpreted as commands.
while read utf8 from filename; do
  # Ignore empty and comment lines.
  if test -z "$filename" || test "$utf8" = '#'; then continue; fi

  # Expand the variables now.
  PROG=`eval echo $ICONV`

  # Test conversion to the endianness dependent encoding.
  echo $ac_n "test encoder: $utf8 -> $from $ac_c"
  $PROG -f $utf8 -t $from < testdata/${filename}..${utf8} > $temp1
  cmp $temp1 testdata/${filename}..${from}.BE > /dev/null 2>&1 ||
  cmp $temp1 testdata/${filename}..${from}.LE > /dev/null 2>&1 ||
    { echo "/FAILED"; failed=1; continue; }
  echo "OK"

  # Test conversion from the endianness dependent encoding.
  echo $ac_n "test decoder: $from -> $utf8 $ac_c"
  $PROG -f $from -t $utf8 < testdata/${filename}..${from}.BE > $temp1
  cmp $temp1 testdata/${filename}..${utf8} > /dev/null 2>&1 ||
    { echo "/FAILED"; failed=1; continue; }
  $PROG -f $from -t $utf8 < testdata/${filename}..${from}.LE > $temp1
  cmp $temp1 testdata/${filename}..${utf8} > /dev/null 2>&1 ||
    { echo "/FAILED"; failed=1; continue; }
  echo "OK"

  # Test byte swapping behaviour.
  echo $ac_n "test non-BOM: ${from}BE -> ${from}LE $ac_c"
  $PROG -f ${from}BE -t ${from}LE < testdata/${filename}..${from}.BE > $temp1
  cmp $temp1 testdata/${filename}..${from}.LE > /dev/null 2>&1 ||
    { echo "/FAILED"; failed=1; continue; }
  echo "OK"

  # Test byte swapping behaviour.
  echo $ac_n "test non-BOM: ${from}LE -> ${from}BE $ac_c"
  $PROG -f ${from}LE -t ${from}BE < testdata/${filename}..${from}.LE > $temp1
  cmp $temp1 testdata/${filename}..${from}.BE > /dev/null 2>&1 ||
    { echo "/FAILED"; failed=1; continue; }
  echo "OK"

done < TESTS2

exit $failed
# Local Variables:
#  mode:shell-script
# End:
