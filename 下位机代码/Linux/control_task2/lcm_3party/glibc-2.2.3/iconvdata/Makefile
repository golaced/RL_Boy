# Copyright (C) 1997, 1998, 1999, 2000, 2001 Free Software Foundation, Inc.
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
# License along with the GNU C Library; see the file COPYING.LIB.  If
# not, write to the Free Software Foundation, Inc.,
# 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

#
#	Makefile for iconv data and code.
#
subdir	:= iconvdata

# Names of all the shared objects which implement the transformations.
modules	:= ISO8859-1 ISO8859-2 ISO8859-3 ISO8859-4 ISO8859-5		 \
	   ISO8859-6 ISO8859-7 ISO8859-8 ISO8859-9 ISO8859-10		 \
	   ISO8859-11 ISO8859-13 ISO8859-14 ISO8859-15 ISO8859-16	 \
	   T.61 ISO_6937 SJIS KOI-8 HP-ROMAN8 EBCDIC-AT-DE		 \
	   EBCDIC-AT-DE-A EBCDIC-CA-FR EUC-KR UHC JOHAB libJIS libKSC	 \
	   BIG5 EUC-JP libGB EUC-CN libCNS EUC-TW ISO646 EBCDIC-DK-NO	 \
	   EBCDIC-DK-NO-A EBCDIC-ES EBCDIC-ES-A EBCDIC-ES-S EBCDIC-FI-SE \
	   EBCDIC-FI-SE-A EBCDIC-FR EBCDIC-IS-FRISS EBCDIC-IT EBCDIC-PT	 \
	   EBCDIC-UK EBCDIC-US IBM037 IBM038 IBM274 IBM275 IBM423	 \
	   IBM500 IBM870 IBM871 IBM891 IBM903 IBM904 IBM905 IBM1047	 \
	   IBM874 CP737 CP775 ISO-2022-KR				 \
	   KOI8-R LATIN-GREEK LATIN-GREEK-1 IBM256 IBM273 IBM277 IBM278	 \
	   IBM280 IBM281 IBM284 IBM285 IBM290 IBM297 IBM420 IBM424	 \
	   IBM437 IBM850 IBM851 IBM852 IBM855 IBM857 IBM860 IBM861	 \
	   IBM862 IBM863 IBM864 IBM865 IBM868 IBM869 IBM875 IBM880	 \
	   IBM866 CP1258 IBM922 IBM1124 IBM1129 IBM932 IBM943		 \
	   IBM856 IBM930 IBM933 IBM935 IBM937 IBM939 IBM1046		 \
	   IBM918 IBM1004 IBM1026 CP1250 CP1251 CP1252 CP1253 CP1254	 \
	   CP1255 CP1256 CP1257 ISO-2022-JP MACINTOSH IEC_P27-1		 \
	   ASMO_449 ANSI_X3.110 CSN_369103 CWI DEC-MCS ECMA-CYRILLIC	 \
	   GOST_19768-74 GREEK-CCITT GREEK7 GREEK7-OLD INIS INIS-8	 \
	   INIS-CYRILLIC ISO_6937-2 ISO_2033 ISO_5427 ISO_5427-EXT	 \
	   ISO_5428 ISO_10367-BOX MAC-IS MAC-UK NATS-DANO NATS-SEFI	 \
	   SAMI-WS2 ISO-IR-197 TIS-620 KOI8-U GBK ISIRI-3342 GBGBK	 \
	   ISO-2022-CN libISOIR165 UTF-16 UNICODE UTF-32 UTF-7 BIG5HKSCS \
	   GB18030 ISO-2022-CN-EXT VISCII GBBIG5

modules.so := $(addsuffix .so, $(modules))

tests = bug-iconv1 bug-iconv2 tst-loading

test-srcs := tst-table-from tst-table-to

include ../Makeconfig

libJIS-routines := jis0201 jis0208 jis0212
libKSC-routines := ksc5601
libGB-routines := gb2312
libCNS-routines := cns11643l1 cns11643
libISOIR165-routines := iso-ir-165

LDFLAGS-EUC-KR.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)EUC-KR.so: $(objpfx)libKSC.so
LDFLAGS-JOHAB.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)JOHAB.so: $(objpfx)libKSC.so
LDFLAGS-UHC.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)UHC.so: $(objpfx)libKSC.so
LDFLAGS-EUC-JP.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)EUC-JP.so: $(objpfx)libJIS.so
LDFLAGS-EUC-CN.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)EUC-CN.so: $(objpfx)libGB.so
LDFLAGS-EUC-TW.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)EUC-TW.so: $(objpfx)libCNS.so

LDFLAGS-ISO-2022-JP.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)ISO-2022-JP.so: $(objpfx)libJIS.so $(objpfx)libGB.so \
			 $(objpfx)libCNS.so $(objpfx)libKSC.so
LDFLAGS-ISO-2022-KR.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)ISO-2022-KR.so: $(objpfx)libKSC.so
LDFLAGS-ISO-2022-CN.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)ISO-2022-CN.so: $(objpfx)libGB.so $(objpfx)libCNS.so
LDFLAGS-ISO-2022-CN-EXT.so = -Wl,-rpath,'$$ORIGIN'
$(objpfx)ISO-2022-CN-EXT.so: $(objpfx)libGB.so $(objpfx)libCNS.so \
			     $(objpfx)libISOIR165.so

LDFLAGS-libJIS.so = -Wl,-soname,$(@F)
LDFLAGS-libKSC.so = -Wl,-soname,$(@F)
LDFLAGS-libGB.so = -Wl,-soname,$(@F)
LDFLAGS-libCNS.so = -Wl,-soname,$(@F)
LDFLAGS-libISOIR165.so = -Wl,-soname,$(@F)

distribute := gconv-modules extra-module.mk gap.awk gaptab.awk		    \
	      gen-8bit.sh gen-8bit-gap.sh gen-8bit-gap-1.sh		    \
	      TESTS $(filter-out testdata/CVS%, $(wildcard testdata/*))	    \
	      TESTS2 run-iconv-test.sh tst-tables.sh tst-table.sh	    \
	      tst-table-charmap.sh tst-table-from.c tst-table-to.c	    \
	      EUC-JP.irreversible ISIRI-3342.irreversible SJIS.irreversible \
	      EUC-KR.irreversible BIG5HKSCS.irreversible BIG5.irreversible  \
	      8bit-generic.c 8bit-gap.c					    \
	      ansi_x3.110.c asmo_449.c big5.c cp737.c cp737.h		    \
	      cp775.c cp775.h ibm874.c cns11643.c cns11643.h		    \
	      cns11643l1.c cns11643l1.h cp1250.c cp1251.c cp1252.c cp1253.c \
	      cp1254.c cp1255.c cp1256.c cp1257.c cp1258.c		    \
	      csn_369103.c cwi.c dec-mcs.c ebcdic-at-de.c ebcdic-at-de-a.c  \
	      ebcdic-ca-fr.c ebcdic-dk-no.c ebcdic-dk-no-a.c ebcdic-es.c    \
	      ebcdic-es-a.c ebcdic-es-s.c ebcdic-fr.c ebcdic-fi-se.c	    \
	      ebcdic-fi-se-a.c ebcdic-is-friss.c ebcdic-it.c ebcdic-pt.c    \
	      ebcdic-uk.c ebcdic-us.c ecma-cyrillic.c euc-cn.c euc-jp.c	    \
	      euc-kr.c euc-tw.c gb2312.c gb2312.h gbk.c gost_19768-74.c	    \
	      greek-ccitt.c greek7.c greek7-old.c hp-roman8.c ibm037.c	    \
	      ibm038.c ibm256.c ibm273.c ibm274.c ibm275.c ibm277.c	    \
	      ibm278.c ibm280.c ibm281.c ibm284.c ibm285.c ibm290.c	    \
	      ibm297.c ibm420.c ibm423.c ibm424.c ibm437.c ibm500.c	    \
	      ibm850.c ibm851.c ibm852.c ibm855.c ibm857.c ibm860.c	    \
	      ibm861.c ibm862.c ibm863.c ibm864.c ibm865.c ibm866.c	    \
	      ibm868.c ibm869.c ibm870.c ibm871.c ibm875.c ibm880.c	    \
	      ibm891.c ibm903.c ibm904.c ibm905.c ibm918.c ibm1004.c	    \
	      ibm856.c ibm930.c ibm933.c ibm935.c ibm937.c ibm939.c ibm1046.c \
	      ibm856.h ibm930.h ibm933.h ibm935.h ibm937.h ibm939.h ibm1046.h \
	      ibm922.c ibm922.h ibm1124.c ibm1124.h ibm1129.c ibm1129.h     \
	      ibm1026.c ibm1047.c iec_p27-1.c inis.c inis-8.c viscii.c	    \
	      inis-cyrillic.c iso-2022-jp.c iso-2022-kr.c iso646.c	    \
	      iso8859-1.c iso8859-2.c iso8859-3.c iso8859-4.c iso8859-5.c   \
	      iso8859-6.c iso8859-7.c iso8859-8.c iso8859-9.c iso8859-10.c  \
	      iso8859-11.c iso8859-11.h iso8859-13.c iso8859-14.c	    \
	      iso8859-15.c iso_2033.c iso_5427-ext.c iso_5427.c iso_5428.c  \
	      iso_6937.c iso_6937-2.c iso_10367-box.c jis0201.c jis0201.h   \
	      jis0208.c jis0208.h jis0212.c jis0212.h johab.c koi-8.c	    \
	      koi8-r.c ksc5601.c ksc5601.h latin-greek.c latin-greek-1.c    \
	      macintosh.c mac-is.c mac-uk.c nats-dano.c nats-sefi.c sjis.c  \
	      t.61.c uhc.c sami-ws2.c iso-ir-197.c tis-620.c koi8-u.c	    \
	      isiri-3342.c isiri-3342.h gbgbk.c iso-2022-cn.c cns11643l2.h  \
	      iso8859-16.c utf-16.c unicode.c utf-32.c utf-7.c big5hkscs.c  \
	      iso-ir-165.c iso-ir-165.h gb18030.c iso-2022-cn-ext.c	    \
	      ibm932.c ibm932.h ibm943.c ibm943.h gbbig5.c

# We build the transformation modules only when we build shared libs.
ifeq (yes,$(build-shared))

# Rule to generate the shared objects.
charmaps = ../localedata/charmaps
-include $(objpfx)iconv-rules
extra-modules-left := $(modules)
include extra-module.mk


extra-objs	+= $(modules.so)
install-others	= $(addprefix $(inst_gconvdir)/, $(modules.so))	\
		  $(inst_gconvdir)/gconv-modules

# We can build the conversion tables for numerous charsets automatically.

gen-8bit-modules := iso8859-2 iso8859-3 iso8859-4 iso8859-6 iso8859-9 koi-8 \
		    hp-roman8 ebcdic-at-de ebcdic-at-de-a ebcdic-ca-fr	    \
		    ebcdic-dk-no ebcdic-dk-no-a ebcdic-es ebcdic-es-a	    \
		    ebcdic-es-s ebcdic-fi-se ebcdic-fi-se-a ebcdic-fr	    \
		    ebcdic-is-friss ebcdic-it ebcdic-pt ebcdic-uk ebcdic-us \
		    ibm037 ibm038 ibm274 ibm275 ibm423 ibm500 ibm870 ibm871 \
		    ibm891 ibm903 ibm904 ibm905 ibm1047 iso8859-16 viscii

gen-8bit-gap-modules := koi8-r latin-greek latin-greek-1 ibm256 ibm273	   \
			ibm277 ibm278 ibm280 ibm281 ibm284 ibm285 ibm290   \
			ibm297 ibm420 ibm424 ibm437 ibm850 ibm851 ibm852   \
			ibm855 ibm857 ibm860 ibm861 ibm862 ibm863 ibm864   \
			ibm865 ibm868 ibm869 ibm875 ibm880 ibm918 ibm1004  \
			ibm1026 cp1250 cp1251 cp1252 cp1253 cp1254 cp1255  \
			cp1256 cp1257 ibm866 iso8859-5 iso8859-7 iso8859-8 \
			iso8859-10 macintosh iec_p27-1 asmo_449		   \
			csn_369103 cwi dec-mcs ecma-cyrillic gost_19768-74 \
			greek-ccitt greek7 greek7-old inis inis-8	   \
			inis-cyrillic iso_2033 iso_5427 iso_5427-ext	   \
			iso_5428 iso_10367-box mac-is nats-dano nats-sefi  \
			iso8859-13 iso8859-14 iso8859-15 mac-uk sami-ws2   \
			iso-ir-197 cp1258 tis-620 koi8-u ibm874

gen-special-modules := iso8859-7jp

generated-modules := $(gen-8bit-modules) $(gen-8bit-gap-modules) \
		     $(gen-special-modules)

generated = $(generated-modules:=.h) $(generated-modules:=.stmp) \
	    iconv-test.out iconv-rules tst-loading.mtrace	 \
	    mtrace-tst-loading tst-tables.out iconv-test.xxx
ifdef objpfx
generated += gconv-modules
endif

# Rules to generate the headers.
export AWK

define generate-8bit-table
$(make-target-directory)
$(SHELL) ./gen-8bit.sh $< > $(@:stmp=T)
$(move-if-change) $(@:stmp=T) $(@:stmp=h)
touch $@
endef

define generate-8bit-gap-table
$(make-target-directory)
$(SHELL) ./gen-8bit-gap.sh $< > $(@:stmp=T)
$(move-if-change) $(@:stmp=T) $(@:stmp=h)
touch $@
endef

# The headers must be generated before the compilation.
before-compile = $(addprefix $(objpfx),$(generated-modules:=.h))

ifndef avoid-generated
$(objpfx)iconv-rules: Makefile
	$(make-target-directory)
	{ echo $(filter-out lib%, $(modules)); \
	  echo 8bit $(gen-8bit-modules); \
	  echo 8bit-gap $(gen-8bit-gap-modules); } | \
	$(AWK) 'NR == 1 { \
		  for (i = 1; i <= NF; i++) \
		    printf "%s-routines := %s\n", $$i, tolower($$i); \
		}; \
		NR > 1 { \
		  for (i = 2; i <= NF; i++) { \
		    printf "$$(objpfx)%s.stmp: ", $$i; \
		    map = toupper($$i); \
		    sub(/^ISO8859/, "ISO-8859", map); \
		    if (map == "ISO_2033") map = map "-1983"; \
		    printf "$$(charmaps)/%s ", map; \
		    printf "gen-%s.sh\n", $$1; \
		    printf "\t$$(generate-%s-table)\n", $$1; \
		  } \
		}' > $@.new
	mv -f $@.new $@
endif

$(generated-modules:%=$(objpfx)%.h): $(objpfx)%.h: $(objpfx)%.stmp
	@:

$(gen-8bit-modules:%=$(objpfx)%.stmp): $(objpfx)%.stmp: $(objpfx)iconv-rules
$(gen-8bit-gap-modules:%=$(objpfx)%.stmp): $(objpfx)%.stmp: $(objpfx)iconv-rules

$(objpfx)iso8859-7jp.stmp: $(charmaps)/ISO-8859-7 gen-8bit-gap-1.sh
	$(make-target-directory)
	$(SHELL) ./gen-8bit-gap-1.sh $< > $(@:stmp=T)
	$(move-if-change) $(@:stmp=T) $(@:stmp=h)
	touch $@

headers: $(addprefix $(objpfx), $(generated-modules:=.h))

$(addprefix $(inst_gconvdir)/, $(modules.so)): \
    $(inst_gconvdir)/%: $(objpfx)% $(+force)
	$(do-install-program)
$(inst_gconvdir)/gconv-modules: gconv-modules $(+force)
	$(do-install)

endif # build-shared = yes

include ../Rules

ifeq (no,$(cross-compiling))
ifeq (yes,$(build-shared))
tests: $(objpfx)iconv-test.out $(objpfx)tst-tables.out
ifneq (no,$(PERL))
tests: $(objpfx)mtrace-tst-loading
endif
endif
endif

tst-loading-ENV = MALLOC_TRACE=$(objpfx)tst-loading.mtrace
$(objpfx)mtrace-tst-loading: $(objpfx)tst-loading.out
	$(common-objpfx)malloc/mtrace $(objpfx)tst-loading.mtrace > $@

$(objpfx)bug-iconv1.out: $(objpfx)gconv-modules
$(objpfx)bug-iconv2.out: $(objpfx)gconv-modules

$(objpfx)iconv-test.out: run-iconv-test.sh $(objpfx)gconv-modules \
			 $(addprefix $(objpfx),$(modules.so)) \
			 $(common-objdir)/iconv/iconv_prog TESTS
	$(SHELL) -e $< $(common-objdir) > $@

$(objpfx)tst-tables.out: tst-tables.sh $(objpfx)gconv-modules \
			 $(addprefix $(objpfx),$(modules.so)) \
			 $(objpfx)tst-table-from $(objpfx)tst-table-to
	$(SHELL) $< $(common-objpfx) $(common-objpfx)iconvdata/ > $@

do-tests-clean common-mostlyclean: tst-tables-clean

.PHONY: tst-tables-clean
tst-tables-clean:
	-rm -f $(objpfx)tst-*.table $(objpfx)tst-EUC-TW.irreversible

ifdef objpfx
$(objpfx)gconv-modules: gconv-modules
	cp $^ $@
endif
