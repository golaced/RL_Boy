/* Copyright (C) 1997, 1999, 2001 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Contributed by Thorsten Kukuk <kukuk@vt.uni-paderborn.de>, 1997.

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

#include <nss.h>
#include <errno.h>
#include <pwd.h>
#include <string.h>
#include <bits/libc-lock.h>
#include <rpcsvc/nis.h>

#include "nss-nisplus.h"
#include "nisplus-parser.h"

__libc_lock_define_initialized (static, lock)

static nis_result *result;
static nis_name tablename_val;
static u_long tablename_len;

static enum nss_status
_nss_create_tablename (int *errnop)
{
  if (tablename_val == NULL)
    {
      char buf [40 + strlen (nis_local_directory ())];
      char *p;

      p = __stpcpy (buf, "passwd.org_dir.");
      p = __stpcpy (p, nis_local_directory ());
      tablename_val = __strdup (buf);
      if (tablename_val == NULL)
	{
	  *errnop = errno;
	  return NSS_STATUS_TRYAGAIN;
	}
      tablename_len = strlen (tablename_val);
    }
  return NSS_STATUS_SUCCESS;
}


enum nss_status
_nss_nisplus_setpwent (int stayopen)
{
  enum nss_status status = NSS_STATUS_SUCCESS;
  int err;

  __libc_lock_lock (lock);

  if (result)
    nis_freeresult (result);
  result = NULL;

  if (tablename_val == NULL)
    status = _nss_create_tablename (&err);

  __libc_lock_unlock (lock);

  return status;
}

enum nss_status
_nss_nisplus_endpwent (void)
{
  __libc_lock_lock (lock);

  if (result)
    nis_freeresult (result);
  result = NULL;

  __libc_lock_unlock (lock);

  return NSS_STATUS_SUCCESS;
}

static enum nss_status
internal_nisplus_getpwent_r (struct passwd *pw, char *buffer, size_t buflen,
			     int *errnop)
{
  int parse_res;

  /* Get the next entry until we found a correct one. */
  do
    {
      nis_result *saved_res;

      if (result == NULL)
	{
	  saved_res = NULL;
          if (tablename_val == NULL)
	    {
	      enum nss_status status = _nss_create_tablename (errnop);

	      if (status != NSS_STATUS_SUCCESS)
		return status;
	    }

	  result = nis_first_entry (tablename_val);
	  if (niserr2nss (result->status) != NSS_STATUS_SUCCESS)
	    return niserr2nss (result->status);
	}
      else
	{
	  nis_result *res;

	  saved_res = result;
	  res = nis_next_entry (tablename_val, &result->cookie);
	  result = res;
	  if (niserr2nss (result->status) != NSS_STATUS_SUCCESS)
	    {
	      nis_freeresult (saved_res);
	      return niserr2nss (result->status);
	    }
	}

      parse_res = _nss_nisplus_parse_pwent (result, pw, buffer,
					    buflen, errnop);
      if (parse_res == -1)
	{
	  nis_freeresult (result);
	  result = saved_res;
	  *errnop = ERANGE;
	  return NSS_STATUS_TRYAGAIN;
	}
      else
	{
	  if (saved_res)
	    nis_freeresult (saved_res);
	}
    } while (!parse_res);

  return NSS_STATUS_SUCCESS;
}

enum nss_status
_nss_nisplus_getpwent_r (struct passwd *result, char *buffer, size_t buflen,
			 int *errnop)
{
  int status;

  __libc_lock_lock (lock);

  status = internal_nisplus_getpwent_r (result, buffer, buflen, errnop);

  __libc_lock_unlock (lock);

  return status;
}

enum nss_status
_nss_nisplus_getpwnam_r (const char *name, struct passwd *pw,
			 char *buffer, size_t buflen, int *errnop)
{
  int parse_res;

  if (tablename_val == NULL)
    {
      enum nss_status status = _nss_create_tablename (errnop);

      if (status != NSS_STATUS_SUCCESS)
	return status;
    }

  if (name == NULL)
    {
      *errnop = EINVAL;
      return NSS_STATUS_UNAVAIL;
    }
  else
    {
      nis_result *result;
      char buf[strlen (name) + 24 + tablename_len];

      sprintf (buf, "[name=%s],%s", name, tablename_val);

      result = nis_list(buf, FOLLOW_PATH | FOLLOW_LINKS, NULL, NULL);

      if (niserr2nss (result->status) != NSS_STATUS_SUCCESS)
	{
	  enum nss_status status =  niserr2nss (result->status);

	  nis_freeresult (result);
	  return status;
	}

      parse_res = _nss_nisplus_parse_pwent (result, pw, buffer, buflen,
					    errnop);

      nis_freeresult (result);

      if (parse_res < 1)
	{
	  if (parse_res == -1)
	    {
	      *errnop = ERANGE;
	      return NSS_STATUS_TRYAGAIN;
	    }
	  else
	    return NSS_STATUS_NOTFOUND;
	}
      return NSS_STATUS_SUCCESS;
    }
}

enum nss_status
_nss_nisplus_getpwuid_r (const uid_t uid, struct passwd *pw,
			 char *buffer, size_t buflen, int *errnop)
{
  if (tablename_val == NULL)
    {
      enum nss_status status = _nss_create_tablename (errnop);

      if (status != NSS_STATUS_SUCCESS)
	return status;
    }

  {
    int parse_res;
    nis_result *result;
    char buf[100 + tablename_len];

    sprintf (buf, "[uid=%d],%s", uid, tablename_val);

    result = nis_list(buf, FOLLOW_PATH | FOLLOW_LINKS, NULL, NULL);

    if (niserr2nss (result->status) != NSS_STATUS_SUCCESS)
      {
	enum nss_status status = niserr2nss (result->status);

	nis_freeresult (result);
	return status;
      }

    parse_res = _nss_nisplus_parse_pwent (result, pw, buffer, buflen, errnop);

    nis_freeresult (result);

    if (parse_res < 1)
      {
	if (parse_res == -1)
	  {
	    *errnop = ERANGE;
	    return NSS_STATUS_TRYAGAIN;
	  }
	else
	  return NSS_STATUS_NOTFOUND;
      }
    return NSS_STATUS_SUCCESS;
  }
}
