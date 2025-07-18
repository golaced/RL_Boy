/* Copyright (C) 1993, 1994, 1995, 1996, 2001 Free Software Foundation, Inc.
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

/* Based on CMU's mach_msg_server.c revision 2.4 of 91/05/14, and thus
   under the following copyright.  Rewritten by Roland McGrath (FSF)
   93/12/06 to use stack space instead of malloc, and to handle
   large messages with MACH_RCV_LARGE.  */

/*
 * Mach Operating System
 * Copyright (c) 1991,1990 Carnegie Mellon University
 * All Rights Reserved.
 *
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie Mellon
 * the rights to redistribute these changes.
 */
/*
 * HISTORY
 * $Log: msgserver.c,v $
 * Revision 1.6  2001/04/09 21:23:38  roland
 * 2001-04-07  Roland McGrath  <roland@frob.com>
 *
 * 	* mach/msgserver.c (__mach_msg_server_timeout): Add an assert.
 *
 * Revision 1.5  1996/12/20 01:32:35  drepper
 * Update from main archive 961219
 *
 * Revision 1.5  1996/12/19 20:23:45  drepper
 * Spelling corrections.
 *
 * Revision 1.4  1996/01/29 15:44:23  roland
 * Declare DEMUX arg with prototype.
 *
 * Revision 1.3  1995/01/21  15:00:57  roland
 * Converted to use weak aliases with macros from libc-symbols.h.
 *
 * Revision 1.2  1994/10/10  07:20:14  roland
 * Increase default MAX_SIZE to two pages.
 *
 * Revision 1.1  1993/12/06  23:25:25  roland
 * entered into RCS
 *
 * Revision 2.4  91/05/14  17:53:22  mrt
 * 	Correcting copyright
 *
 * Revision 2.3  91/02/14  14:17:47  mrt
 * 	Added new Mach copyright
 * 	[91/02/13  12:44:20  mrt]
 *
 * Revision 2.2  90/08/06  17:23:58  rpd
 * 	Created.
 *
 */


#include <mach.h>
#include <mach/mig_errors.h>
#include <stdlib.h>		/* For malloc and free.  */
#include <assert.h>

mach_msg_return_t
__mach_msg_server_timeout (boolean_t (*demux) (mach_msg_header_t *request,
					       mach_msg_header_t *reply),
			   mach_msg_size_t max_size,
			   mach_port_t rcv_name,
			   mach_msg_option_t option,
			   mach_msg_timeout_t timeout)
{
  register mig_reply_header_t *request, *reply;
  register mach_msg_return_t mr;

  if (max_size == 0)
    {
      option |= MACH_RCV_LARGE;
      max_size = 2 * __vm_page_size; /* Generic.  Good? XXX */
    }

  request = __alloca (max_size);
  reply = __alloca (max_size);

  while (1)
    {
    get_request:
      mr = __mach_msg (&request->Head, MACH_RCV_MSG|option,
		       0, max_size, rcv_name,
		       timeout, MACH_PORT_NULL);
      while (mr == MACH_MSG_SUCCESS)
	{
	  /* We have a request message.
	     Pass it to DEMUX for processing.  */

	  (void) (*demux) (&request->Head, &reply->Head);
	  assert (reply->Head.msgh_size <= max_size);

	  switch (reply->RetCode)
	    {
	    case KERN_SUCCESS:
	      /* Hunky dory.  */
	      break;

	    case MIG_NO_REPLY:
	      /* The server function wanted no reply sent.
		 Loop for another request.  */
	      goto get_request;

	    default:
	      /* Some error; destroy the request message to release any
		 port rights or VM it holds.  Don't destroy the reply port
		 right, so we can send an error message.  */
	      request->Head.msgh_remote_port = MACH_PORT_NULL;
	      __mach_msg_destroy (&request->Head);
	      break;
	    }

	  if (reply->Head.msgh_remote_port == MACH_PORT_NULL)
	    {
	      /* No reply port, so destroy the reply.  */
	      if (reply->Head.msgh_bits & MACH_MSGH_BITS_COMPLEX)
		__mach_msg_destroy (&reply->Head);
	      goto get_request;
	    }

	  /* Send the reply and the get next request.  */

	  {
	    /* Swap the request and reply buffers.  mach_msg will read the
	       reply message from the buffer we pass and write the new
	       request message to the same buffer.  */
	    void *tmp = request;
	    request = reply;
	    reply = tmp;
	  }

	  mr = __mach_msg (&request->Head,
			   MACH_SEND_MSG|MACH_RCV_MSG|option,
			   request->Head.msgh_size, max_size, rcv_name,
			   timeout, MACH_PORT_NULL);
	}

      /* A message error occurred.  */

      switch (mr)
	{
	case MACH_RCV_TOO_LARGE:
	  /* The request message is larger than MAX_SIZE, and has not
	     been dequeued.  The message header has the actual size of
	     the message.  We recurse here in hopes that the compiler
	     will optimize the tail-call and allocate some more stack
	     space instead of way too much.  */
	  return __mach_msg_server_timeout (demux, request->Head.msgh_size,
					    rcv_name, option, timeout);

	case MACH_SEND_INVALID_DEST:
	  /* The reply can't be delivered, so destroy it.  This error
	     indicates only that the requester went away, so we
	     continue and get the next request.  */
	  __mach_msg_destroy (&request->Head);
	  break;

	default:
	  /* Some other form of lossage; return to caller.  */
	  return mr;
	}
    }
}
weak_alias (__mach_msg_server_timeout, mach_msg_server_timeout)

mach_msg_return_t
__mach_msg_server (demux, max_size, rcv_name)
     boolean_t (*demux) (mach_msg_header_t *in, mach_msg_header_t *out);
     mach_msg_size_t max_size;
     mach_port_t rcv_name;
{
  return __mach_msg_server_timeout (demux, max_size, rcv_name,
				    MACH_MSG_OPTION_NONE,
				    MACH_MSG_TIMEOUT_NONE);
}
weak_alias (__mach_msg_server, mach_msg_server)
