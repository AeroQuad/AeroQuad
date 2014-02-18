/* CS3 start_c routine.
 *
 * Copyright (c) 2006, 2007 CodeSourcery Inc
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions. No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */

#include "cs3.h"

extern void __libc_init_array (void);

extern int main (int, char **, char **);

extern void exit (int) __attribute__ ((noreturn, weak));

void  __attribute ((noreturn))
__cs3_start_c (void)
{
  unsigned regions = __cs3_region_num;
  const struct __cs3_region *rptr = __cs3_regions;
  int exit_code;

  /* Initialize memory */
  for (regions = __cs3_region_num, rptr = __cs3_regions; regions--; rptr++)
    {
      long long *src = (long long *)rptr->init;
      long long *dst = (long long *)rptr->data;
      unsigned limit = rptr->init_size;
      unsigned count;

      if (src != dst)
	for (count = 0; count != limit; count += sizeof (long long))
	  *dst++ = *src++;
      else
	dst = (long long *)((char *)dst + limit);
      limit = rptr->zero_size;
      for (count = 0; count != limit; count += sizeof (long long))
	*dst++ = 0;
    }

  /* Run initializers.  */
  __libc_init_array ();

  exit_code = main (0, NULL, NULL);
  if (exit)
    exit (exit_code);
  /* If exit is NULL, make sure we don't return. */
  for (;;)
    continue;
}
