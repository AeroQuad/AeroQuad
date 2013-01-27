/*
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

	.text
	.code 16
	.thumb_func

	.globl _start
	.type _start, %function
_start:
	.fnstart
	ldr	r1,=__cs3_stack
	mov	sp,r1
	ldr	r1,=__cs3_start_c
	bx	r1
	.pool
	.cantunwind
	.fnend
