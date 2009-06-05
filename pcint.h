/*
  AeroQuad v1.1 - May 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
  
  Interrupt based method for reading receiver inputs written by Dror Caspi
  http://www.rcgroups.com/forums/showpost.php?p=12356667&postcount=1639
  Copyright (c) 2009 Dror Caspi.  All rights reserved.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef __PCINT_H__
#define __PCINT_H__

#define PCINT_DEBUG

// Pin Change Interrupt handler function type

typedef void (*p_pcintHandler)(void *,     // User data
                               uint8_t,    // 0 if changed from 1 to 0
                               uint32_t);  // Time in usec


//=============================== pcint_attach() ==============================
//
// attach an interrupt to a specific pin using pin change interrupts.
//
//=============================================================================

void pcint_attach(uint8_t   pin,    // Pin to attach
                   void    (*handler)(void *, uint8_t, uint32_t),                                     // Pointer to handler function
                   void     *usr);   // User data to pass to handler

//=============================== pcint_detach() ==============================
//
// dettach an interrupt from a specific pin using pin change interrupts.
//
//=============================================================================

void pcint_detach(uint8_t pin);


#ifdef PCINT_DEBUG
//=============================== pcint_print_stats() =========================
//
// Print some internal statistics (for debug)
//
//=============================================================================

void pcint_print_stats();
#endif

#endif
