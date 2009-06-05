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

/*---------------------------------------------------------------------------
 * The code in this file originated in:
 *    http://www.arduino.cc/playground/Main/PcInt
 *
 * This is an extension to the interrupt support for arduino.
 * add pin change interrupts to the external interrupts, giving a way
 * for users to have interrupts drive off of any pin.
 * Refer to avr-gcc header files, arduino source and atmega datasheet.
 *
 *
 * Theory: all IO pins on Atmega168/328 are covered by Pin Change Interrupts.
 * The PCINT corresponding to the pin must be enabled and masked, and
 * an ISR routine provided.  Since PCINTs are per port, not per pin, the ISR
 * must use some logic to actually implement a per-pin interrupt service.
 *
 *
 * Pin to interrupt map:
 * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
 *---------------------------------------------------------------------------*/


#include <pins_arduino.h>
#include "pcint.h"


//=============================================================================
//
// Static Variables
//
//=============================================================================

// Array of PCMSKn register address per port

static volatile uint8_t *const  port_to_p_pcmask[] =
{
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

// Handler function pointers, per pin

static p_pcintHandler           pcint_handler[24] = {NULL};

// User data passed to handler function, per pin

static void                    *pcint_usr[24] = {NULL};

// Last value read from input register

static volatile uint8_t         pcint_last[3];

#ifdef PCINT_DEBUG
static uint8_t                  num_changed_while_processing = 0;
#endif


//=============================== pcint_attach() ==============================
//
// attach an interrupt to a specific pin using pin change interrupts.
//
//=============================================================================

void pcint_attach(uint8_t   pin,    // Pin to attach
                  void    (*handler)(void *, uint8_t, uint32_t),
                                    // Pointer to handler function
                  void     *usr)    // User data to pass to handler

{
  uint8_t           bit;        // Bit mask for bit in port
  uint8_t           port;       // 8-bit AVR port (0..2)
  uint8_t           slot;       // 0..23
  volatile uint8_t *p_pcmask;   // Pointer to Pin Change Mask register


  bit  = digitalPinToBitMask(pin);
  port = digitalPinToPort(pin);

  // map pin to PCIR register

  if (port == NOT_A_PORT)
  {
    return;
  } 

  else
  {
    port -= 2;
    p_pcmask = port_to_p_pcmask[port];
  }

  slot = port * 8 + (pin % 8);
  pcint_handler[slot] = handler;
  pcint_usr[slot] = usr;

  // set the mask

  *p_pcmask |= bit;

  // enable the interrupt

  PCICR |= 0x01 << port;
}


//=============================== pcint_detach() ==============================
//
// dettach an interrupt from a specific pin using pin change interrupts.
//
//=============================================================================

void pcint_detach(uint8_t pin)

{
  uint8_t           bit;        // Bit mask for bit in port
  uint8_t           port;       // 8-bit AVR port (0..2)
  volatile uint8_t *p_pcmask;   // Pointer to Pin Change Mask register


  bit = digitalPinToBitMask(pin);
  port = digitalPinToPort(pin);
  
  // map pin to PCIR register
  
  if (port == NOT_A_PORT)
  {
    return;
  } 

  else
  {
    port -= 2;
    p_pcmask = port_to_p_pcmask[port];
  }

  // disable the mask.

  *p_pcmask &= ~bit;

  // if that's the last one, disable the interrupt.

  if (*p_pcmask == 0)
  {
    PCICR &= ~(0x01 << port);
  }
}


//=========================== pcint_isr_common() ==============================
//
// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
//
//=============================================================================

static void pcint_isr_common(uint8_t port)
{
  uint8_t           bit;
  volatile uint8_t *p_reg;              // Pointer to input register
  uint8_t           curr;               // Current value read from input
  uint8_t           last;               // Last value read from input
  uint8_t           pin_changed_mask;   // pcint pins that have changed.
  uint8_t           slot;               // Arduino pin (0..23)
  uint32_t          curr_time;


  curr_time = micros();

  // get the pin states for the indicated port and set pin_changed_mask to the pins that
  // have changed. screen out non pcint pins.

  p_reg = portInputRegister(port + 2);

  curr = *p_reg;
  pin_changed_mask = (curr ^ pcint_last[port]) & *port_to_p_pcmask[port];
  last = curr;

  while (pin_changed_mask != 0)
  {
    slot = port * 8;
#if 1
    for (bit = 0x01; bit != 0; bit <<= 1)
    {
      if ((bit & pin_changed_mask) && (pcint_handler[slot] != NULL))
      {
        pcint_handler[slot](pcint_usr[slot], curr & pin_changed_mask, curr_time);
      }

      slot++;
    }
#endif
#if 0   // This code should be better but for some reason it doesn't work :(
    while (pin_changed_mask != 0)
    {
      uint8_t temp_mask = pin_changed_mask & 0x01;
      
    
      if (temp_mask && (pcint_handler[slot] != NULL))
      {
        pcint_handler[slot](pcint_usr[slot], temp_mask, curr_time);
      }

      pin_changed_mask >>= 1;
      slot++;
    }
#endif
    // Inputs may have changed while we were processing
    
    curr = *p_reg;
    pin_changed_mask = (curr ^ last) & *port_to_p_pcmask[port];
    last = curr;
    
    if (pin_changed_mask != 0)
      num_changed_while_processing++;
  }

  pcint_last[port] = last;
}


//=============================================================================
//
// ISRs for PC interrupts on 3 port registers
//
//=============================================================================

ISR(PCINT0_vect)

{
  pcint_isr_common(0);
}


ISR(PCINT1_vect)

{
  pcint_isr_common(1);
}


ISR(PCINT2_vect)

{
  pcint_isr_common(2);
}


#ifdef PCINT_DEBUG
//=============================== pcint_print_stats() =========================
//
// Print some internal statistics (for debug)
//
//=============================================================================

void pcint_print_stats()

{
  Serial.print(num_changed_while_processing, DEC);
}
#endif
