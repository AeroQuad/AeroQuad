/*
  AeroQuad v1.2 - June 2009
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

#include "pcint.h"

//=============================================================================
//
// Local Definitions
//
//=============================================================================

// Receiver Tick defines the measuement units of raw receiver data.  Currently
// this is 1 uSec but it may change for the sake of optimization.

#define RECEIVER_TICK 1   // uSec

// Nominal minimum, middle and maximum values of raw receiver data

#define RECEIVER_NOM_MIN (1000 / RECEIVER_TICK)
#define RECEIVER_NOM_MID (1500 / RECEIVER_TICK)
#define RECEIVER_NOM_MAX (2000 / RECEIVER_TICK)

// Absolute minimum and maximum values of raw receiver data.
// Beyond them the receiver is considered to be non-functional.

#define RECEIVER_LOW_MIN (16000 / RECEIVER_TICK)
#define RECEIVER_LOW_MAX (24000 / RECEIVER_TICK)
#define RECEIVER_HIGH_MIN (1000 / RECEIVER_TICK)
#define RECEIVER_HIGH_MAX (2000 / RECEIVER_TICK)

// Maximum number of consecutive errors before we decalre a fault.
// Experience has shown that from time to time we get too-short or too-long
// pulses from the reciver.  This does not seem to be a s/w bug but either a
// receiver mis-behavior of a h/w problem.  The current solution is to ignore
// illegal-width pulses, if their consecutive number is small.

#define RECEIVER_MAX_ERRORS 4


//=============================================================================
//
// Static Variables
//
//=============================================================================

// Channel data

typedef struct
{
  boolean  last_was_high;   // true if last time channel input was high
  uint8_t  error_count;     // Counts error to detect receiver faults
  uint32_t last_ticks;      // Time (number of ticks) of last pin change
  uint32_t ticks_high;      // Pulse width (number of ticks) last measured
} ReceiverChData;

static ReceiverChData ch_data[LASTCHANNEL];


//=============================================================================
//
// Static Functions
//
//=============================================================================

//======================== receiver_pci_handler() =============================
//
// Handles PCI for receiver pins
//
//=============================================================================

void receiver_pci_handler(void     *p_usr,
                          uint8_t   masked_in,
                          uint32_t  curr_ticks)

{
  ReceiverChData *p_ch_data;
  uint32_t        ticks_diff;   // Time diff (# of ticks) from last input change
  boolean         error_flag;   // Flags a receiver error


  p_ch_data = (ReceiverChData *)p_usr;
  
  if (masked_in == 0)
  {
    // high-to-low transition

    if (! p_ch_data->last_was_high)
    {
      // Sanity check failed, last time was low
      
      error_flag = true;
    }

    else
    {
      ticks_diff = curr_ticks - p_ch_data->last_ticks;
      
      if ((ticks_diff < RECEIVER_HIGH_MIN) || 
          (ticks_diff > RECEIVER_HIGH_MAX))
        error_flag = true;

      else
      {
        p_ch_data->ticks_high = ticks_diff;
        p_ch_data->error_count = 0;   // Only successful high resets the counter
        error_flag = false;
      }
    }
    
    p_ch_data->last_was_high = false;
    p_ch_data->last_ticks = curr_ticks;
  }

  else
  {
    // low-to-high transition

    if (p_ch_data->last_was_high)
    {
      // Sanity check failed, last time was high
      
      error_flag = true;
    }

    else
    {

      ticks_diff = curr_ticks - p_ch_data->last_ticks;

      error_flag = ((ticks_diff < RECEIVER_LOW_MIN) || 
                    (ticks_diff > RECEIVER_LOW_MAX));
    }
    
    p_ch_data->last_was_high = true;
    p_ch_data->last_ticks = curr_ticks;
  }

  if (error_flag)
  {
    if (p_ch_data->error_count < RECEIVER_MAX_ERRORS)
      p_ch_data->error_count++;
  }
}


//=============================================================================
//
// Public Functions
//
//=============================================================================

//============================ configureReceiver() ================================
//
// Initialize the receiver module
// Should be called on system initalization
//
//=============================================================================

boolean configureReceiver(void) {
  // Ret: true if OK, false if failed
  uint8_t ch;
  
 
  for (ch = ROLL; ch < LASTCHANNEL; ch++)
  {
    ch_data[ch].last_was_high = false;
    ch_data[ch].error_count = RECEIVER_MAX_ERRORS;   // Error until proven otherwise
    ch_data[ch].last_ticks = 0;
    ch_data[ch].ticks_high = 0;
    // ch_data[ch].ticks_low = 0;
  }
  
  pinMode(THROTTLEPIN, INPUT);
  pinMode(ROLLPIN, INPUT);
  pinMode(PITCHPIN, INPUT);
  pinMode(YAWPIN, INPUT);
  pinMode(MODE, INPUT);
  pinMode(AUXPIN, INPUT);

  pcint_attach(THROTTLEPIN, receiver_pci_handler, &ch_data[THROTTLE]);
  pcint_attach(ROLLPIN, receiver_pci_handler, &ch_data[ROLL]);
  pcint_attach(PITCHPIN, receiver_pci_handler, &ch_data[PITCH]);
  pcint_attach(YAWPIN, receiver_pci_handler, &ch_data[YAW]);
  pcint_attach(MODEPIN, receiver_pci_handler, &ch_data[MODE]);
  pcint_attach(AUXPIN, receiver_pci_handler, &ch_data[AUX]);

  return true;
}


//======================== receiver_get_status() ==============================
//
// Get the current status of the receiver
//
//=============================================================================

boolean statusReceiver(void) {
  // Ret: true if OK, false if not 
  boolean status = true;
  uint8_t ch;


  for (ch = ROLL; ch < LASTCHANNEL; ch++)
  {
    if (ch_data[ch].error_count >= RECEIVER_MAX_ERRORS)
       status = false;
  }

  return status;
}


//====================== receiver_get_current_raw() ===========================
//
// Get the current raw receiver data (that has been read before from the h/w)
//
//=============================================================================

uint16_t readReceiver(uint8_t ch) {
  // In:  channel
  // Ret: raw data, in units of  RECEIVER_TICK
  uint16_t data;
  uint8_t  old_sreg;


  // Save the interrupt status and disable interrupts.
  // This is required to assure consistent reading of the data.
  
  old_sreg = SREG;
  cli();	

  data = ch_data[ch].ticks_high;

  // Restore the interrupt status
  
  SREG = old_sreg;

  return data;
}


//========================== receiver_print_stats() ===========================
//
// Print some statistics (for debug)
//
//=============================================================================

void receiver_print_stats(void) {
  uint8_t ch;

  
  if (statusReceiver())
    Serial.print("GOOD\t");
  else
    Serial.print("BAD\t");
  
  for (ch = ROLL; ch < LASTCHANNEL; ch++)
  { 
    Serial.print(readReceiver(ch), DEC);
    Serial.print("\t");
  }
  
  pcint_print_stats();
  Serial.println();
}

