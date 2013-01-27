#ifndef _AQ32_SERIAL_MAPPING_h
#define _AQ32_SERIAL_MAPPING_h

// set USE_USB_SERIAL if communication should be do via USB
// otherwise communication is done on Serial1
// see WProgram.h for implementation details

#if !defined (WirelessTelemetry)
  #define USE_USB_SERIAL
#endif

#endif
