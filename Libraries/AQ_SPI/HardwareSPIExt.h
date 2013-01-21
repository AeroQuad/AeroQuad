#ifndef _AEROQUAD_SPI_HARDWARESPIEXT_H_
#define _AEROQUAD_SPI_HARDWARESPIEXT_H_

#if defined(AeroQuadSTM32)

// helper class to extend the maple HardwareSPI class
// used by the MPU6000 library

#include <HardwareSPI.h>

#define SPI_READ_FLAG  0x80
#define SPI_MULTI_FLAG 0x40
#define SetPin digitalWrite

class HardwareSPIExt : public HardwareSPI {
public:
	HardwareSPIExt(uint32 spiPortNumber) : HardwareSPI(spiPortNumber) {
		SetCS(nssPin());
		fSpiMultiFlag = 0;
	}

	void SetCS(int aCS)
	{
		fCS = aCS;
	}

	void SetMultiFlag() {
		fSpiMultiFlag = SPI_MULTI_FLAG;
	}

	void begin(SPIFrequency frequency, uint32 bitOrder, uint32 mode)
	{
		SetPin(fCS, 1);
		pinMode(fCS, OUTPUT);

		HardwareSPI::begin(frequency, bitOrder, mode);
	}

	void Read(int addr, unsigned char *data, int dataLen)
	{
		SetPin(fCS, 0);
		transfer(addr | SPI_READ_FLAG | fSpiMultiFlag);
		while(dataLen-- > 0) {
			*data++ = transfer(0);
		}
		SetPin(fCS, 1);
	}

	unsigned char Read(int addr)
	{
		unsigned char data;
		Read(addr, &data, 1);

		return data;
	}

	void Write(int addr, unsigned char *data, int dataLen)
	{
		SetPin(fCS, 0);
		transfer(addr | fSpiMultiFlag);
		while(dataLen-- > 0) {
			transfer(*data++);
		}
		SetPin(fCS, 1);
	}

	void Write(int addr, unsigned char data)
	{
		Write(addr, &data, 1);
	}


private:
	int fCS;
	unsigned char fSpiMultiFlag;
};

#endif

#endif
