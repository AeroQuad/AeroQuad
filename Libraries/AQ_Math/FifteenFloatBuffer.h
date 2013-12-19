// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __FifteenFloatBuffer_H__
#define __FifteenFloatBuffer_H__

#include <stdint.h>

#define BUFFER_SIZE 15

class FifteenFloatBuffer 
{
private:
    uint8_t     _num_items;             
    uint8_t     _head;                  
    float       _buff[BUFFER_SIZE];     

public:
    FifteenFloatBuffer()
	{
		_num_items = 0;
		clear();
	}

    void clear() 
	{
		_num_items = 0;
		_head = 0;
	}
	
    void add( float item )
	{
		uint8_t tail = _head + _num_items;
		if( tail >= BUFFER_SIZE ) 
		{
			tail -= BUFFER_SIZE;
		}

		_buff[tail] = item;

		if( _num_items < BUFFER_SIZE ) 
		{
			_num_items++;
		}
		else
		{
			_head++;
			if( _head >= BUFFER_SIZE ) 
			{
				_head = 0;
			}
		}
	}
	
    float get()
	{
		float result;

		if( _num_items == 0 ) 
		{
			return 0;
		}

		result = _buff[_head];

		_head++;
		if( _head >= BUFFER_SIZE )
		{
			_head = 0;
		}

		_num_items--;

		return result;
	}
	
    float peek(uint8_t position = 0) const
	{
		uint8_t j = _head+position;

		if( position >= _num_items ) 
		{
			return 0;
		}

		if( j >= BUFFER_SIZE ) 
		{
			j -= BUFFER_SIZE;
		}

		return _buff[j];
	}
	
    uint8_t num_items() const 
	{ 
		return _num_items; 
	}
};


#endif  // __FifteenFloatBuffer_H__
