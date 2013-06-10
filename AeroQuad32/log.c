///////////////////////////////////////////////////////////////////////////////
//#define DEBUG_LOGGING

#include <ff.h>
#include <diskio.h>

///////////////////////////////////////////////////////////////////////////////

static uint16_t log_sd_card_available = 0;

static FATFS log_FATFS_Obj;

static FIL log_hfile;

#define LOGBUFFERSIZE 0x10000
#define LOGPAGESIZE   0x04000

static unsigned char log_buffer[LOGBUFFERSIZE];
static unsigned char *log_wp;
static unsigned char *log_rp;


int log_filecounter = 0;
unsigned int log_bytes_written;
static int log_init_disk_flag;
static int log_end_flag;
static unsigned char *log_end_wp;
static unsigned char *log_end_rp;
char log_filename[20];

int logWrite(char *line, int len);


///////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdarg.h>
void cliPrintF(const char * fmt, ...)
{
#ifdef DEBUG_LOGGING
	char buf[256];

	va_list vlist;
	va_start (vlist, fmt);

	vsnprintf (buf, sizeof(buf)-1, fmt, vlist);
	Serial.print(buf);
	va_end (vlist);
#endif
}

void logInitBuffer(void)
{
	if(log_end_flag) {
		unsigned char *new_log_rp = log_rp + LOGPAGESIZE;
		if(new_log_rp >= log_buffer+LOGBUFFERSIZE) {
			new_log_rp = log_buffer;
		}
		log_rp = new_log_rp;
		log_wp = new_log_rp;
		cliPrintF("initbuffer log_end_flag set, rp: %d\r\n", log_rp - log_buffer);
	} else {
		log_rp = log_buffer;
		log_wp = log_buffer;
		cliPrintF("initbuffer log_end_flag not set, rp: %d\r\n", log_rp - log_buffer);
	}
}

void logInitDisk(void)
{
	int result = disk_initialize(0);

    if (result == 0)
    {
        f_mount(0, &log_FATFS_Obj);

        {
            FILINFO filetest;

            do {
                sprintf(log_filename, "0:log%05u.csv", log_filecounter++);
            }
            while (f_stat(log_filename, &filetest) == FR_OK);
        }

        int result = f_open(&log_hfile, log_filename, FA_CREATE_NEW | FA_WRITE);

        if (result != 0)
        {
       	    cliPrintF("SD failed at f_open, filename '%s'\r\n", log_filename);

            log_sd_card_available = 0;
            return;
        }

        result = f_sync(&log_hfile);

        if (result != 0)
        {
       	    cliPrintF("SD failed at f_sync\r\n");

            log_sd_card_available = 0;
            return;
        }

        result = f_lseek(&log_hfile, log_hfile.fsize);

        if (result != 0)
        {
       	    cliPrintF("SD failed at f_lseek");

            log_sd_card_available = 0;
            return;
        }

        cliPrintF("SD using filename: %s\r\n", log_filename);

        log_sd_card_available = 1;
    } else {
    	cliPrintF("SD disk_initialize failed\r\n");
    }

}

///////////////////////////////////////////////////////////////////////////////

void writeToFile(const char *fname, uint8_t *buffer, uint32_t length)
{
    if (log_sd_card_available == 0)
    {
        return;
    }

    static FIL file2;

    char filename[200];
    sprintf(filename, "0:%s", fname);
    int result = f_open(&file2, filename, FA_CREATE_ALWAYS | FA_WRITE);

    if (result != 0)
    {
   	    cliPrintF("SD failed at f_open:write\r\n");

        log_sd_card_available = 0;
        return;
    }

    unsigned int bw = 0;

    result = f_write(&file2, buffer, length, &bw);

    if (result != 0)
    {
   	    cliPrintF("SD failed at f_write:write\r\n");

        log_sd_card_available = 0;
        return;
    }

    if (bw != length)
    {
   	    cliPrintF("SD failed due to length mismatch:write\r\n");

        log_sd_card_available = 0;
        return;
    }

    result = f_close(&file2);

    if (result != 0)
    {
   	    cliPrintF("SD failed at f_close:write\r\n");

        log_sd_card_available = 0;
        return;
    }
}

///////////////////////////////////////////////////////////////////////////////

void logPrintF(const char *text, ...)
{
    char tmp[500];
    va_list args;
    va_start(args, text);
    vsnprintf(tmp, sizeof(tmp), text, args);
    va_end(args);

    if (log_init_disk_flag == 0 && log_sd_card_available == 0)
    {
        return;
    }

    char line[500];

    uint32_t mmillis = millis();
    uint32_t seconds = mmillis / 1000;
    uint32_t fract   = mmillis - (seconds * 1000);

    snprintf(line, sizeof(line), "%05lu.%03lu, %s", seconds, fract, tmp);

    logWrite(line, strlen(line));
}

///////////////////////////////////////////////////////////////////////////////


void logInitWait()
{
	logInitBuffer();
	logInitDisk();
	log_init_disk_flag = 0;
}

void logInit()
{
	logInitBuffer();
	log_init_disk_flag = 1;
}

void logEnd()
{
	if(log_sd_card_available) {
		log_end_rp = log_rp;
		log_end_wp = log_wp;
		log_end_flag = 1;
		cliPrintF("log_end log_sd_card_available %d, rp: %d, wp: %d\r\n", log_sd_card_available, log_end_rp - log_buffer, log_end_wp - log_buffer);
	} else {
		cliPrintF("log_end log_sd_card_available %d, nothing to do\r\n", log_sd_card_available);
	}
}

void logSync(void)
{
	logEnd();
}

///////////////////////////////////////////////////////////////////////////////

/*
 * #define LOGBUFFERSIZE 0x10000
#define LOGPAGESIZE   0x04000

static unsigned char log_buffer[LOGBUFFERSIZE];
static unsigned char *log_wp;
static unsigned char *log_rp;
 *
 */


int logWrite(char *line, int len)
{
	int inBuffer = (log_wp - log_rp + LOGBUFFERSIZE) % LOGBUFFERSIZE;
	if(LOGBUFFERSIZE-inBuffer > len) {
		if(log_wp-log_buffer+len < LOGBUFFERSIZE) {
			// enough space at end of buffer
			memcpy(log_wp, line, len);
			log_wp += len;
		} else {
			// copy in two parts
			int len1 =  LOGBUFFERSIZE - (log_wp-log_buffer);
			int len2 = len  - len1;
			memcpy(log_wp, line, len1);
			memcpy(log_buffer, line+len1, len2);
			log_wp = log_buffer + len2;
		}
		log_bytes_written += len;
		return 0;
	} else {
		cliPrintF("SD buffer full:log_write\r\n");
		return 1;
	}
}

unsigned int logGetBytesWritten()
{
	return log_bytes_written;
}

char *logGetFilename()
{
	return log_filename;
}

int logFlush(unsigned char *&rp, unsigned char *wp)
{
	int inBuffer = (wp - rp + LOGBUFFERSIZE) % LOGBUFFERSIZE;
	unsigned int bw = 0;
	if(inBuffer > LOGPAGESIZE) {
		unsigned int result = f_write(&log_hfile, rp, LOGPAGESIZE, &bw);
		unsigned char *new_log_rp = rp + LOGPAGESIZE;
		if(new_log_rp >= log_buffer+LOGBUFFERSIZE) {
			new_log_rp = log_buffer;
		}
		rp = new_log_rp;

		inBuffer -= LOGPAGESIZE;

		if (result != 0)
		{
   	    	cliPrintF("SD failed at f_write:logWorker\r\n");

	    	log_sd_card_available = 0;
	    	return 0;
		}

		if (bw != LOGPAGESIZE)
		{
			cliPrintF("SD failed due to length mismatch:logWorker\r\n");

			log_sd_card_available = 0;
			return 0;
		}
	}

	return inBuffer;
}

void logWorker()
{
	if(log_sd_card_available) {
		if(log_end_flag) {
			cliPrintF("log_end_flag set, rp: %d, wp: %d\r\n", log_end_rp - log_buffer, log_end_wp - log_buffer);

			int inBuffer = logFlush(log_end_rp, log_end_wp);
			if(inBuffer > 0) {
				cliPrintF("log_end_flag set, rp: %d, inBuffer: %d\r\n", log_end_rp - log_buffer, inBuffer);
				unsigned int bw;
				f_write(&log_hfile, log_end_rp, inBuffer, &bw);
			}
			f_close(&log_hfile);
			log_end_flag = 0;
			if(!log_init_disk_flag) {
				log_sd_card_available = 0;
			}
		} else {
			logFlush(log_rp, log_wp);
		}
	}


	if(log_init_disk_flag) {
		logInitDisk();
		log_init_disk_flag = 0;
	}
}

