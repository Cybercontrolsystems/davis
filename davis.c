/* DAVIS Interface program */

#include <stdio.h>	// for FILE
#include <stdlib.h>	// for timeval
#include <string.h>	// for strlen etc
#include <time.h>	// for ctime
#include <sys/types.h>	// for fd_set
// #include <sys/socket.h>
// #include <netinet/in.h>
#include <netdb.h>	// for sockaddr_in 
#include <fcntl.h>	// for O_RDWR
#include <termios.h>	// for termios
#include <unistd.h>		// for getopt
#ifdef linux
#include <errno.h>		// for Linux
#include <sys/uio.h>	// for struct iovec
#endif
#include "ccitt.h"		// for CRC

#include "../Common/common.h"

#define REVISION "$Revision: 1.7 $"
/* 1.0 Initial version created from Steca
	1.1 2008/06/12 Close and re-open serial port if connection is lost
	1.2 2009/04/13 Ability to grab a snapshot using LOOP command option.
	1.3 2009/09/11 Suppress repeated Davis Got .. messages
	1.4 2009/10/04 More tolerant of missing bytes.
    1.5 2010/04/21 When debug=1, data message shown. Also set interval (timeout) by command
    1.6 2010/08/15 Common Serial Framework plus message suppression
	1.7 2011/10/16 More tolerance of noise - use getbuf()
*/

static char* id="@(#)$Id: davis.c,v 1.7 2011/10/16 15:16:07 martin Exp $";

#define BAUD B19200
#define PROGNAME "Davis"
const char progname[]=PROGNAME;
#define LOGON "davis"
#define PORTNO 10010
#define LOGFILE "/tmp/davis.log"
#define DUMPFILE "/tmp/davis.dat"
#define SERIALNAME "/dev/ttyAM1"	/* although it MUST be supplied on command line */

#define REALTIMEINTERVAL 300
#define HILOWINTERVAL    3600
#define GRAPHINTERVAL    86400

// Severity levels.  FATAL terminates program
#define INFO	0
#define	WARN	1
#define	ERROR	2
#define	FATAL	3
// Socket retry params
#define NUMRETRIES 3
int numretries = NUMRETRIES;
#define RETRYDELAY	1000000	/* microseconds */
int retrydelay = RETRYDELAY;
// Serial retry params
#define SERIALNUMRETRIES 10
#define SERIALRETRYDELAY 1000000 /*microseconds = 1 sec */
#define WAITTIME 2      /*seconds*/
// Set to if(0) to disable debugging
#define ACK 0x06

/* SOCKET CLIENT */

/* Command line params: 
1 - device name
2 - device timeout. Default to 60 seconds
3 - optional 'nolog' to suppress writing to local logfile
*/

#ifndef linux
extern
#endif
int errno;  

// Procedures in this file
int processSocket(void);			// process server message
void usage(void);					// standard usage message
int getBuffer(char * serialbuf, int size);
int getbuf(int max, int tmout);	// get a buffer full of message
int wakeup(int commfd);				// wake up station. 1 = failure.
char * getversion(void);
int checkCRC(int size, char *msg);	// calc CRC over a buffer
time_t timeMod(time_t t);
void dumphex(int n, char * data);
void writepacket(unsigned char * data);	// Textual output for debug

/* GLOBALS */
FILE * logfp = NULL;
int sockfd[1] = {0};
int debug = 0;
int noserver = 0;		// prevents socket connection when set to 1
char * serialName = SERIALNAME;

// Common Serial Framework
#define BUFSIZE 4100	/* should be longer than max possible message from Davis */
int controllernum = -1;	//	only used in logon message
int commfd;
int tmout = REALTIMEINTERVAL;
struct data {	// The serial buffer
	int count;
	unsigned char buf[BUFSIZE];
//	int escape;		// Count the escapes in this message
//	int sentlength;
} data;

/********/
/* MAIN */
/********/
int main(int argc, char *argv[])
// arg1: serial device file
// arg2: optional timeout in seconds, default 60
// arg3: optional 'nolog' to carry on when filesystem full
{
	int nolog = 0;

    char buffer[256];
	int run = 1;		// set to 0 to stop main loop
	fd_set readfd; 
	int numfds;
	struct timeval timeout;
	int logerror = 0;
	int online = 1;		// used to prevent messages every minute in the event of disconnection
	int option, num, length; 
	time_t nextRealTime = 0;	// when to do next RealTime collection;
	struct iovec iov[3];
	int suppressMessages = 0;

	// Command line arguments
	
	// optind = -1;
	opterr = 0;
	while ((option = getopt(argc, argv, "dt:i:slVm:Z")) != -1) {
		switch (option) {
		case 's': noserver = 1; break;
		case 'l': nolog = 1; break;
		case '?': usage(); exit(1);
		case 't': 
		case 'i':tmout = atoi(optarg); break;
		case 'd': debug = 1; break;
		case 'm': suppressMessages = atoi(optarg); break;
		case 'V': printf("Version %s %s\n", getversion(), id); exit(0);
		case 'Z': decode("(b+#Gjv~z`mcx-@ndd`rxbwcl9Vox=,/\x10\x17\x0e\x11\x14\x15\x11\x0b\x1a" 
				 "\x19\x1a\x13\x0cx@NEEZ\\F\\ER\\\x19YTLDWQ'a-1d()#!/#(-9' >q\"!;=?51-??r"); exit(0);
		}
	}
	
	DEBUG printf("Debug on. optind %d argc %d\n", optind, argc);
	
	if (optind < argc) serialName = argv[optind];		// get serial device name: parameter 1
	optind++;
	if (optind < argc) controllernum = atoi(argv[optind]);	// get optional controller number: parameter 2
	
	if (!nolog) if ((logfp = fopen(LOGFILE, "a")) == NULL) logerror = errno;	
	
	// There is no point in logging the failure to open the logfile
	// to the logfile, and the socket is not yet open.

	sprintf(buffer, "STARTED %s on %s as %d timeout %d %s", argv[0], serialName, controllernum, tmout, nolog ? "nolog" : "");
	logmsg(INFO, buffer);
	
	openSockets(0, 1, LOGON,  REVISION, "", 0);
	
	// Open serial port
	if ((commfd = openSerial(serialName, BAUD, 0, CS8, 1)) < 0) {
		sprintf(buffer, "ERROR " PROGNAME " %d Failed to open %s: %s", controllernum, serialName, strerror(errno));
#ifdef DEBUGCOMMS
		logmsg(INFO, buffer);			// FIXME AFTER TEST
		printf("Using stdio\n");
		commfd = 0;		// use stdin
#else
		logmsg(FATAL, buffer);
#endif
	}

	// If we failed to open the logfile and were NOT called with nolog, warn server
	// Obviously don't use logmsg!
	if (logfp == NULL && nolog == 0) {
		sprintf(buffer, "event WARN " PROGNAME " %d could not open logfile %s: %s", controllernum, LOGFILE, strerror(logerror));
		sockSend(sockfd[0], buffer);
	}
		
	numfds = (sockfd[0] > commfd ? sockfd[0] : commfd) + 1;		// nfds parameter to select. One more than highest descriptor
	DEBUG fprintf(stderr,"Commfd = %d, numfds = %d ", commfd, numfds);

	// Main Loop
	FD_ZERO(&readfd); 
	iov[0].iov_base = &length;
	iov[0].iov_len = 2;
	nextRealTime = time(NULL);
	while(run) {
		int n;
		timeout.tv_sec = 10;
		timeout.tv_usec = 0;
		FD_SET(sockfd[0], &readfd);		// wait for socket input only.
		FD_SET(commfd, &readfd);
		timeout.tv_sec = nextRealTime - time(NULL);	// first time around, this is zero.
		n = select(numfds, &readfd, NULL, NULL, &timeout);	// select timed out.
		DEBUG fprintf(stderr, "timeout Select returned %d ", n);
		if (n == -1)
			DEBUG fprintf(stderr, "Error %s sockfd %d commfd %d numfds %d\n", strerror(errno), sockfd[0], commfd, numfds); 
		if (time(NULL) >= nextRealTime) {	// Get the next RealTime record every 60 seconds
			DEBUG 
				if (FD_ISSET(commfd, &readfd)) fprintf(stderr,"Commfd readable ... ");
			wakeup(commfd);
			sendSerial(commfd, "LOOP 1\n");
			FD_SET(commfd, &readfd);
			timeout.tv_sec = 10;	// up to 10 seconds for Davis response.
			if (n = select(numfds, &readfd, NULL, NULL, &timeout)) {	// select has data on commfd
				online = 1;
				if (n < 0) perror("davis commfd select");
				DEBUG fprintf(stderr, "commfd Select returned %d ", n);
				data.count = 0;
				getbuf(100, 2000);
				if (data.count != 100) {
					DEBUG fprintf(stderr, "Got %d instead of 99 - ignoring packet\n", data.count);
					continue;
				}
				
				if (data.buf[0] != ACK) {
					DEBUG fprintf(stderr, "Byte[0] is %02x not ACK - ignoring packet\n", data.buf[0]);
					continue;
				}
				if (checkCRC(99, data.buf + 1)) {
					DEBUG fprintf(stderr, "CRC failed\n");
					continue;
				}
				
				iov[1].iov_base = "davis realtime";
				iov[1].iov_len = 15; // includes trailing \0
				iov[2].iov_base = data.buf + 1;
				iov[2].iov_len = 97;	// don't send CRC
				length = htons(15 + 97);	// = 112
				if (sockfd[0]) 
					num = writev(sockfd[0], iov, 3);
				
				DEBUG fprintf(stderr, "Davis realtime: sent %d bytes\n" , num);
				DEBUG dumphex(99, data.buf+1);
				DEBUG writepacket(data.buf+1);
			} 
			else	// select timed out
			{
				if (online) {
					logmsg(WARN, "WARN " PROGNAME " no data for last period .. reopening port");
					online = 0;
					do {
						close(commfd);
						sleep(10);
						if ((commfd = openSerial(serialName, B19200, 0, CS8, 1)) < 0) {
							sprintf(buffer, "ERROR " PROGNAME " %d Failed to re-open %s: %s", controllernum, serialName, strerror(errno));
							logmsg(ERROR, buffer);
							sleep(150);
						}
						
					} while (commfd < 0);
				}
			}
			nextRealTime = timeMod(tmout);
			DEBUG fprintf(stderr, "Sleeping %zu ... \n", nextRealTime - time(NULL));
			// Wait until next period - but awaken if a socket message comes in
		}
		else sleep(10);		// To avoid race condition
		
		if ((noserver == 0) && FD_ISSET(sockfd[0], &readfd))
			run = processSocket();	// the server may request a shutdown by setting run to 0
	}
	logmsg(INFO,"INFO " PROGNAME " Shutdown requested");
	close(sockfd[0]);
	closeSerial(commfd);

	return 0;
}

/*********/
/* USAGE */
/*********/
void usage(void) {
	printf("Usage: davis [-t timeout] [-l] [-s] [-d] [-V] /dev/ttyname controllernum\n");
	printf("-l: no log  -s: no server  -d: debug on\n -V version\n");
	return;
}

/*****************/
/* PROCESSSOCKET */
/*****************/
int processSocket(){
	// Deal with commands from MCP.  Return to 0 to do a shutdown
	// Handle graph and hilow commands.
	short int msglen, numread;
	char buffer[128], buffer2[192];	// about 128 is good but rather excessive since longest message is 'truncate'
	char * cp = &buffer[0];
	int retries = NUMRETRIES;
	int num;
	
	if (read(sockfd[0], &msglen, 2) != 2) {
		logmsg(WARN, "WARN " PROGNAME " Failed to read length from socket");
		return 1;
	}
	msglen =  ntohs(msglen);
	while ((numread = read(sockfd[0], cp, msglen)) < msglen) {
		cp += numread;
		msglen -= numread;
		if (--retries == 0) {
			logmsg(WARN, "WARN " PROGNAME " Timed out reading from server");
			return 1;
		}
		usleep(RETRYDELAY);
	}
	cp[numread] = '\0';	// terminate the buffer 
	
	if (strcasecmp(buffer, "exit") == 0)
		return 0;	// Terminate program
	if (strcasecmp(buffer, "Ok") == 0)
		return 1;	// Just acknowledgement
	if (strcasecmp(buffer, "truncate") == 0) {
		if (logfp) {
			// ftruncate(logfp, 0L);
			// lseek(logfp, 0L, SEEK_SET);
			freopen(NULL, "w", logfp);
			logmsg(INFO, "INFO " PROGNAME " Truncated log file");
		} else
			logmsg(INFO, "INFO " PROGNAME " Log file not truncated as it is not open");
		return 1;
	}
	if (strcasecmp(buffer, "debug 0") == 0) {	// turn off debug
		debug = 0;
		return 1;
	}
	if (strcasecmp(buffer, "debug 1") == 0) {	// enable debugging
		debug = 1;
		return 1;
	}
	if (strcasecmp(buffer, "help") == 0 || *buffer == '?') {
		logmsg(INFO, "INFO: Available commands are exit; truncate; debug 0|1; interval; hilow; graph; loop");
		return 1;
	}
	if (strcasecmp(buffer, "hilow") == 0) {
		wakeup(commfd);
		sendSerial(commfd, "HILOWS\n");
		getbuf(438, 1000);
		DEBUG fprintf(stderr, "" PROGNAME " hilow: sent %d bytes\n" , num);
		dumphex(436, data.buf);
		logmsg(INFO, "INFO " PROGNAME " written file " DUMPFILE);
		return 1;
	}
	if (strcasecmp(buffer, "graph") == 0) {
		wakeup(commfd);
		sendSerial(commfd, "GETEE\n");
		getbuf(4098, 1000);	// include checksum
		DEBUG fprintf(stderr, "Davis graph: sent %d bytes\n" , num);
		dumphex(4098, data.buf);
		logmsg(INFO, "INFO " PROGNAME " written file " DUMPFILE);
		return 1;
	}
	if (strcasecmp(buffer, "loop") == 0) {
		wakeup(commfd);
		sendSerial(commfd, "LOOP 1\n");
		getbuf(99, 1000);
		dumphex(99, data.buf);
		logmsg(INFO, "INFO " PROGNAME " written file " DUMPFILE);
		return 1;
	}
	if (strncasecmp(buffer, "interval ", 9) == 0) {
		tmout = strtol(buffer+9, NULL, 0);
		if (tmout == 0) tmout = 60;
		sprintf(buffer, "INFO " PROGNAME " Interval set to %d seconds", tmout);
		logmsg(INFO, buffer);
		return 1;
	}
		
	strcpy(buffer2, "INFO " PROGNAME " Unknown message from server: ");
	strcat(buffer2, buffer);
	logmsg(INFO, buffer2);	// Risk of loop: sending unknown message straight back to server
	
	return 1;	
};

/**************/
/* GETVERSION */
/**************/
char *getversion(void) {
// return pointer to version part of REVISION macro
	static char version[10] = "";	// Room for xxxx.yyyy
	if (!strlen(version)) {
		strcpy(version, REVISION+11);
		version[strlen(version)-2] = '\0';
	}
return version;
}

/**************/
/* SENDSERIAL */
/**************/
int sendSerial(int fd, char *data) {
	// Send a zero-terminated string.  Return 1 for a logged failure
	int retries = SERIALNUMRETRIES;
	int written;
	size_t len = strlen(data);
#ifdef DEBUGCOMMS
	printf("Comms: %s", data);
	return 0;
#endif
	DEBUG fprintf(DEBUGFP, "Sending %zu bytes: %s", len, data);
	while ((written = write(fd, data, len)) < len) {
        fprintf(DEBUGFP, "Serial wrote %d bytes errno = %d", written, errno);
        perror("");
		if (--retries == 0) {
			logmsg(WARN, "WARN " PROGNAME " timed out writing to serial port");
			return 1;
		}
		DEBUG fprintf(DEBUGFP, "Pausing %d ... ", SERIALRETRYDELAY);
		usleep(SERIALRETRYDELAY);
	}
	return 0;       // ok
}

/**********/
/* GETBUF */
/**********/
int getbuf(int max, int tmout) {
	// Read up to max chars into supplied buf. Return number
	// of chars read or negative error code if applicable
	int ready, numtoread, now;
	fd_set readfd; 
	struct timeval timeout;
	FD_ZERO(&readfd);
	// data.escape = 0;
	// numread = 0;
	numtoread = max;
	DEBUG2 fprintf(stderr, "Getbuf entry %d count=%d ", max ,data.count);
	
	while(1) {
		FD_SET(commfd, &readfd);
		timeout.tv_sec = tmout / 1000;
		timeout.tv_usec = (tmout % 1000) * 1000;	 // 0.5sec
		ready = select(commfd + 1, &readfd, NULL, NULL, &timeout);
		DEBUG4 {
			gettimeofday(&timeout, NULL);
			fprintf(stderr, "%03ld.%03d ", timeout.tv_sec%100, timeout.tv_usec / 1000);
		}
		if (ready == 0) {
			DEBUG2 fprintf(stderr, "Gotbuf %d bytes ", data.count);
			return data.count;		// timed out - return what we've got
		}
		DEBUG4 fprintf(stderr, "Getbuf: before read1 ");
		now = read(commfd, data.buf + data.count, 1);
		DEBUG4 fprintf(stderr, "After read1\n");
		DEBUG3 fprintf(stderr, "0x%02x ", data.buf[data.count]);
		if (now < 0)
			return now;
		if (now == 0) {
			fprintf(stderr, "ERROR fd was ready but got no data\n");
			// VBUs / LAN  - can't use standard Reopenserial as device name hostname: port is not valid
			commfd = reopenSerial(commfd, serialName, BAUD, 0, CS8, 1);
			continue;
		}
		
		data.count += now;
		numtoread -= now;
		DEBUG3 fprintf(stderr, "[%d] ", data.count - now);
		if (numtoread == 0) return data.count;
		if (numtoread < 0) {	// CANT HAPPEN
			fprintf(stderr, "ERROR buffer overflow - increase max from %d (numtoread = %d numread = %d)\n", 
					max, numtoread, data.count);
			return data.count;
		}
	}
}

/*************/
/* GETBUFFER */   /* OBSOLETE */
/*************/
int getBuffer(char *buf, int size) {
// Read size bytes into buf from commfd. Validate ACK and CRC
// First byte read must be ACK - not counted in total size.

// TODO this will block forever until it gets enough data`.
// Will return after 100mSec pause in data 
	char * ptr = buf;
	char buffer[100];  // for messages
	int numread, num;
	int numtoread = size;
	static int errormode = 0;
	fd_set fdset;
	struct timeval timeout;
	sleep(1);
	if ((numread = read(commfd, buf, 1)) != 1) {	// commfd should be readable to get here
		sprintf(buffer, "WARN " PROGNAME " couldn't read initial ACK byte - only got %d bytes", numread);
		logmsg(WARN, buffer);
		return 1;
	}
	if (buf[0] != ACK) {	// it wasn't an ACK
		if (errormode == 0) {
			sprintf(buffer, "WARN " PROGNAME " got %02x instead of ACK", buf[0]);
			logmsg(WARN, buffer);
		} 
		errormode++;
		if (errormode % 100 == 0) {
			sprintf(buffer, "WARN " PROGNAME " %d comms errors", errormode);
			logmsg(WARN,buffer);
		}
		return 1;
	}
	if (errormode) {
		sprintf(buffer, "INFO " PROGNAME " had %d bad bytes", errormode);
		logmsg(INFO, buffer);
		errormode = 0;
	}
	FD_ZERO(&fdset);
	while (numtoread > 0) {		// 1.7 changed to > 0 instead of != 0
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;	// 100mS
		FD_SET(commfd, &fdset);
		num = select(commfd+1, &fdset, NULL, NULL, &timeout);
		if (num == 0) {
			DEBUG fprintf(stderr, "Timed out with %d read", numread);
			return 1;
		}
		if ((numread = read(commfd, ptr, numtoread)) < 0) {	// error
			perror("");
			return 1;
		}
		DEBUG fprintf(stderr, "Got %d out of %d\n", numread, numtoread);
		numtoread -= numread;
		ptr += numread;
		usleep(SERIALRETRYDELAY);
	}
#ifdef	DEBUGCOMMS
	fprintf(stderr, "CRC = %04x\n", checkCRC(size, buf));
	return 0;
#else
	return checkCRC(size, buf);
#endif
	
}

/**********/
/* WAKEUP */
/**********/
int wakeup(int commfd) { 
// try and wake up Davis. 0 is success.
// 
	int i;
	char buf[2];
	struct timeval timeout;
	fd_set fd;
	FD_ZERO(&fd);
	for (i=1; i < 3; i++) {
		timeout.tv_sec = 1;
		timeout.tv_usec = 500000;
		FD_SET(commfd, &fd);
		sendSerial(commfd, "\n");
		if (select(commfd+1, &fd, NULL, NULL, &timeout) == 1) {	// data ready
			if (read(commfd, buf, 2) == 2) return 0;
			else fprintf(stderr, "Error - only read 1 byte (0x%02x) in response to wakeup\n", buf[0]);
		}
	}
	return 1;
}

/**************/
/* STORMSTART */
/**************/
int stormstart(int v) {
// Convert davis format to days since 2000.
// M  M  M  M  D  D  D  D  D  Y  Y  Y  Y  Y  Y  Y
// 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
struct tm tm;
time_t t;
	tm.tm_mon = (v & 0xF000) >> 12 - 1;	// tm_mon starts at Jan = 0
	tm.tm_mday = (v & 0xf80) >> 6;
	tm.tm_year = (v & 0x07F) + 100;	// Davis count from 2000 not 1900
	t = (mktime(&tm) / 86400) - 10957;	// to get days since 2000.  Jan 1 1970 + 10957 days  = 1 Jan 2000
	DEBUG fprintf(stderr, "Stormstart - input data %04x\n", v);
	return t;
}
		
/************/
/* CHECKCRC */
/************/
int checkCRC(int size, char *msg)
{
        int i;
        int crc = 0;                                  /* zero checksum to start       */
        for( i = 0; i < size; i++) {
                crc = (crc_table[(crc >> 8) ^ *msg++] ^ (crc << 8)) & 0xFFFF; /* CCITT std */
        }
        return crc;                                    /* if zero, it passed */
} 

/***********/
/* TIMEMOD */
/***********/
time_t timeMod(time_t t) {
// Return a time in the future at modulus t;
// ie, if t = 3600 (1 hour) the time returned
// will be the next time on the hour.
//	time_t now = time(NULL);
	if (t == 0) t = 600;
	DEBUG fprintf(stderr,"TimeMod now = %zu delta = %zu result = %ld\n", time(NULL), t, (time(NULL) / t) * t +t);
	return (time(NULL) / t) * t + t;
}

/***********/
/* DUMPHEX */
/***********/
void dumphex(int n, char * data) {
// Dump N bytes from *data as hex to file /tmp/davis.dat
	int i;
	char buf[100];
	FILE * f;
	if ((f = fopen(DUMPFILE, "w")) == 0) {
		sprintf(buf, "WARN " PROGNAME " Failed to open " DUMPFILE);
		logmsg(WARN, buf);
		return;
		}
	for (i = 0; i < n; i++) {
		fprintf(f, "%02x ", data[i]);
	}
	fprintf(f, "\n");
	for (i = 0; i < n; i++) {
		if (i % 8 == 0) fprintf (f, "\n%03d: ", i);
		fprintf(f, "%02x(%03d) ", data[i], data[i]);
	}
	fprintf(f, "\n");
	fclose(f);
}

#define makeshort(lsb, msb)  ( lsb | (msb << 8))
#define makelong(lsb, b2, b3, msb) (lsb | (b2 << 8) | (b3 << 16) | (msb << 24))

char * mins2hhmm(int x) {
	static char res[8];
	if (x == 0xFFFF) res[0] = '\0';
	else
		sprintf(res, "%02d:%02d", x / 100, x % 100);
	return res;
}

/***************/
/* WRITEPACKET */
/***************/
void writepacket(unsigned char * data) {
	// Textual output
	char buf[300];
	char s1[6], s2[6];
	snprintf(buf, 300, "INFO " PROGNAME " Data %c%c%c0x%02x Hg%.2f Int %.1fF %u%% Ext %.1fF %u%% Wind %u "
			"Avg %u Dir %u Rain %.2f\" Irr %d Day %.2f\" Mon %.2f\" Yr %.2f\" Alrm %u "
			"Bat %.2f [%s-%s]", data[0], data[1], data[2], data[3],	// LOO + barotrend
			(float)(makeshort(data[7], data[8])) / 1000.0,	// Barometer
			(float)(makeshort(data[9], data[10])) / 10.0, 
			data[11], //Internal temp & humidity
			(float)(makeshort(data[12], data[13])) / 10.0, // External Temp
			data[33], //External humidity
			data[14], 
			data[15], 
			makeshort(data[16], data[17]),
			(float)(makeshort(data[41], data[42])) / 100.0, // RainRate
			 makeshort(data[44], data[45]),					// Radiation
			(float)(makeshort(data[50], data[51])) / 100.0, 
			(float)(makeshort(data[52], data[53])) / 100.0,  
			(float)(makeshort(data[54], data[55])) / 100.0,
			makelong(data[70], data[71], data[72], data[73]),
			(float)(makeshort(data[87], data[88]) * 300.0 / 51200.0),
			strcpy(s1, mins2hhmm(makeshort(data[91], data[92]))), 
			strcpy(s2, mins2hhmm(makeshort(data[93], data[94]))));
	DEBUG fprintf(stderr, "Length %zu bytes '%s'\n", strlen(buf), buf);
	logmsg(INFO, buf);
}

	
