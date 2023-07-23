#ifndef __PIONEERSDKESP32__
#define __PIONEERSDKESP32__

typedef struct {
	int port;
	int system_id;
	const char * ip;
} Pioneer;

void init_pioneer(Pioneer * pio);

#endif // __PIONEERSDKESP32__