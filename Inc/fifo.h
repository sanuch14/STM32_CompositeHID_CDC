#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>

typedef struct FIFOBUF{
	char		*Buffer;
	uint8_t		SIZE;
	uint8_t		putIndex;
	uint8_t		getIndex;
	uint8_t		isEmpty;
	void (*putElement) (char element);
	char (*getElement) (void);
}FIFOBUF;

FIFOBUF *createFIFO(uint8_t size);

#endif /* FIFO_H */

