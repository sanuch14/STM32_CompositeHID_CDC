#include <stdlib.h>

#include "fifo.h"

FIFOBUF fifoBuffer;

void putEl(char el){
	if(!fifoBuffer.isEmpty){
		if(fifoBuffer.putIndex==fifoBuffer.getIndex){
			fifoBuffer.getIndex++;
			if(fifoBuffer.getIndex==fifoBuffer.SIZE)
				fifoBuffer.getIndex=0;
		}
	}
	fifoBuffer.Buffer[fifoBuffer.putIndex++]=el;
	if(fifoBuffer.putIndex==fifoBuffer.SIZE)
		fifoBuffer.putIndex=0;
	fifoBuffer.isEmpty=0;
}

char getEl(){
	if(fifoBuffer.isEmpty)
		return '\0';
	char el = fifoBuffer.Buffer[fifoBuffer.getIndex++];
	if(fifoBuffer.getIndex==fifoBuffer.SIZE)
		fifoBuffer.getIndex=0;
	if(fifoBuffer.getIndex==fifoBuffer.putIndex)
		fifoBuffer.isEmpty=1;
	return el;
}

FIFOBUF *createFIFO(uint8_t size){
	char *a = (char*)malloc(size*sizeof(char));
	
	fifoBuffer.Buffer=a;
	fifoBuffer.SIZE=size;
	fifoBuffer.getElement=&getEl;
	fifoBuffer.getIndex=0;
	fifoBuffer.isEmpty=1;
	fifoBuffer.putElement=&putEl;
	fifoBuffer.putIndex=0;
	
	return &fifoBuffer;
}
