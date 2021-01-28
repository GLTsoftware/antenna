#include <math.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <resource.h>
#include <errno.h>
/* If this is put ahead of math.h and sys/types.h, it hides some definitions */
#define _POSIX_SOURCE 1
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <strings.h>
#include <malloc.h>
#include "rm.h"

int antlist[RM_ARRAY_SIZE],rm_status;
int hex = 0;
char type;
#define MAXDIMS 10
int baseSize, ndims;
int dimensions[MAXDIMS] = {1,1,1,1,1,1,1,1,1,1};
void *arg;

/* rmvalue.c */
void ParseName(char *name);
void PrintValue(void);

int main(int argc, char *argv[]) {
	int i, size;

	if(argc == 3 && strncmp(argv[1], "-x", 2) == 0) {
	    hex++;
	    argv++;
	} else if(argc != 2) {
	    fprintf(stderr,"Usage: rmvalue variable name from rm_allocation\n");
	    exit(1);
	}
	rm_status=rm_open(antlist);
	if(rm_status != RM_SUCCESS) {
	    rm_error_message(rm_status,"rm_open()");
	    fprintf(stderr, "I can't do anything without reflective memory\n");
	    exit(1);
	}
	ParseName(argv[1]);
	fprintf(stderr, "Type %c, baseSize %d  ", type,
		baseSize);
	size = baseSize;
	if(ndims > 0) {
	    fprintf(stderr, "%d dimension(s) (", ndims);
	    for(i = ndims - 1; i >= 0; i--) {
		fprintf(stderr, " %d ", dimensions[i]);
		size *= dimensions[i];
	    }
	    putc(')', stderr);
	}
	putc('\n', stderr);
	arg = malloc(size);
	rm_status=rm_read(RM_ANT_0,argv[1], arg);
	if(rm_status != RM_SUCCESS) {
	    rm_error_message(rm_status,"rm_read()");
	}
	while(dimensions[1]-- > 0) {
	    for(i = 0; i < dimensions[0]; i++) {
		PrintValue();
	    }
	    putchar('\n');
	}
	putchar('\n');
	return(0);
}

void ParseName(char *name) {
	char *end, *cp;
	
	if(strncmp(name, "RM_", 3) != 0) {
	    fprintf(stderr, "%s is not a reflective memory variable\n", name);
	    exit(1);
	}
	for(cp = name; *cp; cp++) ;
	end = --cp;
	for(; cp[-1] != '_'; cp--) ;
	type = *cp;
	switch(type) {
	case 'B':
	    baseSize = 1;
	    break;
	case 'C':
	    if(isdigit(cp[1])) {
	        baseSize = atoi(&cp[1]);
	    } else {
		baseSize = 1;
	    }
	    break;
	case 'D':
	    baseSize = sizeof(double);
	    break;
	case 'F':
	    baseSize = sizeof(float);
	    break;
	case 'L':
	    baseSize = sizeof(int);
	    break;
	case 'S':
	    baseSize = sizeof(short);
	    break;
	default:
	    fprintf(stderr, "Variable type %c not recognized\n", type);
	    exit(2);
	}
	if(type != 'C' && cp != end) {
	    printf("Extra chars at end of name\n");
	    exit(3);
	}
	for(ndims = 0; ndims < MAXDIMS; ndims++) {
	    cp -= 2;	/* skip over the '_' */
	    if(isdigit(*cp)) {
		while(isdigit(*cp))
		    cp--;
		if(*cp == 'V' && cp[-1] == '_') {	/* found a dimension */
		    dimensions[ndims] = atoi(&cp[1]);
		} else {
		    return;
		}
	    } else {
		return;
	    }
	}
	fprintf(stderr, "Too many dimensions for me to handle\n");
	exit(10);
}

void PrintValue(void) {
	int i;

	switch(type) {
	case 'B':
	    printf((hex)? "0x%x  ": "%d  ", *(char *)arg);
	    break;
	case 'C':
	    for(i = 0; i < baseSize; i++) {
		putchar(*(char *)arg);
		arg++;
	    }
	    return;
	    break;
	case 'D':
	    printf("%g  ", *(double *)arg);
	    break;
	case 'F':
	    printf("%g  ", *(float *)arg);
	    break;
	case 'L':
	    printf((hex)? "0x%x  ": "%d  ", *(int *)arg);
	    break;
	case 'S':
	    printf((hex)? "0x%x  ": "%d  ", *(short *)arg);
	    break;
	}
	arg += baseSize;
}
