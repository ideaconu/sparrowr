#ifndef _EVSYS_H
#define _EVSYS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void(*voidFuncPtr)(void);

uint8_t events_init(uint8_t generator, uint8_t user);

void events_attach_interrupt(uint8_t resource, voidFuncPtr callback);


#ifdef __cplusplus
}
#endif

#endif
