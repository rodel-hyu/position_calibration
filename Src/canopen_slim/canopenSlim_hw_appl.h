#ifndef __COSLM_HW_APPL_H_
#define __COSLM_HW_APPL_H_

#include <stdint.h>

#define COSLM_BUFLEN            40
#define COSLM_RX_TIMEOUT        1000

extern void canopenSlim_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len);

#endif