#ifndef __CANOPENSLIM_H__
#define __CANOPENSLIM_H__

#include "canopenSlim_hw_appl.h"

#include <stdint.h>

typedef enum COSLM_Status{
  COSLM_OK,
  COSLM_TIMEOUT,
  COSLM_ERROR,
}COSLM_Status;

#define COSLM_PDOSTR_LEN        10
typedef struct COSLM_PDOStruct{
  void* data[COSLM_PDOSTR_LEN];
  uint8_t bitlen[COSLM_PDOSTR_LEN];
  uint8_t mappinglen;
}COSLM_PDOStruct;

/*****************************************************************************/
// CANOpen Interface Functions
/*****************************************************************************/
extern COSLM_Status canopenSlim_sendSync();

extern COSLM_Status canopenSlim_writeOD_float(uint8_t nodeId, uint16_t Index, uint8_t subIndex, float data, uint16_t timeout);
extern COSLM_Status canopenSlim_writeOD_uint32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint32_t data, uint16_t timeout);
extern COSLM_Status canopenSlim_writeOD_int32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int32_t data, uint16_t timeout);
extern COSLM_Status canopenSlim_writeOD_uint16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint16_t data, uint16_t timeout);
extern COSLM_Status canopenSlim_writeOD_int16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int16_t data, uint16_t timeout);
extern COSLM_Status canopenSlim_writeOD_uint8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t data, uint16_t timeout);
extern COSLM_Status canopenSlim_writeOD_int8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int8_t data, uint16_t timeout);

extern COSLM_Status canopenSlim_readOD(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t* data, uint8_t* len, uint16_t timeout);

extern void canopenSlim_mappingPDO_init(COSLM_PDOStruct* pdo_struct);
extern void canopenSlim_mappingPDO_float(COSLM_PDOStruct* pdo_struct, float* data);
extern void canopenSlim_mappingPDO_uint32(COSLM_PDOStruct* pdo_struct, uint32_t* data);
extern void canopenSlim_mappingPDO_int32(COSLM_PDOStruct* pdo_struct, int32_t* data);
extern void canopenSlim_mappingPDO_uint16(COSLM_PDOStruct* pdo_struct, uint16_t* data);
extern void canopenSlim_mappingPDO_int16(COSLM_PDOStruct* pdo_struct, int16_t* data);
extern void canopenSlim_mappingPDO_uint8(COSLM_PDOStruct* pdo_struct, uint8_t* data);
extern void canopenSlim_mappingPDO_int8(COSLM_PDOStruct* pdo_struct, int8_t* data);

extern COSLM_Status canopenSlim_sendPDO(uint8_t nodeId, uint8_t channel, COSLM_PDOStruct* pdo_struct);
extern COSLM_Status canopenSlim_readPDO(uint8_t nodeId, uint8_t channel, COSLM_PDOStruct* pdo_struct, uint16_t timeout);

/*****************************************************************************/
// User Install Functions
/*****************************************************************************/
extern void canopenSlim_addRxBuffer(uint16_t cobID, uint8_t* data);
extern void canopenSlim_timerLoop();
#endif