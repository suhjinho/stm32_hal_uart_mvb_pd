#include <MVB_common.h>     /* ACP types */

EAcpSessResult mvb_passive(uint16_t devAddr);
EAcpSessResult mvb_Regular(void);
EAcpSessResult mvb_PortsCfgWr(const SAcpLpPortCfgDesc* asLpPrtCfgDescs, uint16_t u16NrPorts);
EAcpSessResult mvb_PutDataset(const SAcpLpPortId* portId, const void* pDataset);
EAcpSessResult mvb_GetDataset(void* pDataset, const SAcpLpPortId* portId);
EAcpSessResult mvb_PortEnable(uint16_t portAddr);
EAcpSessResult mvb_PortDisable(uint16_t portAddr);

extern void mvbCommInitDevice(uint8_t dev_address, uint16_t numPorts);



