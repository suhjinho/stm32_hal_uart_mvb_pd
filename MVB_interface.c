
#include <MVB_interface.h>      /* ACP LPI interface */
#include <string.h>

/* ACP LPA length */
typedef enum _EAcpLpaLen
{
  acp_lpa_len_req_wr_cfg      =  4,  /* length of write configuration */
  acp_lpa_len_req_wr_cfg_ext  =  8,  /* length of write configuration extended */
  acp_lpa_len_req_ds_get      =  2,  /* length of get dataset request */
  acp_lpa_len_req_port_ctrl   =  2,  /* enable or disable port */
  acp_lpa_len_nr_ports        =  2,  /* length of number of ports */
  acp_lpa_len_port_desc       =  4,  /* length of port descriptor */
  acp_lpa_len_port_id         =  2,  /* length of port id */
  acp_lpa_len_port_fresh      =  2   /* length of port freshness */
} EAcpLpaLen;


/* ACP LPA offsets */
typedef enum _EAcpLpaOffs
{
  acp_lpa_offs_cfg_dev       =  0,   /* configuration of device */
  acp_lpa_offs_cfg_tmo       =  2,   /* configuration timeout offset */
  acp_lpa_offs_cfg_hw_cfg    =  4,   /* HW configuration */
  acp_lpa_offs_cfg_bal_ctrl  =  5,   /* balance control */
  acp_lpa_offs_cfg_rsv       =  6,   /* reserved */
  acp_lpa_offs_nr_ports      =  0,   /* number of ports */
  acp_lpa_offs_ports         =  2,   /* LPA ports */
  acp_lpa_offs_set_indx      =  0,   /* index of set */
  /* Suboffsets */
  acp_lpa_suboffs_fcode_addr =  0,   /* fcode and address */
  acp_lpa_suboffs_port_cfg   =  2,   /* port configuration */
  acp_lpa_suboffs_fresh      =  0,   /* freshness */
  acp_lpa_suboffs_ds         =  2    /* dataset */
} EAcpLpaOffs;


/* LPA descriptor */
typedef struct _SAcpLpaDesc
{
  uint16_t  devAddr;  /* device address */
} SAcpLpaDesc;


/* Macro to create ports */
#define START_PORT   				 	static SAcpLpPortCfgDesc s_asPorts[] = {
#define ADD_PORT(ADDR, FCODE, CFG)		{{ADDR, FCODE}, CFG},
#define END_PORT 						{{ 0, 0}, 0}};


/* --------------------------------------------------------
 Input    : dev_address - device address, u16NumPorts - 0~255
 Output   :  -
 ------------------------------------------------------- */
extern void mvbAcpCommInitDevice (uint8_t dev_address, uint16_t u16NumPorts);


/*
Bus administrator (device identification A) configuration for demonstration this example:

  Device address          1
  List of known devices:  0x1, 0x12, 0x13
  Polling list of MD:     0x12, 0x13
    --------------------------------------
    |Basic period    Fcode   Port address|
    -------------------------------------
    |   0             0         0x100    |
    |   0             1         0x101    |
    |   1             2         0x102    |
    |   2             3         0x103    |
    |   3             4         0x104    |
    --------------------------------------
*/


START_PORT
/*idx   0*/ ADD_PORT(284, fcode_4, acp_lp_port_snk )
/*idx   1*/ ADD_PORT(276, fcode_4, acp_lp_port_src )
END_PORT


/* === Locals ============================================================== */
static SAcpLpaDesc  s_sAcpLpaDesc;  /* ACP LPA descriptor */


/* --------------------------------------------------------
 Input  :  u16DevAddr  -  device address
           eHwCfg      -  hardware configuration
 Output :  -
 Return :  acp_sess_res_ok           -  success
           acp_loc_err_already_init  -  ACP LPI interface is already initialized
 ------------------------------------------------------- */

EAcpSessResult mvb_passive (uint16_t devAddr)
{
  SAcpDgram       sDgramReq;     /* request ACP datagram */
  uint8_t*        pu8ReqData;    /* ptr to request data */
  EAcpSessResult  eAcpSessRes;   /* ACP session result */
  uint8_t eMsgPriority;
  uint8_t acp_msg_low  = 1;
  /* uint8_t acp_msg_high = 2; */

  /* by default it is configured low message data priority */
  eMsgPriority = acp_msg_low;

  /* set service and command */
  sDgramReq.u8Service   = acp_code_write_mvb_cfg;
  sDgramReq.u8CmdStatus = 0;
  /* set length of the request */
  sDgramReq.u16DataLen = acp_lpa_len_req_wr_cfg;
  /* fill data of command */
  pu8ReqData = sDgramReq.au8Data;
  /* write device address and MD priority and passive state */
  ACP_PUT_U16 (pu8ReqData + acp_lpa_offs_cfg_dev, (eMsgPriority << 12) | (devAddr & 0xFFF) );
  ACP_PUT_U16 (pu8ReqData + acp_lpa_offs_cfg_tmo, 0);


  eAcpSessRes = acpCommRun (&sDgramReq);
  if (eAcpSessRes == acp_sess_res_ok)
  {
    s_sAcpLpaDesc.devAddr = devAddr;
  }

   return eAcpSessRes;
}


/* --------------------------------------------------------

 Return :  acp_sess_res_ok            -  success
           acp_loc_err_not_init       -  ACP LPI interface is not initialized
           acp_loc_err_fatal          -  corrupted internal structures
           acp_loc_err_already_start  -  transfer/receive was already started
 Descr  :  Start of MVB transfer/receive
 ------------------------------------------------------- */

EAcpSessResult mvb_Regular(void)
{
  SAcpDgram       sDgramReq;    /* request ACP datagram */
  uint8_t*        pu8ReqData;   /* ptr to request data */
  EAcpSessResult  eAcpSessRes;  /* ACP session result */
  uint8_t 	      acp_msg_low  = 1;
  /* uint8_t acp_msg_high = 2; */

  /* set service and command */
  sDgramReq.u8Service   = acp_code_write_mvb_cfg;
  sDgramReq.u8CmdStatus = 0;
  /* set length of the request */
  sDgramReq.u16DataLen = acp_lpa_len_req_wr_cfg;
  /* fill data of command */
  pu8ReqData = sDgramReq.au8Data;
  /* write device address and MD priority and passive state */
  ACP_PUT_U16 (pu8ReqData + acp_lpa_offs_cfg_dev, (acp_line_regular << 14) | (acp_msg_low << 12) | (s_sAcpLpaDesc.devAddr & 0xFFF) );
  ACP_PUT_U16 (pu8ReqData + acp_lpa_offs_cfg_tmo, 0);
  eAcpSessRes = acpCommRun (&sDgramReq);

  return eAcpSessRes;
}  /* acpLpStart */

/* --------------------------------------------------------
 Input  :  asLpPrtCfgDescs  -  array of port configuration descriptors
           u16NrPorts       -  number of ports
 Output :  -
 Return :  acp_sess_res_ok            -  success
           acp_loc_err_in_dta_siz     -  incorrect parameters
           acp_loc_err_not_init       -  ACP LPI interface is not initialized
           acp_loc_err_already_start  -  it is impossible to configure ports in regular state
 Descr  :  Write configuration of LPA ports
 ------------------------------------------------------- */
EAcpSessResult mvb_PortsCfgWr (const SAcpLpPortCfgDesc* asLpPrtCfgDescs, uint16_t u16NrPorts)
{
  SAcpDgram                 sDgramReq;                            /* request ACP datagram */
  uint8_t*                  pu8ReqData;                           /* ptr to request data */
  const SAcpLpPortCfgDesc*  psLpPortCfgDesc;                      /* ptr to port configuration descriptor */
  const SAcpLpPortCfgDesc*  asPortsCfgDescTmp = asLpPrtCfgDescs;  /* temporary arr of port configuration descriptors */
  uint8_t*                  pu8DgrmData;                          /* ptr to datagram data */
  uint16_t                  u16NrPortsTmp = u16NrPorts;           /* temporary number of ports */
  EAcpSessResult            eAcpSessRes;                          /* ACP result code */
  uint8_t 					passive_check;


  /* if maximum of ports exceeds call the function recursively */
  for ( ; u16NrPortsTmp > ACP_LPI_PORTS_MAX_NR;
          asPortsCfgDescTmp += ACP_LPI_PORTS_MAX_NR, u16NrPortsTmp -= ACP_LPI_PORTS_MAX_NR)
  {
    eAcpSessRes = mvb_PortsCfgWr (asPortsCfgDescTmp, ACP_LPI_PORTS_MAX_NR);
    if (eAcpSessRes != acp_sess_res_ok)
      /* error in processing write ports configuration */
      return eAcpSessRes;
  }

  /* set service and command */
  sDgramReq.u8Service   = acp_code_write_ports_cfg;    /* P - write ports configration */
  sDgramReq.u8CmdStatus = 0;
  /* process all ports */
  /* set length of the request */
  sDgramReq.u16DataLen = acp_lpa_len_nr_ports + (u16NrPortsTmp << 2);
  if (sDgramReq.u16DataLen > ACP_DATA_LEN)
  {
	  return acp_loc_err_in_dta_siz;	 /* too many ports */
  }
  /* fill data of request */
  pu8ReqData = sDgramReq.au8Data;
  /* code number of ports */
  ACP_PUT_U16 (pu8ReqData + acp_lpa_offs_nr_ports, u16NrPortsTmp);
  /* code ports configurations */
  for (psLpPortCfgDesc  = asPortsCfgDescTmp, pu8DgrmData = sDgramReq.au8Data + 2;
       psLpPortCfgDesc != asPortsCfgDescTmp + u16NrPortsTmp;
       psLpPortCfgDesc++, pu8DgrmData += 4)
  {
    ACP_PUT_U16 (pu8DgrmData + acp_lpa_suboffs_fcode_addr, (psLpPortCfgDesc->sPortId.u4FCode << 12) | psLpPortCfgDesc->sPortId.u12PortAddr );
    ACP_PUT_U16 (pu8DgrmData + acp_lpa_suboffs_port_cfg, psLpPortCfgDesc->u16LpPortCfg);
  }
  /* execute the command */
  passive_check = acpCommRun (&sDgramReq);


  return passive_check;
}  /* acpLpPortsCfgWr */


/* --------------------------------------------------------
 Input  :  psLpPortId  -  identification of the port
           pDataset    -  ptr to dataset
 Output :  -
 Return :  acp_sess_res_ok        -  success
           acp_loc_err_not_init   -  ACP LPI interface is not initialized
           acp_loc_err_not_start  -  MVB is not in regular state
 Descr  :  Write dataset to MVB traffic store
 ------------------------------------------------------- */
EAcpSessResult mvb_PutDataset (const SAcpLpPortId* PortId, const void* pDataset)
{
  SAcpDgram  sDgramReq;  /* request ACP datagram */

  /* set service and command */
  sDgramReq.u8Service   = acp_code_write_dataset;
  sDgramReq.u8CmdStatus = 0;
  /* set length of the request */
  sDgramReq.u16DataLen = acp_lpa_len_port_id + (uint16_t) (2 << PortId->u4FCode);
  /* code request */
  ACP_PUT_U16 (sDgramReq.au8Data, (PortId->u4FCode << 12) | PortId->u12PortAddr);
  memcpy (sDgramReq.au8Data + 2, pDataset, 2 << PortId->u4FCode);

  return acpCommRun (&sDgramReq);
}  /* acpLpPutDataset */



/* --------------------------------------------------------

 Input  :  psLpPortId  -  identification of the port
 Output :  pDataset    -  dataset
           pu16Fresh   -  freshness
 Return :  acp_sess_res_ok        -  success
           acp_loc_err_not_init   -  ACP LPI interface is not initialized
           acp_loc_err_not_start  -  MVB is not in regular state
 Descr  :  Read dataset from the MVB traffic store , all of traffic store
 ------------------------------------------------------- */
EAcpSessResult mvb_GetDataset (void* pDataset, const SAcpLpPortId* portId)
{
  SAcpDgram       sDgramReq;    /* request ACP datagram */
  SAcpDgram       sDgramRsp;    /* response ACP datagram */
  EAcpSessResult  eAcpSessRes;  /* ACP session result */

  /* set service and command */
  sDgramReq.u8Service   = acp_code_read_dataset;
  sDgramReq.u8CmdStatus = 0;
  /* set length of the request */
  sDgramReq.u16DataLen = acp_lpa_len_req_ds_get;
  /* code request */
  ACP_PUT_U16 (sDgramReq.au8Data, (portId->u4FCode << 12) | portId->u12PortAddr);
  /* execute the command */
  eAcpSessRes = acpCommRun (&sDgramReq);
  if (eAcpSessRes == acp_sess_res_ok)
  {
    memcpy (pDataset, sDgramRsp.au8Data + 2, 2 << portId->u4FCode);
  }
  return eAcpSessRes;
}  /* acpLpGetDataset */


/* --------------------------------------------------------
 Input  :  u16PortAddr  -  port address
 Output :  -
 Return :  acp_sess_res_ok        -  success
           acp_loc_err_not_init   -  ACP LPI interface is not initialized
           acp_loc_err_not_start  -  MVB is not in regular state
 Descr  :  Enable transmitting/receiving data from/to port
 ------------------------------------------------------- */
EAcpSessResult mvb_PortEnable (uint16_t portAddr)
{
  SAcpDgram  sDgramReq;  /* request ACP datagram */

  /* set service and command */
  sDgramReq.u8Service   = acp_code_write_port_enable;
  sDgramReq.u8CmdStatus = 0;
  /* set length of the request */
  sDgramReq.u16DataLen = acp_lpa_len_req_port_ctrl;
  /* code request */
  ACP_PUT_U16 (sDgramReq.au8Data, portAddr);
  /* execute the command */
  return acpCommRun (&sDgramReq);
}  /* acpLpPortEnable */


/* --------------------------------------------------------
 Input  :  u16PortAddr  -  port address
 Output :  -
 Return :  acp_sess_res_ok        -  success
           acp_loc_err_not_init   -  ACP LPI interface is not initialized
           acp_loc_err_not_start  -  MVB is not in regular state
 Descr  :  Disable transmitting/receiving data from/to port
 ------------------------------------------------------- */
EAcpSessResult mvb_PortDisable (uint16_t portAddr)
{
  SAcpDgram  sDgramReq;  /* request ACP datagram */

  /* set service and command */
  sDgramReq.u8Service   = acp_code_write_port_disable;
  sDgramReq.u8CmdStatus = 0;
  /* set length of the request */
  sDgramReq.u16DataLen = acp_lpa_len_req_port_ctrl;
  /* code request */
  ACP_PUT_U16 (sDgramReq.au8Data, portAddr);
  /* execute the command */
  return acpCommRun (&sDgramReq);
}  /* acpLpPortDisable */


extern void mvbCommInitDevice (uint8_t dev_address, uint16_t numPorts)
{
	static uint8_t mvb_step=0;
	static uint8_t last_mvb_step=20;
	static uint8_t interval_step=1;
	static uint8_t check=0;
    volatile static uint8_t interval=0;


    if(interval_step == 0 && timer ==90)
    {
  	   interval++;

  	   if(interval==2)
  	   {
      	  interval_step=1;
  		  interval=0;

  		  if(mvb_step > last_mvb_step)
  		  {
  			  mvb_step = 6;
  		  }
  	   }
    }


  else if(mvb_step == 9 && interval_step == 1)
  {
	  static uint8_t apu8OutData3[33]= {41,42,43,44,45,46,47,48,44,44,44,44,44,44,44,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27};
	  last_mvb_step = mvb_step;
  	  check= mvb_PutDataset(&s_asPorts[1], &apu8OutData3 );

      if(check==port_datawrite)
 	  {
    	  mvb_step++;
    	  interval_step=0;
	  }
  }


  else if(mvb_step == 8 && interval_step == 1)
  {
	  check=mvb_PortDisable(276);

 	  if(check==port_write_disable)
 	  {
 		  mvb_step++;
    	  interval_step=0;
 	  }
  }


  else if(mvb_step == 7 && interval_step == 1)
  {
  	  static uint8_t apu8OutData3[32]= {0};
  	  check = mvb_GetDataset (&apu8OutData3, &s_asPorts);

   	  if(check==mvb_read)
   	  {
   		  mvb_step++;
    	  interval_step=0;
   	  }
  }


  else if(mvb_step == 6 && interval_step == 1)
  {
	  check=mvb_PortEnable(276);

 	  if(check==port_write_enable)
 	  {
 		  mvb_step++;
    	  interval_step=0;
 	  }
  }


  else if(mvb_step == 5 && interval_step == 1)
  {
	  static uint8_t apu8OutData2[33]= {25,26,27,28,29,30,31,32,33,34,35,36,45,46,47,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27};
	  check= mvb_PutDataset(&s_asPorts[1], &apu8OutData2 );

	  if(check==port_datawrite)
	  {
		  mvb_step++;
    	  interval_step=0;
	  }
  }


  else if(mvb_step == 4 && interval_step == 1)
  {
	  static uint8_t apu8OutData3[32]= {0};
	  check = mvb_GetDataset (&apu8OutData3, &s_asPorts);

 	  if(check==mvb_read)
 	  {
 		  mvb_step++;
    	  interval_step=0;
 	  }
  }


  else if(mvb_step == 3 && interval_step == 1)
  {
	  check=mvb_PortEnable(284);

 	  if(check==port_write_enable)
 	  {
 		  mvb_step++;
    	  interval_step=0;
 	  }
  }


  else if(mvb_step == 2 && interval_step == 1)
  {
	  check = mvb_Regular();       /* change mvb's mode passive to regular */

	  if(check==mvb_regular_mode)
	  {
	  	 mvb_step++;
	  	 interval_step=0;
	  }
  }


  else if(mvb_step == 1 && interval_step == 1)
  {
	  check=mvb_PortsCfgWr (s_asPorts, numPorts);

	  if(check==port_config_set_ok)
	  {
		  mvb_step++;
    	  interval_step=0;
	  }
  }

  else if (mvb_step == 0 && interval_step == 1)
  {
	  check=mvb_passive(dev_address);     /* Input device address */

	  if(check==acp_sess_res_ok)
	  {
		  mvb_step++;
    	  interval_step=0;
	  }
  }
}
