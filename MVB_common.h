
#include "stm32f4xx_hal.h"
#include "timer.h"

#define ACP_FRM_OFFS_LEN       2  /* offset d�lky r�mce */
#define ACP_DATA_LEN         500  /* data length in ACP datagram */
#define ACP_HDR_CRC_LEN        8  /* head and CRC length */
#define ACP_LPI_PORTS_MAX_NR  124  /* maximal ports number (per 1 group)*/

#define DLE                 0x10
#define STX                 0x02
#define ETX                 0x03


/* type of datagram */
typedef enum _EAcpReqType
{
  acp_rqtype_req    =  'c',  /* request ('c') */
  acp_rqtype_rsp    =  'r'   /* response ('r') */
} EAcpReqType;


typedef enum _EAcpServiceType
{
  /* supervisor service */
  acp_code_write_mvb_cfg            =  'C',   /* write MVB configuration */
  acp_code_read_mvb_cfg             =  'c',   /* read  MVB configuration */

  /* port service */
  acp_code_write_ports_cfg          =  'P',   /* write configuration ports */
  acp_code_read_ports_cfg           =  'p',   /* read  configuration ports */
  acp_code_write_dataset            =  'W',   /* write data to port */
  acp_code_write_grp_datasets       =  'G',   /* write data to group of ports */
  acp_code_read_dataset             =  'r',   /* read data from port */
  acp_code_read_grp_datasets        =  'g',   /* read data from group of ports */
  acp_code_write_port_enable        =  'E',   /* write enable port */
  acp_code_write_port_disable       =  'U',   /* write disable port */
  acp_code_write_grp_ports_enable   =  'S',   /* write enable group of ports */
  acp_code_write_grp_ports_disable  =  'R',   /* write disable group of ports */
  acp_code_write_force_data         =  'F',   /* write force data */
  acp_code_write_unforce_data       =  'X',   /* write unforce data */
  acp_code_write_unforce_all        =  'A',   /* write unforce all data */

 }  EAcpServiceType;


/* code of ACP protocol */
typedef enum _EAcpSessResult
{
  acp_sess_res_ok         			=   0, /* all is OK */
  acp_port_config_ok	  			=   0,
  acp_sess_res_failure    			=   1, /* unspecified failure */

  /* Communication return codes */
  acp_sess_res_bad_len   			=   2, /* incorrect frame length */
  acp_sess_res_send_err  			=   3, /* frame sending error */
  acp_sess_res_tmo_err   			=   4, /* session timeout (while waiting for response) */
  acp_sess_rem_canc_err  			=   5, /* session cancelled by partner */
  acp_sess_rem_not_found 			=   6, /* session not found at partner */
  acp_sess_rem_no_free   			=   7, /* no free resources for incoming request at partner */
  acp_sess_rem_dupl       			=   8, /* previous session was not yet finished */
  acp_sess_rem_usr_unreg  			=   9, /* user not registered at partner */
  acp_sess_rem_int_err    			=  10, /* internal error at partner */
  mvb_regular_mode		  			=  12,
  port_datawrite		  			=  13,
  uart_rx_ok 						=  14,
  uart_tx_ok						=  14,
  port_write_enable		  			=  15,
  mvb_read				  			=  16,
  mvb_data_read			  			=  17,
  port_write_disable	  			=  18,
  nothing_happend					=  19,
  port_config_set_ok	  			=  40,

  acp_loc_err_out_dta_siz 			=  109, /* Local output error data size */
  acp_loc_err_in_dta_siz  			=  110, /* Local input error data size */

}  EAcpSessResult;


/* struct of ACP datagram */
typedef struct _SAcpDgram
{
  uint16_t  u16DataLen;              /* data length */
  uint8_t   u8ReqType;               /* type of service (EAcpReqType) */
  uint8_t   u8Service;               /* service (EAcpServiceType) */
  uint8_t   u8CmdStatus;             /* command request / status response */
  uint8_t   u8SessSeqNr;             /* sequential number */
  uint8_t   au8Data [ACP_DATA_LEN];  /* data */
  uint16_t  u16Crc;                  /* CRC */
}  SAcpDgram;



/* Length port */
typedef enum _EAcpPortFcode
{
  fcode_0  =  0,  /* 16-bit port */
  fcode_1  =  1,  /* 32-bit port */
  fcode_2  =  2,  /* 64-bit port */
  fcode_3  =  3,  /* 128-bit port */
  fcode_4  =  4   /* 256-bit port */
} EAcpPortFcode;


/* port configuration */
typedef enum _EAcpLpPortCfg
{
  /* static configuration */
  acp_lp_port_undef  =  0x00,  /* port configuration is not defined */
  acp_lp_port_snk    =  0x01,  /* port sink */
  acp_lp_port_src    =  0x02,  /* port source */
  /* current configuration */
  acp_lp_port_cur_frc    =  0x04,  /* port force */
  acp_lp_port_cur_twc    =  0x08,  /* transfer with checksum */
  acp_lp_port_cur_snk    =  0x10,  /* enable sink */
  acp_lp_port_cur_src    =  0x20,  /* enable source */
} EAcpLpPortCfg;


/* identification of the port */
typedef struct _SAcpLpPortId
{
  uint16_t  u12PortAddr : 12;  /* port address */
  uint16_t  u4FCode     :  4;  /* f-code */
} SAcpLpPortId;


/* port configuration descriptor */
typedef struct _SAcpLpPortCfgDesc
{
  SAcpLpPortId  sPortId;       /* port identification */
  uint16_t      u16LpPortCfg;  /* port configuration (EAcpLpPortCfg) */
} SAcpLpPortCfgDesc;


/* reda data port descriptor */
typedef struct _SAcpLpPortRData
{
  uint16_t           u16Freshness; /* freshness port */
  void              *pDataset;    /* ptr to data */
} SAcpLpPortRData;


/* === Defines ============================================================= */
#define ACP_PUT_U16(p, x) \
  { \
    if (g_uAcpEndian.eEndian == acp_endian_little) \
      *((uint16_t*) (p)) = (uint16_t) (x); \
    else \
      *((uint8_t*) (p)) = (uint8_t) (((x) >> 8) & 0xFF), *((uint8_t*) (p) + 1) = (uint8_t) ((x) & 0xFF); \
  }

#define ACP_GET_U16(p) \
    (g_uAcpEndian.eEndian == acp_endian_little ? \
      *((uint16_t*) (p)) \
    : \
      (uint16_t) ( (*((uint8_t*) (p)) << 8) | *((uint8_t*) (p) + 1) ) )


/* Operation State of Device */
typedef enum _EAcpLineState
{
  acp_line_passive = 0,
  acp_line_regular = 3
} EAcpLineState;


/* Endian type */
typedef enum _EAcpEndian
{
  acp_endian_undef  = 0,            /* undefined endian */
  acp_endian_little = 0x11223344    /* little endian platform */
}  EAcpEndian;


/* Endian detection union */
typedef union _UAcpEndian
{
  uint8_t     au8Test [4];  /* test array */
  EAcpEndian  eEndian;      /* endian */
} UAcpEndian;


extern const UAcpEndian g_uAcpEndian;

EAcpSessResult acpCommRun (SAcpDgram* psReqDgram);
uint8_t acpCommPutSerial (uint8_t* pu8TxData, uint16_t u16Length);



