/*
 * dm9051 Ethernet
 */

#ifndef _DM9051X_H_
#define _DM9051X_H_
//#define DM9051_ID		0x90510A46
#define EEPROM_SK_PERIOD	400	/* in us */

#define DM9051_NCR		0x00
#define DM9051_NSR		0x01
#define DM9051_MPTRCR			0x55
#define DM9051_RD_DLY			0x73

#define DM9051_TCR		0x02
#define DM9051_TSR1		0x03
#define DM9051_TSR2		0x04

#define DM9051_RCR		0x05
#define DM9051_RSR		0x06
#define DM9051_ROCR		0x07
#define DM9051_RLENCR			0x52
#define DM9051_BCASTCR			0x53

#define DM9051_BPTR		0x08
#define DM9051_FCTR		0x09
#define DM9051_FCR		0x0A
#define DM9051_PPCSR			0x3D

#define DM9051_EPCR		0x0B
#define DM9051_EPAR		0x0C
#define DM9051_EPDRL		0x0D
#define DM9051_EPDRH		0x0E

#define DM9051_WCR		0x0F

#define DM9051_PAR		0x10
#define DM9051_MAR		0x16

#define DM9051_GPCR		0x1E
#define DM9051_GPR		0x1F

#define DM9051_TRPAL		0x22
#define DM9051_TRPAH		0x23
#define DM9051_RWPAL		0x24
#define DM9051_RWPAH		0x25

#define DM9051_VIDL		0x28
#define DM9051_VIDH		0x29
#define DM9051_PIDL		0x2A
#define DM9051_PIDH		0x2B

#define DM9051_CHIPR		0x2C

#define DM9051_TCSCR		0x31
#define DM9051_RCSCSR		0x32


#define DM9051_SMCR		0x2F
#define DM9051_ATCR			0x30
#define DM9051_PBCR		0x38 /* pbcr version */
#define DM9051_SBCR		0x38 /* pbcr version */

#define DM9051_INTCR		0x39
#define DM9051_INTCKCR			0x54

#define DM9051_NLEDCR		0x57

#define DM9051_MRCMDX		0x70
#define DM9051_MRCMDX1			0x71
#define DM9051_MRCMD		0x72
#define DM9051_MRRL		0x74
#define DM9051_MRRH		0x75

#define DM9051_MWCMDX		0x76
#define DM9051_MWCMD		0x78
#define DM9051_MWRL		0x7A
#define DM9051_MWRH		0x7B
#define DM9051_TXPLL		0x7C
#define DM9051_TXPLH		0x7D

#define DM9051_ISR		0x7E
#define DM9051_IMR		0x7F

#define NCR_EXT_PHY		(1 << 7)
#define NCR_WAKEEN		(1 << 6)
#define NCR_FCOL		(1 << 4)
#define NCR_FDX			(1 << 3)
#define NCR_LBK			(3 << 1)
#define NCR_RST			(1 << 0)

#define NSR_SPEED		(1 << 7)
#define NSR_LINKST		(1 << 6)
#define NSR_WAKEST		(1 << 5)
#define NSR_TX2END		(1 << 3)
#define NSR_TX1END		(1 << 2)
#define NSR_RXOV		(1 << 1)

#define TCR_TJDIS		(1 << 6)
#define TCR_EXCECM		(1 << 5)
#define TCR_PAD_DIS2		(1 << 4)
#define TCR_CRC_DIS2		(1 << 3)
#define TCR_PAD_DIS1		(1 << 2)
#define TCR_CRC_DIS1		(1 << 1)
#define TCR_TXREQ		(1 << 0)

#define TSR_TJTO		(1 << 7)
#define TSR_LC			(1 << 6)
#define TSR_NC			(1 << 5)
#define TSR_LCOL		(1 << 4)
#define TSR_COL			(1 << 3)
#define TSR_EC			(1 << 2)

#define RCR_WTDIS		(1 << 6)
#define RCR_DIS_LONG		(1 << 5)
#define RCR_DIS_CRC		(1 << 4)
#define RCR_ALL			(1 << 3)
#define RCR_RUNT		(1 << 2)
#define RCR_PRMSC		(1 << 1)
#define RCR_RXEN		(1 << 0)

#define RSR_RF			(1 << 7)
#define RSR_MF			(1 << 6)
#define RSR_LCS			(1 << 5)
#define RSR_RWTO		(1 << 4)
#define RSR_PLE			(1 << 3)
#define RSR_AE			(1 << 2)
#define RSR_CE			(1 << 1)
#define RSR_FOE			(1 << 0)

#define FCTR_HWOT(ot)		((ot & 0xf) << 4)
#define FCTR_LWOT(ot)		(ot & 0xf)

#define ISR_LNKCHNG		(1 << 5)
#define ISR_UNDERRUN		(1 << 4)
#define ISR_ROOS		(1 << 3)
#define ISR_ROS			(1 << 2)
#define ISR_PTS			(1 << 1)
#define ISR_PRS			(1 << 0)
#define ISR_CLR_STATUS		(ISR_ROOS | ISR_ROS | ISR_PTS | ISR_PRS)

#define IMR_PAR			(1 << 7)
#define IMR_LNKCHNG		(1 << 5)
#define IMR_UNDERRUN		(1 << 4)
#define IMR_ROOM		(1 << 3)
#define IMR_ROM			(1 << 2)
#define IMR_PTM			(1 << 1)
#define IMR_PRM			(1 << 0)

#define EPCR_REEP		(1 << 5)
#define EPCR_WEP		(1 << 4)
#define EPCR_EPOS		(1 << 3)
#define EPCR_ERPRR		(1 << 2)
#define EPCR_ERPRW		(1 << 1)
#define EPCR_ERRE		(1 << 0)

#define GPCR_GEP_CNTL		(1 << 0)

#define DM9051_PKT_RDY		0x01	/* Packet ready to receive */
#define DM9051_PKT_MAX		1536	/* Received packet max size */

#endif /* _DM9051X_H_ */
