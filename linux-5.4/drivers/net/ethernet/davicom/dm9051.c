/*
 * DM9051 SPI Ethernet Driver (MAC + PHY)
 *
 * Copyright (C) axwdragon.com
 *        based on the code by Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * da2-1588sk

 1.09k
     add MCU_SPI_MAX_LEN for MCU SPI length limit (like MTK fifo 32byte , DMA 1024byte..)
 1.09l
     add load /etc/myconfig mac_config (can load kernel file )
 1.09n
    add soft_irq
    cheng define setting
    add link off tx, rx fifo clean
 1.09n_dts
    add dts support
    del KERNEL_VERSION < 3.10 linux setting
 1.09o
    del KERNEL_VERSION < 4.0 linux setting
    add KERNEL_VERSION > 5.0 linux setting
 1.09p
    edit rx function
 1.09q
    add dts or module select
    edit phy/eeprom read/write function
 1.09r
    add mtk 4byte dma fix , can not use DM9051_SPI_XFER2
    add tx_in_rx , speed up tx function
    add mult write , read bug fix
  1.10a 
    add allwinner cat not use DM9051_SPI_XFER2
  1.10b 
    edit rx function
  1.10c
    edit rx clean isr bit     
    

 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/version.h>
#include <linux/spi/spi.h>
#include "dm9051.h"

//#include <asm/gpio.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>

#define DM9051_ID                       0x90510a46


#define DM_DEBUG                        2               // debug msg level
#define DRV_VERSION                     "1.10c"

/* speed up */
#define DM9051_LINUX_THREADED_IRQ       1               // 1 = Enable , 0 = Disable 
#define DM9051_SPI_XFER2                0               // 1 = Enable , 0 = Disable
#define DM9051_TX_IN_RX                 1               // 1 = Enable , 0 = Disable
#define DM9051_RX_CLEAN_ISR             1               // 1 = Enable , 0 = Disable    

/* system interrupt support mode */
#define DM9051_INT_ACTIVE_LOW           0x01            // 0x01 = INT_pin active low output , 0x00 = INT_pin active high output
#define DM9051_INT_ACTIVE_HIGH          0x00
#define DM9051_INT_ACTIVE_NMOS          0x02            // 0x02 = INT_pin nmos output, 0x00 = INT_pin cmos output
#define DM9051_INT_ACTIVE_CMOS          0x00            

/* other DM9051 parameter */
#define DM9051_RX_MAX_CNT               250             // in irq recvice rx packet count
#define DM9051_TX_MAX_CNT               30              // store in work tx packet count

#define DM9051_SOFT_IRQ_TIME            (HZ / 100)      // (HZ / 1000) // poll cycle time
#define DM9051_RCR_SET                  (RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN)

//#define DM_MSG0(format,arg...)          printk(format,##arg)
#define DM_MSG0(format, ...)             printk(format, ##__VA_ARGS__)

#if DM_DEBUG > 0
    #define DM_MSG1                     DM_MSG0
#else
    #define DM_MSG1(...)
#endif

#if DM_DEBUG > 1
    #define DM_MSG2                     DM_MSG0
#else
    #define DM_MSG2(...)
#endif

#if DM_DEBUG > 2
    #define DM_MSG3                     DM_MSG0
#else
    #define DM_MSG3(...)
#endif

#if DM_DEBUG > 3
    #define DM_MSG4                     DM_MSG0
#else
    #define DM_MSG4(...)
#endif

#define DM9051_PHY                      0x40
#define DM9051_LOCK                     mutex_lock(&dm->lock);
#define DM9051_UNLOCK                   mutex_unlock(&dm->lock);

#define DM9051_TX_FIFO(x,y,z)           dm9051_m_w_tx(x,y,z)
#define DM9051_RX_FIFO(x,y,z)           dm9051_m_r_rx(x,y,z)

typedef struct dm9051_net
{
    struct net_device   *netdev;
    struct spi_device   *spidev;
    struct mutex        lock;
    spinlock_t          statelock;

    u32                 msg_enable ____cacheline_aligned;

    u8                  imr_set;

    u16                 eeprom_size;

    u16                 save_phy_reg0;

    struct mii_if_info  mii;

    struct work_struct  irq_work;

    struct work_struct  tx_send_work;
    struct sk_buff_head tx_skb_q;
    u8                  tx_pkt_cnt;

    struct delayed_work link_work;
    struct work_struct  rx_ctrl_work;

    struct spi_message  spi_msg1;
    struct spi_transfer spi_xfer1;

    u8                  poll_work_active;
    struct delayed_work poll_work;

    int		        rst_gpio;

    u16                 spi_max_len;
    u8                  spi_mosi_fifo[1600]; //spi_tx
#if (! DM9051_SPI_XFER2)
    u8                  spi_miso_fifo[1600]; //spi_rx
#endif
    
    u16                 reg_nsr_cnt;
    u8                  irq_pin_type;
    u8                  irq_pin_duty;
    
#if DM9051_SPI_XFER2
    struct spi_message  spi_msg2;
    struct spi_transfer spi_xfer2_0;
    struct spi_transfer spi_xfer2_1;
#endif    
    
}dm9051_net_t;

static int msg_enable;

static int dm9051_w_spi(struct dm9051_net *dm, u8 *spi_data_value, u16 spi_data_len)
{
    struct spi_transfer *xfer = &dm->spi_xfer1;
    struct spi_message *msg = &dm->spi_msg1;
    int ret;

    xfer->tx_buf = spi_data_value;
    xfer->rx_buf = NULL;
    xfer->len = spi_data_len;

    ret = spi_sync(dm->spidev, msg);

    return ret;
}


static void dm9051_w_reg(struct dm9051_net *dm, u8 reg_addr, u8 data_value)
{
    u8 w_cmd[2] = {(reg_addr | 0x80), data_value};
    int ret;

    ret = dm9051_w_spi(dm, w_cmd, 2);
    if (ret < 0)
        DM_MSG0("dm9051_w_reg: spi_sync() failed\n");
}

static void DM9051_write_locked(struct dm9051_net *dm, u8 reg_addr, u8 data_value)
{
    DM9051_LOCK;
    dm9051_w_reg(dm, reg_addr, data_value);
    DM9051_UNLOCK;

}

//mult register write
static void dm9051_mw_reg(struct dm9051_net *dm, u8 *mw_cmd, u16 mw_cmd_len)
{
    u16 i;
    int ret;

    for(i = 0; i < mw_cmd_len; i += 2)
        mw_cmd[i] |= 0x80;

    ret = dm9051_w_spi(dm, mw_cmd, mw_cmd_len);
    if (ret < 0)
        DM_MSG0("dm9051_mw_reg: spi_sync() failed\n");
}


static u8 dm9051_r_reg(struct dm9051_net *dm, u16 reg_addr)
{
    struct spi_transfer *xfer = &dm->spi_xfer1;
    struct spi_message *msg = &dm->spi_msg1;
    u8 r_cmd[2] = {reg_addr, 0x00};
    u8 r_data[2] = {0x00, 0x00};
    int ret;

    xfer->tx_buf = r_cmd;
    xfer->rx_buf = r_data;
    xfer->len = 2;

    ret = spi_sync(dm->spidev, msg);
    if (ret < 0)
        DM_MSG0("dm9051_r_reg: spi_sync() failed\n");

    return r_data[1];
}

static u8 DM9051_read_locked(struct dm9051_net *dm, u8 reg_addr)
{
    u8 ret;

    DM9051_LOCK;
    ret = dm9051_r_reg(dm, reg_addr);
    DM9051_UNLOCK;

    return ret;
}

#if DM9051_SPI_XFER2
static void dm9051_m_w_tx(struct dm9051_net *dm, u8 *tx_data, u16 data_len)
{
    struct spi_transfer *xfer;
    struct spi_message *msg = &dm->spi_msg2;
    u8 wr_cmd[1];
    u16 tx_point = 0;
    int ret;

    wr_cmd[0] = /*DM9051_MWCMD | 0x80 */0xF8;

    xfer = &dm->spi_xfer2_0;
    xfer->tx_buf = wr_cmd;
    xfer->rx_buf = NULL;
    xfer->len = 1;

    DM_MSG4("dm9051_m_w_data XFER2 data_len %d , spi_max_len %d , mult_write_loop %d, other len  %d\n",
             data_len,
             dm->spi_max_len,
             (data_len / dm->spi_max_len),
             (data_len % dm->spi_max_len));

    if(data_len & 0x01) data_len++;            
             
    do
    {
        xfer = &dm->spi_xfer2_1;

        xfer->len = data_len - tx_point;
        if(xfer->len > dm->spi_max_len)
            xfer->len = dm->spi_max_len;

        xfer->tx_buf = &tx_data[tx_point];
        xfer->rx_buf = NULL;

        ret = spi_sync(dm->spidev, msg);
        if (ret < 0)
            goto tx_end;

        tx_point += xfer->len;
    }while(data_len > tx_point);

tx_end:
    if (ret < 0)
        DM_MSG0("dm9051_m_w_tx: spi_sync() failed\n");
}

static void dm9051_m_r_rx(struct dm9051_net *dm, u8 *data_value, u16 data_len)
{
    struct spi_transfer *xfer;
    struct spi_message *msg = &dm->spi_msg2;
    u8 wr_cmd[1];
    u8 *zero_data = dm->spi_mosi_fifo;
    u16 rx_point = 0;
    int ret;

//    memset(zero_data, 0x00, dm->spi_max_len);

    DM_MSG4("dm9051_m_r_data XFER2 data_len %d , spi_max_len %d , mult_read_loop %d, other len  %d\n", 
            data_len,
            dm->spi_max_len,
            (data_len / dm->spi_max_len),
            (data_len % dm->spi_max_len));
            
    if(data_len & 0x01) data_len++;            

    wr_cmd[0] = DM9051_MRCMD;

    xfer = &dm->spi_xfer2_0;
    xfer->tx_buf = wr_cmd;
    xfer->rx_buf = NULL;
    xfer->len = 1;

    do
    {
        xfer = &dm->spi_xfer2_1;

        xfer->len = data_len - rx_point;
        if(xfer->len > dm->spi_max_len)
            xfer->len = dm->spi_max_len;

        xfer->tx_buf = zero_data;
        xfer->rx_buf = &data_value[rx_point];

        ret = spi_sync(dm->spidev, msg);
        if (ret < 0)
            goto rx_end;

        rx_point += xfer->len;
    }while(data_len > rx_point);

rx_end:
    if (ret < 0)
        DM_MSG0("dm9051_m_r_rx: spi_sync() failed\n");
}



#else
static void dm9051_m_w_tx(struct dm9051_net *dm, u8 *tx_data, u16 data_len)
{
    struct spi_transfer *xfer = &dm->spi_xfer1;
    struct spi_message *msg = &dm->spi_msg1;
    u8 *new_tx_data = dm->spi_mosi_fifo; //spi_tx
    u8 *discard_data = dm->spi_miso_fifo; //spi_rx
    u16 tx_point = 0;
    u16 calc_len;
    int ret;

    new_tx_data[0] = /*DM9051_MWCMD | 0x80 */0xF8;

    DM_MSG4("dm9051_m_w_data tx_len %d , spi_max_len %d , mult_write_loop %d, other len  %d\n",
            data_len,
            dm->spi_max_len,
            (data_len / dm->spi_max_len),
            (data_len % dm->spi_max_len));
            
    if(data_len & 0x01) data_len++;

    do
    {
        calc_len = data_len - tx_point;
        if(calc_len > dm->spi_max_len)
            calc_len = dm->spi_max_len;

        memcpy(&new_tx_data[1], &tx_data[tx_point], calc_len);
        tx_point += calc_len;

        xfer->tx_buf = new_tx_data;
        xfer->rx_buf = discard_data;
        xfer->len = (calc_len + 1);

        ret = spi_sync(dm->spidev, msg);
        if (ret < 0)
            goto tx_end;
    }while(data_len > tx_point);


tx_end:
    if (ret < 0)
        DM_MSG0("dm9051_m_w_tx: spi_sync() failed\n");
}

static void dm9051_m_r_rx(struct dm9051_net *dm, u8 *data_value, u16 data_len)
{
    struct spi_transfer *xfer = &dm->spi_xfer1;
    struct spi_message *msg = &dm->spi_msg1;
    u8 *new_rx_data = dm->spi_miso_fifo; //spi_rx
    u8 *new_tx_cmd = dm->spi_mosi_fifo; //spi_tx
    u16 rx_point = 0; //point zero is spi_cmd , point 1 is real_data_addr
    u16 calc_len;
    int ret;

    DM_MSG4("dm9051_m_r_rx data_len %d , spi_max_len %d , mult_read_loop %d, other len  %d\n", 
            data_len,
            dm->spi_max_len,
            (data_len / dm->spi_max_len),
            (data_len % dm->spi_max_len));

    if(data_len & 0x01) data_len++;

    memset(&new_tx_cmd[1], 0x00, sizeof(dm->spi_mosi_fifo));
    new_tx_cmd[0] = DM9051_MRCMD;

    do
    {
        calc_len = data_len - rx_point;
        if(calc_len > dm->spi_max_len)
            calc_len = dm->spi_max_len;

        xfer->tx_buf = new_tx_cmd;
        xfer->rx_buf = new_rx_data;
        xfer->len = (calc_len + 1);

        ret = spi_sync(dm->spidev, msg);
        if (ret < 0)
            goto rx_end;

        memcpy(&data_value[rx_point], &new_rx_data[1], calc_len);
        rx_point += calc_len;
    }while(data_len > rx_point);
        
rx_end:
    if (ret < 0)
        DM_MSG0("dm9051_m_r_rx: spi_sync() failed\n");
}
#endif

static int dm9051_tx_function(struct dm9051_net *dm)
{

    struct net_device *dev = dm->netdev;
    struct sk_buff        *tx_skb;

    u8 txlen_cmd[4] = {
            DM9051_TXPLL , 0x00,
            DM9051_TXPLH , 0x00};
    u8 tx_end_status;
    int ret = -1;

    if(!skb_queue_empty(&dm->tx_skb_q))
    {
        tx_end_status = dm9051_r_reg(dm, DM9051_NSR) & (NSR_TX1END | NSR_TX2END);

        if(tx_end_status)
        {
            if((NSR_TX1END | NSR_TX2END) == tx_end_status)
                 dm9051_w_reg(dm, DM9051_MPTRCR, 0x02);

            tx_skb = skb_dequeue(&dm->tx_skb_q);

            if(NULL != tx_skb)
            {
                txlen_cmd[1] = (tx_skb->len  & 0xff);
                txlen_cmd[3] = ((tx_skb->len >> 8) & 0xff);

                dm9051_mw_reg(dm, txlen_cmd, 4);

                /* Move data to DM9051 TX RAM */
                DM9051_TX_FIFO(dm, tx_skb->data, tx_skb->len);

                dm9051_w_reg(dm, DM9051_TCR, 0x01);

                dev->stats.tx_packets++;
                dev->stats.tx_bytes += tx_skb->len;
                dm->tx_pkt_cnt--;

                dev_kfree_skb(tx_skb);
            }

            if(skb_queue_empty(&dm->tx_skb_q))
                dm->tx_pkt_cnt = 0;

            if(dm->tx_pkt_cnt < DM9051_TX_MAX_CNT)
                netif_wake_queue(dev);
            ret = 0;
        }
    }

    return ret;
}


static void dm9051_tx_send_work(struct work_struct *work)
{
    struct dm9051_net *dm = container_of(work, struct dm9051_net, tx_send_work);
    struct net_device *dev = dm->netdev;
    struct sk_buff        *tx_skb;
    u8 tx_max_send_count = DM9051_TX_MAX_CNT;

    DM9051_LOCK;

    if(netif_carrier_ok(dev))  //device is link
    {
        while(!dm9051_tx_function(dm))
        {
            if(!(tx_max_send_count--))
                break;
        }

        if(!skb_queue_empty(&dm->tx_skb_q))
            schedule_work(&dm->tx_send_work);
    }
    else
    {
        while(!skb_queue_empty(&dm->tx_skb_q))
        {
            tx_skb = skb_dequeue(&dm->tx_skb_q);
            dev_kfree_skb(tx_skb);
        }
        dm->tx_pkt_cnt = 0;
    }

    DM9051_UNLOCK;
}

static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);
    netdev_tx_t ret = NETDEV_TX_OK;

    spin_lock(&dm->statelock);

    if ((dm->tx_pkt_cnt >= DM9051_TX_MAX_CNT) || !netif_carrier_ok(dev))
    {
        netif_stop_queue(dev);
        ret = NETDEV_TX_BUSY;
    }
    else
    {
        /* Remember the skb for deferred processing */
        dm->tx_pkt_cnt++;
        skb_queue_tail(&dm->tx_skb_q, skb);
    }

    spin_unlock(&dm->statelock);

    schedule_work(&dm->tx_send_work);

    return ret;
}


void dm9051_jump_data(struct dm9051_net *dm, u16 data_len)
{
    u8 jump_data[512];
    u16 jump_len;
    
    do
    {
        if(data_len > sizeof(jump_data))
            jump_len = sizeof(jump_data);
        else
            jump_len = data_len;
            
        DM9051_RX_FIFO(dm, jump_data, jump_len);
        
        data_len -= jump_len;
    }while(data_len);
}

struct dm9051_rxhdr {
    u8    RxPktReady;
    u8    RxStatus;
    u16   RxLen;
} __packed;

static u16 dm9051_rx_pkts(struct dm9051_net *dm)
{
    struct net_device *dev = dm->netdev;
    struct sk_buff *rx_skb;
    struct dm9051_rxhdr rx_head;
    u8 rx_ready;
    u8 rec_packet = DM9051_RX_MAX_CNT;

    do
    {
        if(dm9051_r_reg(dm, DM9051_NSR) & DM9051_PKT_RDY)
        {
            rx_ready = dm9051_r_reg(dm, DM9051_MRCMDX);
            rx_ready = dm9051_r_reg(dm, DM9051_MRCMDX);

            if(rx_ready != DM9051_PKT_RDY)
            {
                if((rx_ready > DM9051_PKT_RDY) || dm->reg_nsr_cnt++ > 10)
                {
                    goto rx_point_err;
                }
                goto rx_fifo_end;
            }        
            dm->reg_nsr_cnt = 0;
        }
        else
        {
#if DM9051_RX_CLEAN_ISR
	        dm9051_w_reg(dm, DM9051_ISR, ISR_PRS);
#endif
            goto rx_fifo_end;
        }
        
        DM9051_RX_FIFO(dm, (u8 *)&rx_head, sizeof(rx_head));

        rx_head.RxLen = le16_to_cpu(rx_head.RxLen);

        DM_MSG3("dm9051: dm9051_rx_pkts  %04x , len %04x \n", rx_head.RxStatus, rx_head.RxLen);

        /* rxhdr.rxstatus is identical to RSR register. */
        if ((rx_head.RxStatus & (RSR_RWTO | RSR_RF)) ||
            (rx_head.RxLen > sizeof(dm->spi_mosi_fifo)) ||
            (rx_head.RxLen < 64))
        {
            DM_MSG0("dm9051: RxLen err %02x , %d \n", rx_head.RxStatus, rx_head.RxLen);
            dev->stats.rx_length_errors++;
            goto rx_point_err;
        }

        /* the length of the packet includes the 32bit CRC */
        /* start the packet dma process, and set auto-dequeue rx */
        rx_skb = NULL;
        if(rx_head.RxStatus & (RSR_FOE | RSR_CE))
        {
            DM_MSG0("dm9051: RxStatus err %02x \n", rx_head.RxStatus);
            dev->stats.rx_crc_errors++;
            dm9051_jump_data(dm, rx_head.RxLen);
        }        
        else
        {
            rx_skb = dev_alloc_skb(rx_head.RxLen + 5);
            if(NULL != rx_skb)
            {
                skb_put(rx_skb, rx_head.RxLen + 1);

                DM9051_RX_FIFO(dm, rx_skb->data, rx_head.RxLen);

                rx_skb->protocol = eth_type_trans(rx_skb, dm->netdev);

                netif_rx_ni(rx_skb);   //       fix NOHZ local_softirq_pending 08 warning

                dev->stats.rx_packets++;
                dev->stats.rx_bytes += rx_head.RxLen;
            }
            else
            {
                DM_MSG0("dm9051: dev_alloc_skb fail %d \n", rx_head.RxLen);
                dm9051_jump_data(dm, rx_head.RxLen);
            }
        }

#if DM9051_TX_IN_RX
        dm9051_tx_function(dm);
#endif
    }while(rec_packet--);

rx_fifo_end:
    return 0;

rx_point_err:
    DM_MSG1("dm9051: status check fail: %02x %d %02x%02x %02x %02x\n", 
            rx_ready,
            dm->reg_nsr_cnt,
            dm9051_r_reg(dm, DM9051_MRRH),
            dm9051_r_reg(dm, DM9051_MRRL),
            dm9051_r_reg(dm, DM9051_IMR),
            dm9051_r_reg(dm, DM9051_VIDL)
            );
            dev->stats.rx_fifo_errors++;
            dm9051_w_reg(dm, DM9051_MPTRCR, 0x01);
            dm->reg_nsr_cnt = 0;

    return 0;
}

static void dm9051_link_work(struct work_struct *w)
{
    struct delayed_work *dw = to_delayed_work(w);
    dm9051_net_t *dm = container_of(dw, dm9051_net_t, link_work);
    struct net_device *dev = dm->netdev;
    u8 nsr, old_carrier, new_carrier, ncr;
    struct sk_buff *tx_skb;

    DM9051_read_locked(dm, DM9051_NSR);
    nsr = DM9051_read_locked(dm, DM9051_NSR);
    old_carrier = netif_carrier_ok(dev) ? 1 : 0;
    new_carrier = (nsr & NSR_LINKST) ? 1 : 0;

    if(old_carrier != new_carrier)
    {
        if(new_carrier)
        {
            ncr = DM9051_read_locked(dm, DM9051_NCR);
            DM_MSG0("dm9051: link up, %dMbps, %s-duplex\n",
                 (nsr & NSR_SPEED) ? 10 : 100,
                 (ncr & NCR_FDX) ? "full" : "half");
        }
        else
            DM_MSG0("dm9051: link down\n");

        if(new_carrier)
        {
            netif_carrier_on(dev);
            netif_wake_queue(dev);
        }
        else
        {
            netif_carrier_off(dev);

            while(!skb_queue_empty(&dm->tx_skb_q))
            {
                tx_skb = skb_dequeue(&dm->tx_skb_q);
                dev_kfree_skb(tx_skb);
            }
            dm->tx_pkt_cnt = 0;

            DM9051_write_locked(dm, DM9051_MPTRCR, 0x01);
            dm->reg_nsr_cnt = 0;
        }
    }
}

static void dm9051_irq_unlock(struct dm9051_net *dm)
{
    u8 isr_status;
    u8 irq_cmd[4] = {
            DM9051_IMR, IMR_PAR,
            DM9051_ISR, 0x00};
    u8 * clean_isr = irq_cmd;

    isr_status = dm9051_r_reg(dm, DM9051_ISR);

#if DM9051_RX_CLEAN_ISR
    irq_cmd[3] = isr_status & ~(ISR_PRS);
#else
    irq_cmd[3] = isr_status;
#endif

    dm9051_mw_reg(dm, clean_isr, 4);

    if (isr_status & ISR_LNKCHNG)
    {
        /* fire a link-change request */
        schedule_delayed_work(&dm->link_work, 1);
    }

    /* Get DM9051 interrupt status */
    if (isr_status & (ISR_ROOS |ISR_ROS | ISR_PRS))
    {
        dm9051_rx_pkts(dm);
    }

    dm9051_w_reg(dm, DM9051_IMR , dm->imr_set);
}

static void dm9051_irq_work(struct work_struct *work)
{
    struct dm9051_net *dm = container_of(work, struct dm9051_net, irq_work);

    DM9051_LOCK;

    dm9051_irq_unlock(dm);

    DM9051_UNLOCK;

    if(!dm->poll_work_active)
        enable_irq(dm->netdev->irq);
}

static irqreturn_t dm9051_irq(int irq, void *_dm)
{
    struct dm9051_net *dm = _dm;

#if DM9051_LINUX_THREADED_IRQ
    DM9051_LOCK;
    dm9051_irq_unlock(dm);
    DM9051_UNLOCK;
#else
    if(!dm->poll_work_active)
        disable_irq_nosync(irq);
    
    if(!work_pending(&dm->irq_work))
        schedule_work(&dm->irq_work);
#endif

    return IRQ_HANDLED;
}

static void dm9051_poll_work(struct work_struct *w)
{
    struct delayed_work *dw = to_delayed_work(w);
    dm9051_net_t *dm = container_of(dw, dm9051_net_t, poll_work);

    DM_MSG3("dm9051_poll_work in\n");

    if(!work_pending(&dm->irq_work))
        schedule_work(&dm->irq_work);

    schedule_delayed_work(&dm->poll_work, DM9051_SOFT_IRQ_TIME);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void dm9051_poll_controller(struct net_device *dev)
{
//     disable_irq(dev->irq);
     DM_MSG3("dm9051_poll_controller in\n");
     dm9051_irq(dev->irq, dev);
//     enable_irq(dev->irq);
}
#endif


/*
 *  Set DM9051 multicast address
 */
static void
dm9051_hash_table_unlocked(struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);
    struct netdev_hw_addr *ha;
    int i, oft;
    u32 hash_val;
    u16 hash_table[4];
    u8 rcr = DM9051_RCR_SET;

    for (i = 0; i < ETH_ALEN; i++)
        dm9051_w_reg(dm, DM9051_PAR + i , dev->dev_addr[i]);

    /* Clear Hash Table */
    for (i = 0; i < 4; i++)
    hash_table[i] = 0x0;

    if (dev->flags & IFF_PROMISC)
    rcr |= RCR_PRMSC;

    if (dev->flags & IFF_ALLMULTI)
    rcr |= RCR_ALL;

    /* the multicast address in Hash Table : 64 bits */
    netdev_for_each_mc_addr(ha, dev)
    {
        hash_val = ether_crc_le(6, ha->addr) & 0x3f;
        hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
    }

    /* Write the hash table to MAC MD table */
    for (i = 0, oft = DM9051_MAR; i < 4; i++)
    {
        dm9051_w_reg(dm, oft++, hash_table[i]);
        dm9051_w_reg(dm, oft++, hash_table[i] >> 8);
    }

    dm9051_w_reg(dm, DM9051_RCR, rcr);
}

static void dm9051_hash_table_work(struct work_struct *work)
{
    struct dm9051_net *dm = container_of(work, struct dm9051_net, rx_ctrl_work);
    struct net_device *dev = dm->netdev;

    DM9051_LOCK;

    dm9051_hash_table_unlocked(dev);

    DM9051_UNLOCK;
}

static void
dm9051_hash_table(struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);

    schedule_work(&dm->rx_ctrl_work);
}

static void dm9051_phy_write(struct net_device *dev,
                 int phy, int reg, int value)
{
    struct dm9051_net *dm = netdev_priv(dev);
    u8  w_time = 50;

    DM9051_LOCK;
    /* Fill the phyxcer register into REG_0C */
    dm9051_w_reg(dm, DM9051_EPAR, DM9051_PHY | reg);
    /* Fill the written data into REG_0D & REG_0E */
    dm9051_w_reg(dm, DM9051_EPDRL, value);
    dm9051_w_reg(dm, DM9051_EPDRH, value >> 8);
    /* Issue phyxcer write command */
    dm9051_w_reg(dm, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);

    do
    {
        /* Wait write complete */
        udelay(10);
    }while(--w_time && (dm9051_r_reg(dm, DM9051_EPCR) & (EPCR_ERPRW | EPCR_ERRE)));
    /* Wait read complete */
    udelay(500);

    /* Clear phyxcer write command */
    dm9051_w_reg(dm, DM9051_EPCR, 0x0);
    DM9051_UNLOCK;

    if(!w_time) 
        DM_MSG0("dm9051_phy_write time out\n");
    else
        DM_MSG3("dm9051_phy_write %d, %04x \n", reg, value);

}

static int dm9051_phy_read(struct net_device *dev, int phy, int reg)
{
    struct dm9051_net *dm = netdev_priv(dev);
    u16 phy_data;
    u8  r_time = 20;

    DM9051_LOCK;
    /* Fill the phyxcer register into REG_0C */
    dm9051_w_reg(dm, DM9051_EPAR, DM9051_PHY | reg);
    /* Issue phyxcer read command */
    dm9051_w_reg(dm, DM9051_EPCR, EPCR_EPOS| EPCR_ERPRR);

    do
    {
        /* Wait read complete */
        udelay(10);
    }while(--r_time && (dm9051_r_reg(dm, DM9051_EPCR) & (EPCR_ERPRR | EPCR_ERRE)));

    /* Clear phyxcer read command */
    dm9051_w_reg(dm, DM9051_EPCR, 0x0);

    /* The read data keeps on REG_0D & REG_0E */
    phy_data = (dm9051_r_reg(dm, DM9051_EPDRL) |
        (dm9051_r_reg(dm, DM9051_EPDRH) << 8));
    DM9051_UNLOCK;

    if(!r_time) 
        DM_MSG0("dm9051_phy_read time out\n");
    else
        DM_MSG3("dm9051_phy_read %d, %04x \n", reg, phy_data);

    return phy_data;
}

static void dm9051_soft_reset(struct dm9051_net *dm)
{
    dm9051_w_reg(dm, DM9051_NCR, 0x03);
    mdelay(1);    /* wait a short time to effect reset */
    dm9051_w_reg(dm, DM9051_NCR, 0x00);
    mdelay(1);    /* wait a short time to effect reset */
    dm9051_w_reg(dm, 0x5e, 0x00);
}


static void dm9051_init_chip(struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);

    DM9051_LOCK;
    dm9051_soft_reset(dm);

    /* Program operating register */
    dm9051_w_reg(dm, DM9051_IMR, IMR_PAR);
    dm9051_w_reg(dm, DM9051_TCR, 0x00);    /* TX Polling clear */
    dm9051_w_reg(dm, DM9051_BPTR, 0x3f);    /* Less 3Kb, 200us */
    dm9051_w_reg(dm, DM9051_FCR, 0x28);    /* Flow Control */

    dm9051_w_reg(dm, DM9051_PPCSR, 0x08);
    /* clear TX status */
    dm9051_w_reg(dm, DM9051_NSR, NSR_WAKEST);
    /* Clear interrupt status */
    dm9051_w_reg(dm, DM9051_ISR, ISR_CLR_STATUS);

    dm9051_w_reg(dm, DM9051_ATCR, 0x00);

    dm9051_w_reg(dm, DM9051_NLEDCR, 0x81);
    dm9051_w_reg(dm, DM9051_SBCR, 0x64);

    dm9051_w_reg(dm, DM9051_BCASTCR, 0xc0);

    /* Set address filter table */
    dm9051_hash_table_unlocked(dev);

    /* Init Driver variable */
    dm->tx_pkt_cnt = 0;
    dm->reg_nsr_cnt = 0;

    netif_start_queue(dm->netdev);

    /* Enable TX/RX interrupt mask */
    if(dm->poll_work_active)
    {    
        dm->imr_set = (IMR_PAR);
    }
    else
    {
        dm9051_w_reg(dm, DM9051_INTCR, dm->irq_pin_type);
        dm9051_w_reg(dm, DM9051_INTCKCR, dm->irq_pin_duty);
        dm->imr_set = (IMR_PAR | IMR_LNKCHNG | IMR_ROOM | IMR_ROM | IMR_PRM);        
    }
    
    dm9051_w_reg(dm, DM9051_IMR, dm->imr_set);
    DM9051_UNLOCK;
}


static int dm9051_net_open(struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);

    /* setup transmission parameters */
    DM9051_LOCK;
    dm9051_w_reg(dm, DM9051_GPR, 0x00);    /* Enable PHY */
    DM9051_UNLOCK;
    mdelay(10);

    dm9051_phy_write(dev, 0x01, 0x14, 0x0200);
    dm9051_phy_write(dev, 0x01, 0x00, dm->save_phy_reg0);

    netif_carrier_off(dev);

    if(dm->poll_work_active)
    {
        schedule_delayed_work(&dm->poll_work, 3 * HZ);
    }
#if (! DM9051_LINUX_THREADED_IRQ)
    else
    {
        // re-enable irq function
        enable_irq(dev->irq);
    }
#endif    

    dm9051_init_chip(dev);

    schedule_delayed_work(&dm->link_work, HZ);

    return 0;
}

static int dm9051_net_stop(struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);

    netif_stop_queue(dev);

    DM9051_LOCK;
    dm9051_w_reg(dm, DM9051_IMR, IMR_PAR);    /* Disable all interrupt */
    dm9051_w_reg(dm, DM9051_RCR, 0x00);    /* Disable RX */
    DM9051_UNLOCK;

    dm->save_phy_reg0 = dm9051_phy_read(dev, 0x01, 0x00);

    dm9051_phy_write(dev, 0x01, 0x00, 0x0800);

    DM9051_LOCK;
    dm9051_w_reg(dm, DM9051_GPR, 0x01);    /* Disable PHY */
    DM9051_UNLOCK;

    /* stop any outstanding work */
    if(dm->poll_work_active)
    {
        cancel_delayed_work_sync(&dm->poll_work);
    }
#if (! DM9051_LINUX_THREADED_IRQ)    
    else
    {
        //  disable_irq_nosync(dev->irq);
        disable_irq(dev->irq);
        cancel_work_sync(&dm->irq_work);
    }
#endif

    cancel_work_sync(&dm->rx_ctrl_work);
    cancel_work_sync(&dm->tx_send_work);
    while(!skb_queue_empty(&dm->tx_skb_q))
    {
        struct sk_buff *tx_skb = skb_dequeue(&dm->tx_skb_q);
        dev_kfree_skb(tx_skb);
    }
    cancel_delayed_work_sync(&dm->link_work);

    return 0;
}


static int dm9051_set_mac_address(struct net_device *dev, void *addr)
{
    struct sockaddr *sa = addr;
    struct dm9051_net *dm = netdev_priv(dev);
    int i;

    if (netif_running(dev))
        return -EBUSY;

    if (!is_valid_ether_addr(sa->sa_data))
        return -EADDRNOTAVAIL;

    memcpy(dev->dev_addr, sa->sa_data, ETH_ALEN);

    DM9051_LOCK;
    for (i = 0; i < ETH_ALEN; i++)
        dm9051_w_reg(dm, DM9051_PAR + i , dev->dev_addr[i]);
    DM9051_UNLOCK;

    return 0;
}

static int dm9051_net_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
    struct dm9051_net *dm = netdev_priv(dev);

    if (!netif_running(dev))
        return -EINVAL;

    return generic_mii_ioctl(&dm->mii, if_mii(req), cmd, NULL);
}

static const struct net_device_ops dm9051_netdev_ops =
{
    .ndo_open               = dm9051_net_open,
    .ndo_stop               = dm9051_net_stop,
    .ndo_do_ioctl           = dm9051_net_ioctl,
    .ndo_start_xmit         = dm9051_start_xmit,
    .ndo_set_mac_address    = dm9051_set_mac_address,
    .ndo_set_rx_mode        = dm9051_hash_table,
    .ndo_validate_addr      = eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
    .ndo_poll_controller    = dm9051_poll_controller,
#endif
};

/* ethtool support */

static void dm9051_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *di)
{
    strlcpy(di->driver, "DM9051", sizeof(di->driver));
    strlcpy(di->version, DRV_VERSION, sizeof(di->version));
    strlcpy(di->bus_info, dev_name(dev->dev.parent), sizeof(di->bus_info));
}

static u32 dm9051_get_msglevel(struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);
    return dm->msg_enable;
}

static void dm9051_set_msglevel(struct net_device *dev, u32 set_val)
{
    struct dm9051_net *dm = netdev_priv(dev);
    dm->msg_enable = set_val;
}

static int dm9051_get_link_ksettings(struct net_device *dev, struct ethtool_link_ksettings *cmd)
{
    struct dm9051_net *dm = netdev_priv(dev);
	mii_ethtool_get_link_ksettings(&dm->mii, cmd);

    return 0;
}

static int dm9051_set_link_ksettings(struct net_device *dev, const struct ethtool_link_ksettings *cmd)
{
    struct dm9051_net *dm = netdev_priv(dev);
    return mii_ethtool_set_link_ksettings(&dm->mii, cmd);
}

static u32 dm9051_get_link(struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);
    return mii_link_ok(&dm->mii);
}

static int dm9051_nway_reset(struct net_device *dev)
{
    struct dm9051_net *dm = netdev_priv(dev);
    return mii_nway_restart(&dm->mii);
}

static int dm9051_get_eeprom_len(struct net_device *dev)
{
    return 128;
}

/*
 * Write a word data to SROM
 */
static void DM9051_write_eeprom(struct dm9051_net *dm, int offset, u8 *data)
{
    u8  w_time = 50;

    DM9051_LOCK;
    dm9051_w_reg(dm, DM9051_EPAR, offset);
    dm9051_w_reg(dm, DM9051_EPDRH, data[1]);
    dm9051_w_reg(dm, DM9051_EPDRL, data[0]);
    dm9051_w_reg(dm, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);

    do
    {
        /* Wait write complete */
        //udelay(100);
        mdelay(500);
    }while(--w_time && (dm9051_r_reg(dm, DM9051_EPCR) & (EPCR_ERPRW | EPCR_ERRE)));
    /* Wait read complete */

    dm9051_w_reg(dm, DM9051_EPCR, 0);
    DM9051_UNLOCK;

    if(!w_time) DM_MSG0("dm9051_eeprom_write time out\n");

}



/*
 *  Read a word data from EEPROM
 */
static void DM9051_read_eeprom(struct dm9051_net *dm, int offset, u8 *data)
{
    u8  r_time = 20;

    DM9051_LOCK;
    dm9051_w_reg(dm, DM9051_EPAR, offset);
    dm9051_w_reg(dm, DM9051_EPCR, EPCR_ERPRR);

    do
    {
        /* Wait read complete */
        //udelay(500);
        mdelay(5);
    }while(--r_time && (dm9051_r_reg(dm, DM9051_EPCR) & (EPCR_ERPRR | EPCR_ERRE)));

    dm9051_w_reg(dm, DM9051_EPCR, 0x0);
    data[1] = dm9051_r_reg(dm, DM9051_EPDRH);
    data[0] = dm9051_r_reg(dm, DM9051_EPDRL);
    DM9051_UNLOCK;

    DM_MSG0("dm9051_eeprom_read addr %x , value %02x%02x\n", offset, data[1], data[0]);

    if(!r_time) DM_MSG0("dm9051_eeprom_read time out\n");
}



static int dm9051_get_eeprom(struct net_device *dev,
                struct ethtool_eeprom *eeprom, u8 *bytes)
{
    struct dm9051_net *dm = netdev_priv(dev);
    int offset = eeprom->offset;
    int len = eeprom->len;
    int ret_val = 0;
    u16 i;

    if (eeprom->len == 0)
        return -EINVAL;

    if (eeprom->len > dm->eeprom_size)
        return -EINVAL;

    for (i = 0; i < len; i += 2)
        DM9051_read_eeprom(dm, (offset + i) / 2, bytes + i);

    return ret_val;
}

#define DM_EEPROM_MAGIC        (0x444D394B)
static int dm9051_set_eeprom(struct net_device *dev,
                struct ethtool_eeprom *eeprom, u8 *bytes)
{
    struct dm9051_net *dm = netdev_priv(dev);
    int offset = eeprom->offset;
    int len = eeprom->len;
    u16 i;

    if (eeprom->len == 0)
        return -EOPNOTSUPP;

    if (eeprom->len > dm->eeprom_size)
        return -EINVAL;

    if (eeprom->magic != DM_EEPROM_MAGIC)
        return -EFAULT;

    for (i = 0; i < len; i += 2)
        DM9051_write_eeprom(dm, (offset + i) / 2, bytes + i);

    return 0;
}

static const struct ethtool_ops dm9051_ethtool_ops =
{
    .get_drvinfo        = dm9051_get_drvinfo,
    .get_msglevel       = dm9051_get_msglevel,
    .set_msglevel       = dm9051_set_msglevel,
    .get_link           = dm9051_get_link,
    .nway_reset         = dm9051_nway_reset,
    .get_eeprom_len     = dm9051_get_eeprom_len,
    .get_eeprom         = dm9051_get_eeprom,
    .set_eeprom         = dm9051_set_eeprom,
    .get_link_ksettings = dm9051_get_link_ksettings,
    .set_link_ksettings = dm9051_set_link_ksettings,
};


/* driver bus management functions */
#ifdef CONFIG_PM_SLEEP
static int dm9051_suspend(struct device *dev)
{
    struct dm9051_net *dm = dev_get_drvdata(dev);
    struct net_device *netdev = dm->netdev;

    if (netif_running(netdev))
    {
        netif_device_detach(netdev);
        dm9051_net_stop(netdev);
    }

    return 0;
}

static int dm9051_resume(struct device *dev)
{
    struct dm9051_net *dm = dev_get_drvdata(dev);
    struct net_device *netdev = dm->netdev;

    if (netif_running(netdev))
    {
        dm9051_net_open(netdev);
        netif_device_attach(netdev);
    }

    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(dm9051_pm_ops, dm9051_suspend, dm9051_resume);

static int dm9051_probe(struct spi_device *spi)
{
    struct net_device *ndev;
    struct dm9051_net *dm;
    int ret;
    u32 id_val, i;
    
    int reset_gpio;
    
    u16 get_spi_max_len;
    
    const unsigned char *dts_mac_addr;
    unsigned char mac_addr[6];
    int mac_len = 0;
    int mac_load_type = 0;  

    ndev = alloc_etherdev(sizeof(struct dm9051_net));
    if (!ndev)
    {
        dev_err(&spi->dev, "failed to alloc ethernet device\n");
        return -ENOMEM;
    }

    spi->bits_per_word = 8;
    dm = netdev_priv(ndev);        /* spenser: Align 32bits boundary */

    DM_MSG0("dm9051[dts]: set_mode         = 0x%02x\n", spi->mode);
    DM_MSG0("dm9051[dts]: set_max_speed_hz = %d\n", spi->max_speed_hz);
    DM_MSG0("dm9051[dts]: set_chip_select  = %d\n", spi->chip_select);

    reset_gpio = of_get_named_gpio_flags(spi->dev.of_node, "reset-gpios", 0, NULL);
    if(-EPROBE_DEFER != reset_gpio)
    {

        if (gpio_is_valid(reset_gpio))
        {
            ret = devm_gpio_request_one(&spi->dev, reset_gpio, GPIOF_OUT_INIT_LOW, "dm9051_rst_n");
            if (ret)
            {
                dev_err(&spi->dev, "reset gpio request failed\n");
                goto err_gpio;
            }
        
            dm->rst_gpio = reset_gpio;
   
            gpio_set_value(reset_gpio, 0);
            DM_MSG0("dm9051[dts]: reset gpio activate \n");
            mdelay(100);    /* wait a short time to effect reset */
            gpio_set_value(reset_gpio, 1);
            mdelay(300);    /* wait a short time to effect reset */
        }
        else
        {
            DM_MSG0("dm9051[dts]: reset gpio valid fail\n");
        }
    }
    else
    {
        DM_MSG0("dm9051[dts]: reset gpio not set \n");
    }
    
    dm->spi_max_len = sizeof(dm->spi_mosi_fifo);
    ret = of_property_read_u16(spi->dev.of_node, "mcu_spi_max_len", &get_spi_max_len);
    if(ret < 0)
    {
        DM_MSG0("dm9051[dts]: spi_max_len not set , set to def %d\n", dm->spi_max_len);
    }    
    else
    {
        if(get_spi_max_len < dm->spi_max_len) 
        {
            DM_MSG0("dm9051[dts]: mcu_spi_max_len %d \n", get_spi_max_len);
            dm->spi_max_len = get_spi_max_len;
        }
        else
        {
            DM_MSG0("dm9051[dts]: mcu_spi_max_len %d is oversize , set def %d \n", get_spi_max_len, dm->spi_max_len);
        }
    }
    
    dm->spi_max_len = dm->spi_max_len - 1;
    
    dm->netdev = ndev;
    dm->spidev = spi;

    mutex_init(&dm->lock);
    spin_lock_init(&dm->statelock);

    /* initialise pre-made spi transfer messages */
    spi_message_init(&dm->spi_msg1);
    spi_message_add_tail(&dm->spi_xfer1, &dm->spi_msg1);
    
#if DM9051_SPI_XFER2
    spi_message_init(&dm->spi_msg2);
    spi_message_add_tail(&dm->spi_xfer2_0, &dm->spi_msg2);
    spi_message_add_tail(&dm->spi_xfer2_1, &dm->spi_msg2);
    memset(dm->spi_mosi_fifo, 0x00, sizeof(dm->spi_mosi_fifo));
#endif    

    /* issue a global soft reset to reset the device. */
    dm9051_soft_reset(dm);

    /* try multiple times, DM9051 sometimes gets the read wrong */
    for (i = 0; i < 100 ; i++)
    {
        id_val  = dm9051_r_reg(dm, DM9051_VIDL);
        id_val |= (u32)dm9051_r_reg(dm, DM9051_VIDH) << 8;
        id_val |= (u32)dm9051_r_reg(dm, DM9051_PIDL) << 16;
        id_val |= (u32)dm9051_r_reg(dm, DM9051_PIDH) << 24;

        DM_MSG0("dm9051[sys]: the id_val is 0x%08x \n", id_val);
        if (id_val == DM9051_ID) break;

        mdelay(1);    /* wait a short time to effect reset */
    } //end of for

    if(id_val != DM9051_ID)
    {
        ret = -ENODEV;
        goto err_id;
    }

    DM_MSG0("dm9051[sys]: driver v%s chip revision 0x%02x mode %02x\n", 
            DRV_VERSION,
            dm9051_r_reg(dm, DM9051_CHIPR),
            dm9051_r_reg(dm, DM9051_CHIPR + 0x30));

    dm9051_w_reg(dm, DM9051_GPR, 0x00);    /* Enable PHY */
    mdelay(100);

    INIT_WORK(&dm->irq_work, dm9051_irq_work);
    
    INIT_WORK(&dm->rx_ctrl_work, dm9051_hash_table_work);
    INIT_WORK(&dm->tx_send_work, dm9051_tx_send_work);
    INIT_DELAYED_WORK(&dm->link_work, dm9051_link_work);

    /* setup mii state */
    dm->mii.dev             = ndev;
    dm->mii.phy_id          = 1,
    dm->mii.phy_id_mask     = 1;
    dm->mii.reg_num_mask    = 0x1f;
    dm->mii.mdio_read       = dm9051_phy_read;
    dm->mii.mdio_write      = dm9051_phy_write;

    /* set the default message enable */
    dm->msg_enable = netif_msg_init(msg_enable, (NETIF_MSG_DRV |
                    NETIF_MSG_PROBE | NETIF_MSG_LINK));

    skb_queue_head_init(&dm->tx_skb_q);

    ndev->ethtool_ops = &dm9051_ethtool_ops;
    SET_NETDEV_DEV(ndev, &spi->dev);

    spi_set_drvdata(spi, dm);
    ndev->if_port = IF_PORT_100BASET;
    ndev->netdev_ops = &dm9051_netdev_ops;
    ndev->irq = spi->irq;
    dm->eeprom_size = 128;

    dm->save_phy_reg0 = 0x3100;

    ret = of_irq_get(spi->dev.of_node, 0);
    if(ret < 0)
    {
        DM_MSG0("dm9051[dts]: failed to get irq, use soft poll mode\n");        
        dm->poll_work_active = 1;
        INIT_DELAYED_WORK(&dm->poll_work, dm9051_poll_work);
    }
    else
    {
	int irq_type = irq_get_trigger_type(spi->irq);
	
        dm->poll_work_active = 0;

	switch(irq_type)
	{
	    case IRQF_TRIGGER_LOW: 
	        DM_MSG0("dm9051[dts]: IRQ_TYPE_LEVEL_LOW / IRQF_TRIGGER_LOW mode\n");
	        dm->irq_pin_duty = 0xcf;
	        dm->irq_pin_type = (DM9051_INT_ACTIVE_LOW | DM9051_INT_ACTIVE_CMOS);
	        break;
	    case IRQF_TRIGGER_HIGH:
	        DM_MSG0("dm9051[dts]: IRQ_TYPE_LEVEL_HIGH / IRQF_TRIGGER_HIGH mode\n");
	        dm->irq_pin_duty = 0xcf;
	        dm->irq_pin_type = (DM9051_INT_ACTIVE_HIGH | DM9051_INT_ACTIVE_CMOS);
	        break;	        
	    case IRQF_TRIGGER_FALLING:
	        DM_MSG0("dm9051[dts]: IRQ_TYPE_EDGE_FALLING / IRQF_TRIGGER_FALLING mode\n");
	        dm->irq_pin_duty = 0x90;
	        dm->irq_pin_type = (DM9051_INT_ACTIVE_LOW | DM9051_INT_ACTIVE_CMOS);
	        break;
	    case IRQF_TRIGGER_RISING:
	        DM_MSG0("dm9051[dts]: IRQ_TYPE_EDGE_RISING / IRQF_TRIGGER_RISING mode\n");
	        dm->irq_pin_duty = 0x90;
	        dm->irq_pin_type = (DM9051_INT_ACTIVE_HIGH | DM9051_INT_ACTIVE_CMOS);
	        break;
	    default: 
	        DM_MSG0("dm9051[dts]: IRQF_not found mode\n");  
	        goto err_irq;
	        break;
	}
	
        dm9051_w_reg(dm, DM9051_INTCR, dm->irq_pin_type);
        
#if DM9051_LINUX_THREADED_IRQ
        DM_MSG0("dm9051[sys]: request_threaded_irq mode\n"); 
        ret = request_threaded_irq(spi->irq,
                                   NULL,
                                   dm9051_irq,
                                   irq_type | IRQF_ONESHOT,
                                   ndev->name,
                                   dm);
#else
        DM_MSG0("dm9051[sys]: request_irq mode\n"); 
        ret = request_irq(spi->irq,
                          dm9051_irq,
                          irq_type,
                          ndev->name,
                          dm);
#endif        

        if (ret < 0)
        {
            DM_MSG0("dm9051[dts]: failed to get irq pin\n");
            dev_err(&spi->dev, "failed to get irq\n");
            goto err_irq;
        }
        else
        {
            DM_MSG0("dm9051[dts]: get irq pin %d , use irq mode \r\n", spi->irq);
        }
        
#if (! DM9051_LINUX_THREADED_IRQ)
        disable_irq(ndev->irq);
#endif
    }

    ret = register_netdev(ndev);
    if (ret)
    {
        DM_MSG0("dm9051[sys]: failed to register network device\n");
        dev_err(&spi->dev, "failed to register network device\n");
        goto err_netdev;
    }
        
    dts_mac_addr = of_get_property(dm->spidev->dev.of_node, "local-mac-address", &mac_len);
    
    if(dts_mac_addr)
    {
        if(is_valid_ether_addr(dts_mac_addr))
        {
            ether_addr_copy(ndev->dev_addr, dts_mac_addr);
            mac_load_type = 1;
        }
    }
    
    if(!mac_load_type)
    {
        for (i = 0; i < ETH_ALEN; i++)
            mac_addr[i] = dm9051_r_reg(dm, DM9051_PAR + i);    
            
        if(is_valid_ether_addr(mac_addr))
        {
            ether_addr_copy(ndev->dev_addr, mac_addr);
            mac_load_type = 2;
        }
    }
    
    if(!mac_load_type)
    {
        if(!is_valid_ether_addr(ndev->dev_addr))
        {
            random_ether_addr(ndev->dev_addr);
            mac_load_type = 3;
        }
        else
        {
            mac_load_type = 4;
        }
    }
    
    switch(mac_load_type)
    {
        case 1: DM_MSG0("dm9051[dts]: mac load for dts\n"); break;
        case 2: DM_MSG0("dm9051[chip]: mac load for chip\n"); break;
        case 3: DM_MSG0("dm9051[sys]: mac load for random\n"); break;
        case 4: DM_MSG0("dm9051[sys]: mac load for def\n"); break;
    }
    
    for (i = 0; i < ETH_ALEN; i++)
        dm9051_w_reg(dm, DM9051_PAR + i , ndev->dev_addr[i]);

    DM_MSG0("dm9051[sys]: mac address ==> %02x:%02x:%02x:%02x:%02x:%02x\n",
        dm9051_r_reg(dm, DM9051_PAR),
        dm9051_r_reg(dm, (DM9051_PAR + 1)),
        dm9051_r_reg(dm, (DM9051_PAR + 2)),
        dm9051_r_reg(dm, (DM9051_PAR + 3)),
        dm9051_r_reg(dm, (DM9051_PAR + 4)),
        dm9051_r_reg(dm, (DM9051_PAR + 5)));

    return 0;

err_netdev:
    free_irq(ndev->irq, ndev);

err_gpio:
err_id:
err_irq:
    free_netdev(ndev);
    return ret;
}

static int dm9051_remove(struct spi_device *spi)
{
    struct dm9051_net *priv = dev_get_drvdata(&spi->dev);

    unregister_netdev(priv->netdev);
    free_irq(spi->irq, priv);
    free_netdev(priv->netdev);

    if (gpio_is_valid(priv->rst_gpio))
        gpio_set_value(priv->rst_gpio, 0);

    return 0;
}

static const struct of_device_id dm9051_match_table[] =
{
    { .compatible = "davicom,dm9051" },
    { }
};

MODULE_DEVICE_TABLE(of, dm9051_match_table);

struct spi_device_id dm9051_spi_id_table = {"dm9051", 0};

static struct spi_driver dm9051_driver =
{
    .driver =
    {
        .name = "dm9051",
        .owner = THIS_MODULE,
        .of_match_table = dm9051_match_table,
        .pm = &dm9051_pm_ops,
    },
    .probe = dm9051_probe,
    .remove = dm9051_remove,
};


static int __init dm9051_init(void)
{
    return spi_register_driver(&dm9051_driver);
}

static void __exit dm9051_exit(void)
{
    spi_unregister_driver(&dm9051_driver);
}

module_init(dm9051_init);
module_exit(dm9051_exit);

MODULE_DESCRIPTION("DM9051 Network driver");
MODULE_AUTHOR("axwdragon.com");
MODULE_LICENSE("GPL");

module_param_named(message, msg_enable, int, 0);
MODULE_PARM_DESC(message, "Message verbosity level (0=none, ff=all)");
MODULE_ALIAS("spi:dm9051");
