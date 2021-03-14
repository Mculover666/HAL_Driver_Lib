#include "w5500_port_hal.h"

/**
 * @brief   enter critical section
 * @param   none
 * @return  none
 */
static void w5500_cris_enter(void)
{
    __set_PRIMASK(1);
}

/**
 * @brief   exit critical section
 * @param   none
 * @return  none
 */
static void w5500_cris_exit(void)
{
    __set_PRIMASK(0);
}

/**
 * @brief   select chip
 * @param   none
 * @return  none
 */
static void w5500_cs_select(void)
{
    HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief   deselect chip
 * @param   none
 * @return  none
 */
static void w5500_cs_deselect(void)
{
    HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief   read byte in SPI interface
 * @param   none
 * @return  the value of the byte read
 */
static uint8_t w5500_spi_readbyte(void)
{
    uint8_t value;
    
    if (HAL_SPI_Receive(&W5500_SPI_HANDLE, &value, 1, 1000) != HAL_OK) {
        value = 0;
    }
    
    return value;
}

/**
 * @brief   write byte in SPI interface
 * @param   wb  the value to write
 * @return  none
 */
static void w5500_spi_writebyte(uint8_t wb)
{
    HAL_SPI_Transmit(&W5500_SPI_HANDLE, &wb, 1, 1000);
}

/**
 * @brief   burst read byte in SPI interface
 * @param   pBuf    pointer of data buf
 * @param   len     number of bytes to read
 * @return  none
 */
static void w5500_spi_readburst(uint8_t* pBuf, uint16_t len)
{
    if (!pBuf) {
        return;
    }
    
    HAL_SPI_Receive(&W5500_SPI_HANDLE, pBuf, len, 1000);
}

/**
 * @brief   burst write byte in SPI interface
 * @param   pBuf    pointer of data buf
 * @param   len     number of bytes to write
 * @return  none
 */
static void w5500_spi_writeburst(uint8_t* pBuf, uint16_t len)
{
    if (!pBuf) {
        return;
    }
    
    HAL_SPI_Transmit(&W5500_SPI_HANDLE, pBuf, len, 1000);
}

/**
 * @brief   hard reset
 * @param   none
 * @return  none
 */
static void w5500_hard_reset(void)
{
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

/**
 * @brief   Initializes WIZCHIP with socket buffer size
 * @param   none
 * @return  errcode
 * @retval  0   success
 * @retval  -1  fail
 */
static int w5500_chip_init(void)
{
    /* default size is 2KB */
    
    return wizchip_init(NULL, NULL);
}

/**
 * @brief   set phy config if autonego is disable
 * @param   none
 * @return  none
 */
static void w5500_phy_init(void)
{
#ifdef USE_AUTONEGO
    // no thing to do
#else
    wiz_PhyConf conf;
    
    conf.by = PHY_CONFBY_SW;
    conf.mode = PHY_MODE_MANUAL;
    conf.speed = PHY_SPEED_100;
    conf.duplex = PHY_DUPLEX_FULL;
    
    wizphy_setphyconf(&conf);
#endif
}

/**
 * @brief   initializes the network infomation
 * @param   none
 * @return  none
 */
static void w5500_network_info_init(void)
{
    wiz_NetInfo info;
    
    uint8_t mac[6] = DEFAULT_MAC_ADDR;
    uint8_t ip[4] = DEFAULT_IP_ADDR;
    uint8_t sn[4] = DEFAULT_SUB_MASK;
    uint8_t gw[4] = DEFAULT_GW_ADDR;
    uint8_t dns[4] = DEFAULT_DNS_ADDR;
    
    memcpy(info.mac, mac, 6);
    memcpy(info.ip, ip, 4);
    memcpy(info.sn, sn, 4);
    memcpy(info.gw, gw, 4);
    memcpy(info.dns, dns, 4);
    
#ifdef USE_DHCP
    info.dhcp = NETINFO_DHCP;
#else
    info.dhcp = NETINFO_STATIC;
#endif
    
    wizchip_setnetinfo(&info);
}

/**
 * @brief   read and show the network infomation
 * @param   none
 * @return  none
 */
void w5500_network_info_show(void)
{
    wiz_NetInfo info;
    
    wizchip_getnetinfo(&info);
    
    printf("w5500 network infomation:\r\n");
    printf("  -mac:%d:%d:%d:%d:%d:%d\r\n", info.mac[0], info.mac[1], info.mac[2], 
            info.mac[3], info.mac[4], info.mac[5]);
    printf("  -ip:%d.%d.%d.%d\r\n", info.ip[0], info.ip[1], info.ip[2], info.ip[3]);
    printf("  -sn:%d.%d.%d.%d\r\n", info.sn[0], info.sn[1], info.sn[2], info.sn[3]);
    printf("  -gw:%d.%d.%d.%d\r\n", info.gw[0], info.gw[1], info.gw[2], info.gw[3]);
    printf("  -dns:%d.%d.%d.%d\r\n", info.dns[0], info.dns[1], info.dns[2], info.dns[3]);
    
    if (info.dhcp == NETINFO_DHCP) {
        printf("  -dhcp_mode: dhcp\r\n");
    } else {
        printf("  -dhcp_mode: static\r\n");
    }
}

/**
 * @brief   w5500 init
 * @param   none
 * @return  errcode
 * @retval  0   success
 * @retval  -1  chip init fail
 */
int w5500_init(void)
{
    /* W5500 hard reset */
    w5500_hard_reset();
    
    /* Register spi driver function */
    reg_wizchip_cris_cbfunc(w5500_cris_enter, w5500_cris_exit);
    reg_wizchip_cs_cbfunc(w5500_cs_select, w5500_cs_deselect);
    reg_wizchip_spi_cbfunc(w5500_spi_readbyte, w5500_spi_writebyte);
    reg_wizchip_spiburst_cbfunc(w5500_spi_readburst, w5500_spi_writeburst);

    /* socket buffer size init */
    if (w5500_chip_init() != 0) {
        return -1;
    }
    
    /* phy init */
    w5500_phy_init();
    
    /* network infomation init */
    w5500_network_info_init();
    
    /* show network infomation */
    w5500_network_info_show();
    
    return 0;
}
