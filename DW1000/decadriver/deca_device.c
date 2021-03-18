/*! ------------------------------------------------------------------------------------------------------------------
 * @file    deca_device.c
 * @brief   Decawave device configuration and control functions
 *
 * @attention
 *
 * Copyright 2013 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <assert.h>
#include <stdlib.h>

#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"

// Defines for enable_clocks function
#define FORCE_SYS_XTI  0
#define ENABLE_ALL_SEQ 1
#define FORCE_SYS_PLL  2
#define READ_ACC_ON    7
#define READ_ACC_OFF   8
#define FORCE_OTP_ON   11
#define FORCE_OTP_OFF  12
#define FORCE_TX_PLL   13
#define FORCE_LDE      14

// Defines for ACK request bitmask in DATA and MAC COMMAND frame control (first byte) - Used to detect AAT bit wrongly set.
#define FCTRL_ACK_REQ_MASK 0x20
// Frame control maximum length in bytes.
#define FCTRL_LEN_MAX 2

// #define DWT_API_ERROR_CHECK     // define so API checks config input parameters

// -------------------------------------------------------------------------------------------------------------------
//
// Internal functions for controlling and configuring the device
//
// -------------------------------------------------------------------------------------------------------------------

// Enable and Configure specified clocks
void _dwt_enableclocks(int clocks) ;
// Configure the ucode (FP algorithm) parameters
void _dwt_configlde(int prf);
// Load ucode from OTP/ROM
void _dwt_loaducodefromrom(void);
// Read non-volatile memory
uint32 _dwt_otpread(uint32 address);
// Program the non-volatile memory
uint32 _dwt_otpprogword32(uint32 data, uint16 address);
// Upload the device configuration into always on memory
void _dwt_aonarrayupload(void);
// -------------------------------------------------------------------------------------------------------------------

/*!
 * Static data for DW1000 DecaWave Transceiver control
 */

// -------------------------------------------------------------------------------------------------------------------
// Structure to hold device data
typedef struct
{
    uint32      partID ;            // IC Part ID - read during initialisation
    uint32      lotID ;             // IC Lot ID - read during initialisation
    uint8       longFrames ;        // Flag in non-standard long frame mode
    uint8       otprev ;            // OTP revision number (read during initialisation)
    uint32      txFCTRL ;           // Keep TX_FCTRL register config
    uint8       init_xtrim;         // initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed)
    uint8       dblbuffon;          // Double RX buffer mode flag
    uint32      sysCFGreg ;         // Local copy of system config register
    uint16      sleep_mode;         // Used for automatic reloading of LDO tune and microcode at wake-up
    uint8       wait4resp ;         // wait4response was set with last TX start command
    dwt_cb_data_t cbData;           // Callback data structure
    dwt_cb_t    cbTxDone;           // Callback for TX confirmation event
    dwt_cb_t    cbRxOk;             // Callback for RX good frame event
    dwt_cb_t    cbRxTo;             // Callback for RX timeout events
    dwt_cb_t    cbRxErr;            // Callback for RX error events
} dwt_local_data_t ;

static dwt_local_data_t dw1000local[DWT_NUM_DW_DEV] ; // Static local device data, can be an array to support multiple DW1000 testing applications/platforms
static dwt_local_data_t *pdw1000local = dw1000local ; // Static local data structure pointer

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setlocaldataptr()
 *
 * @brief This function sets the local data structure pointer to point to the element in the local array as given by the index.
 *
 * input parameters
 * @param index    - selects the array element to point to. Must be within the array bounds, i.e. < DWT_NUM_DW_DEV
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_setlocaldataptr(unsigned int index)
{
    // Check the index is within the array bounds
    if (DWT_NUM_DW_DEV > index) // return error if index outside the array bounds
    {
        return DWT_ERROR ;
    }

    pdw1000local = &dw1000local[index];

    return DWT_SUCCESS ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_initialise()
 *
 * @brief This function initiates communications with the DW1000 transceiver
 * and reads its DEV_ID register (address 0x00) to verify the IC is one supported
 * by this software (e.g. DW1000 32-bit device ID value is 0xDECA0130).  Then it
 * does any initial once only device configurations needed for use and initialises
 * as necessary any static data items belonging to this low-level driver.
 *
 * NOTES:
 * 1.this function needs to be run before dwt_configuresleep, also the SPI frequency has to be < 3MHz
 * 2.it also reads and applies LDO tune and crystal trim values from OTP memory
 *
 * input parameters
 * @param config    -   specifies what configuration to load
 *                  DWT_LOADUCODE     0x1 - load the LDE microcode from ROM - enabled accurate RX timestamp
 *                  DWT_LOADNONE      0x0 - do not load any values from OTP memory
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
// OTP addresses definitions
#define LDOTUNE_ADDRESS (0x04)
#define PARTID_ADDRESS (0x06)
#define LOTID_ADDRESS  (0x07)
#define VBAT_ADDRESS   (0x08)
#define VTEMP_ADDRESS  (0x09)
#define XTRIM_ADDRESS  (0x1E)

int dwt_initialise(uint16 config)
{
    uint16 otp_addr = 0;
    uint32 ldo_tune = 0;

    pdw1000local->dblbuffon = 0; // Double buffer mode off by default
    pdw1000local->wait4resp = 0;
    pdw1000local->sleep_mode = 0;

    pdw1000local->cbTxDone = NULL;
    pdw1000local->cbRxOk = NULL;
    pdw1000local->cbRxTo = NULL;
    pdw1000local->cbRxErr = NULL;

    // Read and validate device ID return -1 if not recognised
    if (DWT_DEVICE_ID != dwt_readdevid()) // MP IC ONLY (i.e. DW1000) FOR THIS CODE
    {
        return DWT_ERROR ;
    }

    // Make sure the device is completely reset before starting initialisation
    dwt_softreset();

    _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is necessary to make sure the values read by _dwt_otpread are reliable

    // Configure the CPLL lock detect
    dwt_write8bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, EC_CTRL_PLLLCK);

    // Read OTP revision number
    otp_addr = _dwt_otpread(XTRIM_ADDRESS) & 0xffff;        // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
    pdw1000local->otprev = (otp_addr >> 8) & 0xff;            // OTP revision is next byte

    // Load LDO tune from OTP and kick it if there is a value actually programmed.
    ldo_tune = _dwt_otpread(LDOTUNE_ADDRESS);
    if((ldo_tune & 0xFF) != 0)
    {
        // Kick LDO tune
        dwt_write8bitoffsetreg(OTP_IF_ID, OTP_SF, OTP_SF_LDO_KICK); // Set load LDE kick bit
        pdw1000local->sleep_mode |= AON_WCFG_ONW_LLDO; // LDO tune must be kicked at wake-up
    }

    // Load Part and Lot ID from OTP
    pdw1000local->partID = _dwt_otpread(PARTID_ADDRESS);
    pdw1000local->lotID = _dwt_otpread(LOTID_ADDRESS);

    // XTAL trim value is set in OTP for DW1000 module and EVK/TREK boards but that might not be the case in a custom design
    pdw1000local->init_xtrim = otp_addr & 0x1F;
    if (!pdw1000local->init_xtrim) // A value of 0 means that the crystal has not been trimmed
    {
        pdw1000local->init_xtrim = FS_XTALT_MIDRANGE ; // Set to mid-range if no calibration value inside
    }
    // Configure XTAL trim
    dwt_setxtaltrim(pdw1000local->init_xtrim);

    // Load leading edge detect code
    if(config & DWT_LOADUCODE)
    {
        _dwt_loaducodefromrom();
        pdw1000local->sleep_mode |= AON_WCFG_ONW_LLDE; // microcode must be loaded at wake-up
    }
    else // Should disable the LDERUN enable bit in 0x36, 0x4
    {
        uint16 rega = dwt_read16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET+1) ;
        rega &= 0xFDFF ; // Clear LDERUN bit
        dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET+1, rega) ;
    }

    _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing

    // The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode.
    dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, 0x00);

    // Read system register / store local copy
    pdw1000local->sysCFGreg = dwt_read32bitreg(SYS_CFG_ID) ; // Read sysconfig register

    return DWT_SUCCESS ;

} // end dwt_initialise()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otprevision()
 *
 * @brief This is used to return the read OTP revision
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read OTP revision value
 */
uint8 dwt_otprevision(void)
{
    return pdw1000local->otprev ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setfinegraintxseq()
 *
 * @brief This function enables/disables the fine grain TX sequencing (enabled by default).
 *
 * input parameters
 * @param enable - 1 to enable fine grain TX sequencing, 0 to disable it.
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setfinegraintxseq(int enable)
{
    if (enable)
    {
        dwt_write16bitoffsetreg(PMSC_ID, PMSC_TXFINESEQ_OFFSET, PMSC_TXFINESEQ_ENABLE);
    }
    else
    {
        dwt_write16bitoffsetreg(PMSC_ID, PMSC_TXFINESEQ_OFFSET, PMSC_TXFINESEQ_DISABLE);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setlnapamode()
 *
 * @brief This is used to enable GPIO for external LNA or PA functionality - HW dependent, consult the DW1000 User Manual.
 *        This can also be used for debug as enabling TX and RX GPIOs is quite handy to monitor DW1000's activity.
 *
 * NOTE: Enabling PA functionality requires that fine grain TX sequencing is deactivated. This can be done using
 *       dwt_setfinegraintxseq().
 *
 * input parameters
 * @param lna - 1 to enable LNA functionality, 0 to disable it
 * @param pa - 1 to enable PA functionality, 0 to disable it
 *
 * output parameters
 *
 * no return value
 */
void dwt_setlnapamode(int lna, int pa)
{
    uint32 gpio_mode = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
    gpio_mode &= ~(GPIO_MSGP4_MASK | GPIO_MSGP5_MASK | GPIO_MSGP6_MASK);
    if (lna)
    {
        gpio_mode |= GPIO_PIN6_EXTRXE;
    }
    if (pa)
    {
        gpio_mode |= (GPIO_PIN5_EXTTXE | GPIO_PIN4_EXTPA);
    }
    dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, gpio_mode);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setgpiodirection()
 *
 * @brief This is used to set GPIO direction as an input (1) or output (0)
 *
 * input parameters
 * @param gpioNum    -   this is the GPIO to configure - see GxM0... GxM8 in the deca_regs.h file
 * @param direction  -   this sets the GPIO direction - see GxP0... GxP8 in the deca_regs.h file
 *
 * output parameters
 *
 * no return value
 */
void dwt_setgpiodirection(uint32 gpioNum, uint32 direction)
{
    uint8 buf[GPIO_DIR_LEN];
    uint32 command = direction | gpioNum;

    buf[0] = command & 0xff;
    buf[1] = (command >> 8) & 0xff;
    buf[2] = (command >> 16) & 0xff;

    dwt_writetodevice(GPIO_CTRL_ID, GPIO_DIR_OFFSET, GPIO_DIR_LEN, buf);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setgpiovalue()
 *
 * @brief This is used to set GPIO value as (1) or (0) only applies if the GPIO is configured as output
 *
 * input parameters
 * @param gpioNum    -   this is the GPIO to configure - see GxM0... GxM8 in the deca_regs.h file
 * @param value  -   this sets the GPIO value - see GDP0... GDP8 in the deca_regs.h file
 *
 * output parameters
 *
 * no return value
 */
void dwt_setgpiovalue(uint32 gpioNum, uint32 value)
{
    uint8 buf[GPIO_DOUT_LEN];
    uint32 command = value | gpioNum;

    buf[0] = command & 0xff;
    buf[1] = (command >> 8) & 0xff;
    buf[2] = (command >> 16) & 0xff;

    dwt_writetodevice(GPIO_CTRL_ID, GPIO_DOUT_OFFSET, GPIO_DOUT_LEN, buf);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getpartid()
 *
 * @brief This is used to return the read part ID of the device
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit part ID value as programmed in the factory
 */
uint32 dwt_getpartid(void)
{
    return pdw1000local->partID;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getlotid()
 *
 * @brief This is used to return the read lot ID of the device
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit lot ID value as programmed in the factory
 */
uint32 dwt_getlotid(void)
{
    return pdw1000local->lotID;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readdevid()
 *
 * @brief This is used to return the read device type and revision information of the DW1000 device (MP part is 0xDECA0130)
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read value which for DW1000 is 0xDECA0130
 */
uint32 dwt_readdevid(void)
{
    return dwt_read32bitoffsetreg(DEV_ID_ID,0);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuretxrf()
 *
 * @brief This function provides the API for the configuration of the TX spectrum
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type dwt_txconfig_t that holds all the configurable items.
 *
 * input parameters
 * @param config    -   pointer to the txrf configuration structure, which contains the tx rf config data
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuretxrf(dwt_txconfig_t *config)
{

    // Configure RF TX PG_DELAY
    dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGDELAY_OFFSET, config->PGdly);

    // Configure TX power
    dwt_write32bitreg(TX_POWER_ID, config->power);

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configure()
 *
 * @brief This function provides the main API for the configuration of the
 * DW1000 and this low-level driver.  The input is a pointer to the data structure
 * of type dwt_config_t that holds all the configurable items.
 * The dwt_config_t structure shows which ones are supported
 *
 * input parameters
 * @param config    -   pointer to the configuration structure, which contains the device configuration data.
 *
 * output parameters
 *
 * no return value
 */
void dwt_configure(dwt_config_t *config)
{
    uint8 nsSfd_result  = 0;
    uint8 useDWnsSFD = 0;
    uint8 chan = config->chan ;
    uint32 regval ;
    uint16 reg16 = lde_replicaCoeff[config->rxCode];
    uint8 prfIndex = config->prf - DWT_PRF_16M;
    uint8 bw = ((chan == 4) || (chan == 7)) ? 1 : 0 ; // Select wide or narrow band

#ifdef DWT_API_ERROR_CHECK
    assert(config->dataRate <= DWT_BR_6M8);
    assert(config->rxPAC <= DWT_PAC64);
    assert((chan >= 1) && (chan <= 7) && (chan != 6));
    assert(((config->prf == DWT_PRF_64M) && (config->txCode >= 9) && (config->txCode <= 24))
           || ((config->prf == DWT_PRF_16M) && (config->txCode >= 1) && (config->txCode <= 8)));
    assert(((config->prf == DWT_PRF_64M) && (config->rxCode >= 9) && (config->rxCode <= 24))
           || ((config->prf == DWT_PRF_16M) && (config->rxCode >= 1) && (config->rxCode <= 8)));
    assert((config->txPreambLength == DWT_PLEN_64) || (config->txPreambLength == DWT_PLEN_128) || (config->txPreambLength == DWT_PLEN_256)
           || (config->txPreambLength == DWT_PLEN_512) || (config->txPreambLength == DWT_PLEN_1024) || (config->txPreambLength == DWT_PLEN_1536)
           || (config->txPreambLength == DWT_PLEN_2048) || (config->txPreambLength == DWT_PLEN_4096));
    assert((config->phrMode == DWT_PHRMODE_STD) || (config->phrMode == DWT_PHRMODE_EXT));
#endif

    // For 110 kbps we need a special setup
    if(DWT_BR_110K == config->dataRate)
    {
        pdw1000local->sysCFGreg |= SYS_CFG_RXM110K ;
        reg16 >>= 3; // lde_replicaCoeff must be divided by 8
    }
    else
    {
        pdw1000local->sysCFGreg &= (~SYS_CFG_RXM110K) ;
    }

    pdw1000local->longFrames = config->phrMode ;

    pdw1000local->sysCFGreg &= ~SYS_CFG_PHR_MODE_11;
    pdw1000local->sysCFGreg |= (SYS_CFG_PHR_MODE_11 & (config->phrMode << SYS_CFG_PHR_MODE_SHFT));

    dwt_write32bitreg(SYS_CFG_ID,pdw1000local->sysCFGreg) ;
    // Set the lde_replicaCoeff
    dwt_write16bitoffsetreg(LDE_IF_ID, LDE_REPC_OFFSET, reg16) ;

    _dwt_configlde(prfIndex);

    // Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
    dwt_write32bitoffsetreg(FS_CTRL_ID, FS_PLLCFG_OFFSET, fs_pll_cfg[chan_idx[chan]]);
    dwt_write8bitoffsetreg(FS_CTRL_ID, FS_PLLTUNE_OFFSET, fs_pll_tune[chan_idx[chan]]);

    // Configure RF RX blocks (for specified channel/bandwidth)
    dwt_write8bitoffsetreg(RF_CONF_ID, RF_RXCTRLH_OFFSET, rx_config[bw]);

    // Configure RF TX blocks (for specified channel and PRF)
    // Configure RF TX control
    dwt_write32bitoffsetreg(RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config[chan_idx[chan]]);

    // Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings)
    // DTUNE0
    dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE0b_OFFSET, sftsh[config->dataRate][config->nsSFD]);

    // DTUNE1
    dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1a_OFFSET, dtune1[prfIndex]);

    if(config->dataRate == DWT_BR_110K)
    {
        dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_110K);
    }
    else
    {
        if(config->txPreambLength == DWT_PLEN_64)
        {
            dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_6M8_PRE64);
            dwt_write8bitoffsetreg(DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE64);
        }
        else
        {
            dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_850K_6M8);
            dwt_write8bitoffsetreg(DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE128PLUS);
        }
    }

    // DTUNE2
    dwt_write32bitoffsetreg(DRX_CONF_ID, DRX_TUNE2_OFFSET, digital_bb_config[prfIndex][config->rxPAC]);

    // DTUNE3 (SFD timeout)
    // Don't allow 0 - SFD timeout will always be enabled
    if(config->sfdTO == 0)
    {
        config->sfdTO = DWT_SFDTOC_DEF;
    }
    dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_SFDTOC_OFFSET, config->sfdTO);

    // Configure AGC parameters
    dwt_write32bitoffsetreg( AGC_CFG_STS_ID, 0xC, agc_config.lo32);
    dwt_write16bitoffsetreg( AGC_CFG_STS_ID, 0x4, agc_config.target[prfIndex]);

    // Set (non-standard) user SFD for improved performance,
    if(config->nsSFD)
    {
        // Write non standard (DW) SFD length
        dwt_write8bitoffsetreg(USR_SFD_ID, 0x00, dwnsSFDlen[config->dataRate]);
        nsSfd_result = 3 ;
        useDWnsSFD = 1 ;
    }
    regval =  (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | // Transmit Channel
              (CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | // Receive Channel
              (CHAN_CTRL_RXFPRF_MASK & (config->prf << CHAN_CTRL_RXFPRF_SHIFT)) | // RX PRF
              ((CHAN_CTRL_TNSSFD|CHAN_CTRL_RNSSFD) & (nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
              (CHAN_CTRL_DWSFD & (useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | // Use DW nsSFD
              (CHAN_CTRL_TX_PCOD_MASK & (config->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
              (CHAN_CTRL_RX_PCOD_MASK & (config->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)) ; // RX Preamble Code

    dwt_write32bitreg(CHAN_CTRL_ID,regval) ;

    // Set up TX Preamble Size, PRF and Data Rate
    pdw1000local->txFCTRL = ((config->txPreambLength | config->prf) << TX_FCTRL_TXPRF_SHFT) | (config->dataRate << TX_FCTRL_TXBR_SHFT);
    dwt_write32bitreg(TX_FCTRL_ID, pdw1000local->txFCTRL);

    // The SFD transmit pattern is initialised by the DW1000 upon a user TX request, but (due to an IC issue) it is not done for an auto-ACK TX. The
    // SYS_CTRL write below works around this issue, by simultaneously initiating and aborting a transmission, which correctly initialises the SFD
    // after its configuration or reconfiguration.
    // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
    dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF); // Request TX start and TRX off at the same time
} // end dwt_configure()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxantennadelay()
 *
 * @brief This API function writes the antenna delay (in time units) to RX registers
 *
 * input parameters:
 * @param rxDelay - this is the total (RX) antenna delay value, which
 *                          will be programmed into the RX register
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxantennadelay(uint16 rxDelay)
{
    // Set the RX antenna delay for auto TX timestamp adjustment
    dwt_write16bitoffsetreg(LDE_IF_ID, LDE_RXANTD_OFFSET, rxDelay);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_settxantennadelay()
 *
 * @brief This API function writes the antenna delay (in time units) to TX registers
 *
 * input parameters:
 * @param txDelay - this is the total (TX) antenna delay value, which
 *                          will be programmed into the TX delay register
 *
 * output parameters
 *
 * no return value
 */
void dwt_settxantennadelay(uint16 txDelay)
{
    // Set the TX antenna delay for auto TX timestamp adjustment
    dwt_write16bitoffsetreg(TX_ANTD_ID, TX_ANTD_OFFSET, txDelay);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetxdata()
 *
 * @brief This API function writes the supplied TX data into the DW1000's
 * TX buffer.  The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 * input parameters
 * @param txFrameLength  - This is the total frame length, including the two byte CRC.
 *                         Note: this is the length of TX message (including the 2 byte CRC) - max is 1023
 *                         standard PHR mode allows up to 127 bytes
 *                         if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                         see dwt_configure function
 * @param txFrameBytes   - Pointer to the user뭩 buffer containing the data to send.
 * @param txBufferOffset - This specifies an offset in the DW1000뭩 TX Buffer at which to start writing data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_writetxdata(uint16 txFrameLength, uint8 *txFrameBytes, uint16 txBufferOffset)
{
#ifdef DWT_API_ERROR_CHECK
    assert(txFrameLength >= 2);
    assert((pdw1000local->longFrames && (txFrameLength <= 1023)) || (txFrameLength <= 127));
    assert((txBufferOffset + txFrameLength) <= 1024);
#endif

    if ((txBufferOffset + txFrameLength) <= 1024)
    {
        // Write the data to the IC TX buffer, (-2 bytes for auto generated CRC)
        dwt_writetodevice( TX_BUFFER_ID, txBufferOffset, txFrameLength-2, txFrameBytes);
        return DWT_SUCCESS;
    }
    else
    {
        return DWT_ERROR;
    }
} // end dwt_writetxdata()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetxfctrl()
 *
 * @brief This API function configures the TX frame control register before the transmission of a frame
 *
 * input parameters:
 * @param txFrameLength - this is the length of TX message (including the 2 byte CRC) - max is 1023
 *                              NOTE: standard PHR mode allows up to 127 bytes
 *                              if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                              see dwt_configure function
 * @param txBufferOffset - the offset in the tx buffer to start writing the data
 * @param ranging - 1 if this is a ranging frame, else 0
 *
 * output parameters
 *
 * no return value
 */
void dwt_writetxfctrl(uint16 txFrameLength, uint16 txBufferOffset, int ranging)
{

#ifdef DWT_API_ERROR_CHECK
    assert((pdw1000local->longFrames && (txFrameLength <= 1023)) || (txFrameLength <= 127));
#endif

    // Write the frame length to the TX frame control register
    // pdw1000local->txFCTRL has kept configured bit rate information
    uint32 reg32 = pdw1000local->txFCTRL | txFrameLength | (txBufferOffset << TX_FCTRL_TXBOFFS_SHFT) | (ranging << TX_FCTRL_TR_SHFT);
    dwt_write32bitreg(TX_FCTRL_ID, reg32);
} // end dwt_writetxfctrl()


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxdata()
 *
 * @brief This is used to read the data from the RX buffer, from an offset location give by offset parameter
 *
 * input parameters
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param rxBufferOffset - the offset in the rx buffer from which to read the data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readrxdata(uint8 *buffer, uint16 length, uint16 rxBufferOffset)
{
    dwt_readfromdevice(RX_BUFFER_ID,rxBufferOffset,length,buffer) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readaccdata()
 *
 * @brief This is used to read the data from the Accumulator buffer, from an offset location give by offset parameter
 *
 * NOTE: Because of an internal memory access delay when reading the accumulator the first octet output is a dummy octet
 *       that should be discarded. This is true no matter what sub-index the read begins at.
 *
 * input parameters
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param accOffset - the offset in the acc buffer from which to read the data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readaccdata(uint8 *buffer, uint16 len, uint16 accOffset)
{
    // Force on the ACC clocks if we are sequenced
    _dwt_enableclocks(READ_ACC_ON);

    dwt_readfromdevice(ACC_MEM_ID,accOffset,len,buffer) ;

    _dwt_enableclocks(READ_ACC_OFF); // Revert clocks back
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readcarrierintegrator()
 *
 * @brief This is used to read the RX carrier integrator value (relating to the frequency offset of the TX node)
 *
 * NOTE: This is a 21-bit signed quantity, the function sign extends the most significant bit, which is bit #20
 *       (numbering from bit zero) to return a 32-bit signed integer value.
 *
 * input parameters - NONE
 *
 * return value - the (int32) signed carrier integrator value.
 *                A positive value means the local RX clock is running faster than the remote TX device.
 */

#define B20_SIGN_EXTEND_TEST (0x00100000UL)
#define B20_SIGN_EXTEND_MASK (0xFFF00000UL)

int32 dwt_readcarrierintegrator(void)
{
    uint32  regval = 0 ;
    int     j ;
    uint8   buffer[DRX_CARRIER_INT_LEN] ;

    /* Read 3 bytes into buffer (21-bit quantity) */

    dwt_readfromdevice(DRX_CONF_ID,DRX_CARRIER_INT_OFFSET,DRX_CARRIER_INT_LEN, buffer) ;

    for (j = 2 ; j >= 0 ; j --)  // arrenge the three bytes into an unsigned interger value
    {
        regval = (regval << 8) + buffer[j] ;
    }

    if (regval & B20_SIGN_EXTEND_TEST) regval |= B20_SIGN_EXTEND_MASK ; // sign extend bit #20 to whole word
    else regval &= DRX_CARRIER_INT_MASK ;                               // make sure upper bits are clear if not sign extending

    return (int32) regval ; // cast unsigned value to signed quantity.
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readdiagnostics()
 *
 * @brief this function reads the RX signal quality diagnostic data
 *
 * input parameters
 * @param diagnostics - diagnostic structure pointer, this will contain the diagnostic data read from the DW1000
 *
 * output parameters
 *
 * no return value
 */
void dwt_readdiagnostics(dwt_rxdiag_t *diagnostics)
{
    // Read the HW FP index
    diagnostics->firstPath = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);

    // LDE diagnostic data
    diagnostics->maxNoise = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_THRESH_OFFSET);

    // Read all 8 bytes in one SPI transaction
    dwt_readfromdevice(RX_FQUAL_ID, 0x0, 8, (uint8*)&diagnostics->stdNoise);

    diagnostics->firstPathAmp1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_AMPL1_OFFSET);

    diagnostics->rxPreamCount = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT  ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamp()
 *
 * @brief This is used to read the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read TX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readtxtimestamp(uint8 * timestamp)
{
    dwt_readfromdevice(TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, TX_TIME_TX_STAMP_LEN, timestamp) ; // Read bytes directly into buffer
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of TX timestamp
 */
uint32 dwt_readtxtimestamphi32(void)
{
    return dwt_read32bitoffsetreg(TX_TIME_ID, 1); // Offset is 1 to get the 4 upper bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamplo32()
 *
 * @brief This is used to read the low 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns low 32-bits of TX timestamp
 */
uint32 dwt_readtxtimestamplo32(void)
{
    return dwt_read32bitreg(TX_TIME_ID); // Read TX TIME as a 32-bit register to get the 4 lower bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamp()
 *
 * @brief This is used to read the RX timestamp (adjusted time of arrival)
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read RX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readrxtimestamp(uint8 * timestamp)
{
    dwt_readfromdevice(RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, RX_TIME_RX_STAMP_LEN, timestamp) ; // Get the adjusted time of arrival
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the RX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of RX timestamp
 */
uint32 dwt_readrxtimestamphi32(void)
{
    return dwt_read32bitoffsetreg(RX_TIME_ID, 1); // Offset is 1 to get the 4 upper bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamplo32()
 *
 * @brief This is used to read the low 32-bits of the RX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns low 32-bits of RX timestamp
 */
uint32 dwt_readrxtimestamplo32(void)
{
    return dwt_read32bitreg(RX_TIME_ID); // Read RX TIME as a 32-bit register to get the 4 lower bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readsystimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the system time
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of system time timestamp
 */
uint32 dwt_readsystimestamphi32(void)
{
    return dwt_read32bitoffsetreg(SYS_TIME_ID, 1); // Offset is 1 to get the 4 upper bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readsystime()
 *
 * @brief This is used to read the system time
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read system time
 *
 * output parameters
 * @param timestamp - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readsystime(uint8 * timestamp)
{
    dwt_readfromdevice(SYS_TIME_ID, SYS_TIME_OFFSET, SYS_TIME_LEN, timestamp) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetodevice()
 *
 * @brief  this function is used to write to the DW1000 device registers
 * Notes:
 *        1. Firstly we create a header (the first byte is a header byte)
 *        a. check if sub index is used, if subindexing is used - set bit-6 to 1 to signify that the sub-index address follows the register index byte
 *        b. set bit-7 (or with 0x80) for write operation
 *        c. if extended sub address index is used (i.e. if index > 127) set bit-7 of the first sub-index byte following the first header byte
 *
 *        2. Write the header followed by the data bytes to the DW1000 device
 *
 *
 * input parameters:
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param buffer        - pointer to buffer containing the 'length' bytes to be written
 *
 * output parameters
 *
 * no return value
 */
void dwt_writetodevice
(
    uint16      recordNumber,
    uint16      index,
    uint32      length,
    const uint8 *buffer
)
{
    uint8 header[3] ; // Buffer to compose header in
    int   cnt = 0; // Counter for length of header
#ifdef DWT_API_ERROR_CHECK
    assert(recordNumber <= 0x3F); // Record number is limited to 6-bits.
#endif

    // Write message header selecting WRITE operation and addresses as appropriate (this is one to three bytes long)
    if (index == 0) // For index of 0, no sub-index is required
    {
        header[cnt++] = 0x80 | recordNumber ; // Bit-7 is WRITE operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
    }
    else
    {
#ifdef DWT_API_ERROR_CHECK
        assert((index <= 0x7FFF) && ((index + length) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.
#endif
        header[cnt++] = 0xC0 | recordNumber ; // Bit-7 is WRITE operation, bit-6 one=sub-address follows, bits 5-0 is reg file id

        if (index <= 127) // For non-zero index < 127, just a single sub-index byte is required
        {
            header[cnt++] = (uint8)index ; // Bit-7 zero means no extension, bits 6-0 is index.
        }
        else
        {
            header[cnt++] = 0x80 | (uint8)(index) ; // Bit-7 one means extended index, bits 6-0 is low seven bits of index.
            header[cnt++] =  (uint8) (index >> 7) ; // 8-bit value = high eight bits of index.
        }
    }

    // Write it to the SPI
    writetospi(cnt,header,length,buffer);
} // end dwt_writetodevice()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readfromdevice()
 *
 * @brief  this function is used to read from the DW1000 device registers
 * Notes:
 *        1. Firstly we create a header (the first byte is a header byte)
 *        a. check if sub index is used, if subindexing is used - set bit-6 to 1 to signify that the sub-index address follows the register index byte
 *        b. set bit-7 (or with 0x80) for write operation
 *        c. if extended sub address index is used (i.e. if index > 127) set bit-7 of the first sub-index byte following the first header byte
 *
 *        2. Write the header followed by the data bytes to the DW1000 device
 *        3. Store the read data in the input buffer
 *
 * input parameters:
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being read
 * @param buffer        - pointer to buffer in which to return the read data.
 *
 * output parameters
 *
 * no return value
 */
void dwt_readfromdevice
(
    uint16  recordNumber,
    uint16  index,
    uint32  length,
    uint8   *buffer
)
{
    uint8 header[3] ; // Buffer to compose header in
    int   cnt = 0; // Counter for length of header
#ifdef DWT_API_ERROR_CHECK
    assert(recordNumber <= 0x3F); // Record number is limited to 6-bits.
#endif

    // Write message header selecting READ operation and addresses as appropriate (this is one to three bytes long)
    if (index == 0) // For index of 0, no sub-index is required
    {
        header[cnt++] = (uint8) recordNumber ; // Bit-7 zero is READ operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
    }
    else
    {
#ifdef DWT_API_ERROR_CHECK
        assert((index <= 0x7FFF) && ((index + length) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.
#endif
        header[cnt++] = (uint8)(0x40 | recordNumber) ; // Bit-7 zero is READ operation, bit-6 one=sub-address follows, bits 5-0 is reg file id

        if (index <= 127) // For non-zero index < 127, just a single sub-index byte is required
        {
            header[cnt++] = (uint8) index ; // Bit-7 zero means no extension, bits 6-0 is index.
        }
        else
        {
            header[cnt++] = 0x80 | (uint8)(index) ; // Bit-7 one means extended index, bits 6-0 is low seven bits of index.
            header[cnt++] =  (uint8) (index >> 7) ; // 8-bit value = high eight bits of index.
        }
    }

    // Do the read from the SPI
    readfromspi(cnt, header, length, buffer);  // result is stored in the buffer
} // end dwt_readfromdevice()



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_read32bitoffsetreg()
 *
 * @brief  this function is used to read 32-bit value from the DW1000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 32 bit register value
 */
uint32 dwt_read32bitoffsetreg(int regFileID,int regOffset)
{
    uint32  regval = 0 ;
    int     j ;
    uint8   buffer[4] ;

    dwt_readfromdevice(regFileID,regOffset,4,buffer); // Read 4 bytes (32-bits) register into buffer

    for (j = 3 ; j >= 0 ; j --)
    {
        regval = (regval << 8) + buffer[j] ;
    }
    return regval ;

} // end dwt_read32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_read16bitoffsetreg()
 *
 * @brief  this function is used to read 16-bit value from the DW1000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 16 bit register value
 */
uint16 dwt_read16bitoffsetreg(int regFileID,int regOffset)
{
    uint16  regval = 0 ;
    uint8   buffer[2] ;

    dwt_readfromdevice(regFileID,regOffset,2,buffer); // Read 2 bytes (16-bits) register into buffer

    regval = (buffer[1] << 8) + buffer[0] ;
    return regval ;

} // end dwt_read16bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_read8bitoffsetreg()
 *
 * @brief  this function is used to read an 8-bit value from the DW1000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 8-bit register value
 */
uint8 dwt_read8bitoffsetreg(int regFileID, int regOffset)
{
    uint8 regval;

    dwt_readfromdevice(regFileID, regOffset, 1, &regval);

    return regval ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_write8bitoffsetreg()
 *
 * @brief  this function is used to write an 8-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
void dwt_write8bitoffsetreg(int regFileID, int regOffset, uint8 regval)
{
    dwt_writetodevice(regFileID, regOffset, 1, &regval);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_write16bitoffsetreg()
 *
 * @brief  this function is used to write 16-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
void dwt_write16bitoffsetreg(int regFileID,int regOffset,uint16 regval)
{
    uint8   buffer[2] ;

    buffer[0] = regval & 0xFF;
    buffer[1] = regval >> 8 ;

    dwt_writetodevice(regFileID,regOffset,2,buffer);
} // end dwt_write16bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_write32bitoffsetreg()
 *
 * @brief  this function is used to write 32-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
void dwt_write32bitoffsetreg(int regFileID,int regOffset,uint32 regval)
{
    int     j ;
    uint8   buffer[4] ;

    for ( j = 0 ; j < 4 ; j++ )
    {
        buffer[j] = regval & 0xff ;
        regval >>= 8 ;
    }

    dwt_writetodevice(regFileID,regOffset,4,buffer);
} // end dwt_write32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_enableframefilter()
 *
 * @brief This is used to enable the frame filtering - (the default option is to
 * accept any data and ACK frames with correct destination address
 *
 * input parameters
 * @param - bitmask - enables/disables the frame filtering options according to
 *      DWT_FF_NOTYPE_EN        0x000   no frame types allowed
 *      DWT_FF_COORD_EN         0x002   behave as coordinator (can receive frames with no destination address (PAN ID has to match))
 *      DWT_FF_BEACON_EN        0x004   beacon frames allowed
 *      DWT_FF_DATA_EN          0x008   data frames allowed
 *      DWT_FF_ACK_EN           0x010   ack frames allowed
 *      DWT_FF_MAC_EN           0x020   mac control frames allowed
 *      DWT_FF_RSVD_EN          0x040   reserved frame types allowed
 *
 * output parameters
 *
 * no return value
 */
void dwt_enableframefilter(uint16 enable)
{
    uint32 sysconfig = SYS_CFG_MASK & dwt_read32bitreg(SYS_CFG_ID) ; // Read sysconfig register

    if(enable)
    {
        // Enable frame filtering and configure frame types
        sysconfig &= ~(SYS_CFG_FF_ALL_EN); // Clear all
        sysconfig |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
    }
    else
    {
        sysconfig &= ~(SYS_CFG_FFE);
    }

    pdw1000local->sysCFGreg = sysconfig ;
    dwt_write32bitreg(SYS_CFG_ID,sysconfig) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setpanid()
 *
 * @brief This is used to set the PAN ID
 *
 * input parameters
 * @param panID - this is the PAN ID
 *
 * output parameters
 *
 * no return value
 */
void dwt_setpanid(uint16 panID)
{
    // PAN ID is high 16 bits of register
    dwt_write16bitoffsetreg(PANADR_ID, PANADR_PAN_ID_OFFSET, panID);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setaddress16()
 *
 * @brief This is used to set 16-bit (short) address
 *
 * input parameters
 * @param shortAddress - this sets the 16 bit short address
 *
 * output parameters
 *
 * no return value
 */
void dwt_setaddress16(uint16 shortAddress)
{
    // Short address into low 16 bits
    dwt_write16bitoffsetreg(PANADR_ID, PANADR_SHORT_ADDR_OFFSET, shortAddress);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_seteui()
 *
 * @brief This is used to set the EUI 64-bit (long) address
 *
 * input parameters
 * @param eui64 - this is the pointer to a buffer that contains the 64bit address
 *
 * output parameters
 *
 * no return value
 */
void dwt_seteui(uint8 *eui64)
{
    dwt_writetodevice(EUI_64_ID, EUI_64_OFFSET, EUI_64_LEN, eui64);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_geteui()
 *
 * @brief This is used to get the EUI 64-bit from the DW1000
 *
 * input parameters
 * @param eui64 - this is the pointer to a buffer that will contain the read 64-bit EUI value
 *
 * output parameters
 *
 * no return value
 */
void dwt_geteui(uint8 *eui64)
{
    dwt_readfromdevice(EUI_64_ID, EUI_64_OFFSET, EUI_64_LEN, eui64);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otpread()
 *
 * @brief This is used to read the OTP data from given address into provided array
 *
 * input parameters
 * @param address - this is the OTP address to read from
 * @param array - this is the pointer to the array into which to read the data
 * @param length - this is the number of 32 bit words to read (array needs to be at least this length)
 *
 * output parameters
 *
 * no return value
 */
void dwt_otpread(uint32 address, uint32 *array, uint8 length)
{
    int i;

    _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: Set system clock to XTAL - this is necessary to make sure the values read by _dwt_otpread are reliable

    for(i=0; i<length; i++)
    {
        array[i] = _dwt_otpread(address + i) ;
    }

    _dwt_enableclocks(ENABLE_ALL_SEQ); // Restore system clock to PLL

    return ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpread()
 *
 * @brief function to read the OTP memory. Ensure that MR,MRa,MRb are reset to 0.
 *
 * input parameters
 * @param address - address to read at
 *
 * output parameters
 *
 * returns the 32bit of read data
 */
uint32 _dwt_otpread(uint32 address)
{
    uint32 ret_data;

    // Write the address
    dwt_write16bitoffsetreg(OTP_IF_ID, OTP_ADDR, address);

    // Perform OTP Read - Manual read mode has to be set
    dwt_write8bitoffsetreg(OTP_IF_ID, OTP_CTRL, OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN);
    dwt_write8bitoffsetreg(OTP_IF_ID, OTP_CTRL, 0x00); // OTPREAD is self clearing but OTPRDEN is not

    // Read read data, available 40ns after rising edge of OTP_READ
    ret_data = dwt_read32bitoffsetreg(OTP_IF_ID, OTP_RDAT);

    // Return the 32bit of read data
    return ret_data;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpsetmrregs()
 *
 * @brief Configure the MR registers for initial programming (enable charge pump).
 * Read margin is used to stress the read back from the
 * programmed bit. In normal operation this is relaxed.
 *
 * input parameters
 * @param mode - "0" : Reset all to 0x0:           MRA=0x0000, MRB=0x0000, MR=0x0000
 *               "1" : Set for inital programming: MRA=0x9220, MRB=0x000E, MR=0x1024
 *               "2" : Set for soak programming:   MRA=0x9220, MRB=0x0003, MR=0x1824
 *               "3" : High Vpp:                   MRA=0x9220, MRB=0x004E, MR=0x1824
 *               "4" : Low Read Margin:            MRA=0x0000, MRB=0x0003, MR=0x0000
 *               "5" : Array Clean:                MRA=0x0049, MRB=0x0003, MR=0x0024
 *               "4" : Very Low Read Margin:       MRA=0x0000, MRB=0x0003, MR=0x0000
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
uint32 _dwt_otpsetmrregs(int mode)
{
    uint8 rd_buf[4];
    uint8 wr_buf[4];
    uint32 mra=0,mrb=0,mr=0;

    // PROGRAMME MRA
    // Set MRA, MODE_SEL
    wr_buf[0] = 0x03;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL+1,1,wr_buf);

    // Load data
    switch(mode&0x0f) {
    case 0x0 :
        mr =0x0000;
        mra=0x0000;
        mrb=0x0000;
        break;
    case 0x1 :
        mr =0x1024;
        mra=0x9220; // Enable CPP mon
        mrb=0x000e;
        break;
    case 0x2 :
        mr =0x1824;
        mra=0x9220;
        mrb=0x0003;
        break;
    case 0x3 :
        mr =0x1824;
        mra=0x9220;
        mrb=0x004e;
        break;
    case 0x4 :
        mr =0x0000;
        mra=0x0000;
        mrb=0x0003;
        break;
    case 0x5 :
        mr =0x0024;
        mra=0x0000;
        mrb=0x0003;
        break;
    default :
        return DWT_ERROR;
    }

    wr_buf[0] = mra & 0x00ff;
    wr_buf[1] = (mra & 0xff00)>>8;
    dwt_writetodevice(OTP_IF_ID, OTP_WDAT,2,wr_buf);


    // Set WRITE_MR
    wr_buf[0] = 0x08;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);

    // Wait?

    // Set Clear Mode sel
    wr_buf[0] = 0x02;
    dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);

    // Set AUX update, write MR
    wr_buf[0] = 0x88;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
    // Clear write MR
    wr_buf[0] = 0x80;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
    // Clear AUX update
    wr_buf[0] = 0x00;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);

    ///////////////////////////////////////////
    // PROGRAM MRB
    // Set SLOW, MRB, MODE_SEL
    wr_buf[0] = 0x05;
    dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);

    wr_buf[0] = mrb & 0x00ff;
    wr_buf[1] = (mrb & 0xff00)>>8;
    dwt_writetodevice(OTP_IF_ID, OTP_WDAT,2,wr_buf);

    // Set WRITE_MR
    wr_buf[0] = 0x08;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);

    // Wait?

    // Set Clear Mode sel
    wr_buf[0] = 0x04;
    dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);

    // Set AUX update, write MR
    wr_buf[0] = 0x88;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
    // Clear write MR
    wr_buf[0] = 0x80;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
    // Clear AUX update
    wr_buf[0] = 0x00;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);

    ///////////////////////////////////////////
    // PROGRAM MR
    // Set SLOW, MODE_SEL
    wr_buf[0] = 0x01;
    dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
    // Load data

    wr_buf[0] = mr & 0x00ff;
    wr_buf[1] = (mr & 0xff00)>>8;
    dwt_writetodevice(OTP_IF_ID, OTP_WDAT,2,wr_buf);

    // Set WRITE_MR
    wr_buf[0] = 0x08;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);

    // Wait?
    deca_sleep(10);
    // Set Clear Mode sel
    wr_buf[0] = 0x00;
    dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);

    // Read confirm mode writes.
    // Set man override, MRA_SEL
    wr_buf[0] = OTP_CTRL_OTPRDEN;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);
    wr_buf[0] = 0x02;
    dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
    // MRB_SEL
    wr_buf[0] = 0x04;
    dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
    deca_sleep(100);

    // Clear mode sel
    wr_buf[0] = 0x00;
    dwt_writetodevice(OTP_IF_ID,OTP_CTRL+1,1,wr_buf);
    // Clear MAN_OVERRIDE
    wr_buf[0] = 0x00;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL,1,wr_buf);

    deca_sleep(10);

    if (((mode&0x0f) == 0x1)||((mode&0x0f) == 0x2))
    {
        // Read status register
        dwt_readfromdevice(OTP_IF_ID, OTP_STAT,1,rd_buf);
    }

    return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpprogword32()
 *
 * @brief function to program the OTP memory. Ensure that MR,MRa,MRb are reset to 0.
 * VNM Charge pump needs to be enabled (see _dwt_otpsetmrregs)
 * Note the address is only 11 bits long.
 *
 * input parameters
 * @param address - address to read at
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
uint32 _dwt_otpprogword32(uint32 data, uint16 address)
{
    uint8 rd_buf[1];
    uint8 wr_buf[4];
    uint8 otp_done;

    // Read status register
    dwt_readfromdevice(OTP_IF_ID, OTP_STAT, 1, rd_buf);

    if((rd_buf[0] & 0x02) != 0x02)
    {
        return DWT_ERROR;
    }

    // Write the data
    wr_buf[3] = (data>>24) & 0xff;
    wr_buf[2] = (data>>16) & 0xff;
    wr_buf[1] = (data>>8) & 0xff;
    wr_buf[0] = data & 0xff;
    dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 4, wr_buf);

    // Write the address [10:0]
    wr_buf[1] = (address>>8) & 0x07;
    wr_buf[0] = address & 0xff;
    dwt_writetodevice(OTP_IF_ID, OTP_ADDR, 2, wr_buf);

    // Enable Sequenced programming
    wr_buf[0] = OTP_CTRL_OTPPROG;
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
    wr_buf[0] = 0x00; // And clear
    dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

    // WAIT for status to flag PRGM OK..
    otp_done = 0;
    while(otp_done == 0)
    {
        deca_sleep(1);
        dwt_readfromdevice(OTP_IF_ID, OTP_STAT, 1, rd_buf);

        if((rd_buf[0] & 0x01) == 0x01)
        {
            otp_done = 1;
        }
    }

    return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otpwriteandverify()
 *
 * @brief This is used to program 32-bit value into the DW1000 OTP memory.
 *
 * input parameters
 * @param value - this is the 32-bit value to be programmed into OTP
 * @param address - this is the 16-bit OTP address into which the 32-bit value is programmed
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_otpwriteandverify(uint32 value, uint16 address)
{
    int prog_ok = DWT_SUCCESS;
    int retry = 0;
    // Firstly set the system clock to crystal
    _dwt_enableclocks(FORCE_SYS_XTI); //set system clock to XTI

    //
    //!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!
    //Set the supply to 3.7V
    //

    _dwt_otpsetmrregs(1); // Set mode for programming

    // For each value to program - the readback/check is done couple of times to verify it has programmed successfully
    while(1)
    {
        _dwt_otpprogword32(value, address);

        if(_dwt_otpread(address) == value)
        {
            break;
        }
        retry++;
        if(retry==5)
        {
            break;
        }
    }

    // Even if the above does not exit before retry reaches 5, the programming has probably been successful

    _dwt_otpsetmrregs(4); // Set mode for reading

    if(_dwt_otpread(address) != value) // If this does not pass please check voltage supply on VDDIO
    {
        prog_ok = DWT_ERROR;
    }

    _dwt_otpsetmrregs(0); // Setting OTP mode register for low RM read - resetting the device would be alternative

    return prog_ok;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_aonconfigupload()
 *
 * @brief This function uploads always on (AON) configuration, as set in the AON_CFG0_OFFSET register.
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dwt_aonconfigupload(void)
{
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_UPL_CFG);
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, 0x00); // Clear the register
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_aonarrayupload()
 *
 * @brief This function uploads always on (AON) data array and configuration. Thus if this function is used, then _dwt_aonconfigupload
 * is not necessary. The DW1000 will go so SLEEP straight after this if the DWT_SLP_EN has been set.
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dwt_aonarrayupload(void)
{
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, 0x00); // Clear the register
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_entersleep()
 *
 * @brief This function puts the device into deep sleep or sleep. dwt_configuresleep() should be called first
 * to configure the sleep and on-wake/wake-up parameters
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_entersleep(void)
{
    // Copy config to AON - upload the new configuration
    _dwt_aonarrayupload();
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuresleepcnt()
 *
 * @brief sets the sleep counter to new value, this function programs the high 16-bits of the 28-bit counter
 *
 * NOTE: this function needs to be run before dwt_configuresleep, also the SPI frequency has to be < 3MHz
 *
 * input parameters
 * @param sleepcnt - this it value of the sleep counter to program
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuresleepcnt(uint16 sleepcnt)
{
    // Force system clock to crystal
    _dwt_enableclocks(FORCE_SYS_XTI);

    // Reset sleep configuration to make sure we don't accidentally go to sleep
    dwt_write8bitoffsetreg(AON_ID, AON_CFG0_OFFSET, 0x00); // NB: this write change the default LPCLKDIVA value which is not used anyway.
    dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, 0x00);

    // Disable the sleep counter
    _dwt_aonconfigupload();

    // Set new value
    dwt_write16bitoffsetreg(AON_ID, AON_CFG0_OFFSET + AON_CFG0_SLEEP_TIM_OFFSET, sleepcnt);
    _dwt_aonconfigupload();

    // Enable the sleep counter
    dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, AON_CFG1_SLEEP_CEN);
    _dwt_aonconfigupload();

    // Put system PLL back on
    _dwt_enableclocks(ENABLE_ALL_SEQ);
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_calibratesleepcnt()
 *
 * @brief calibrates the local oscillator as its frequency can vary between 7 and 13kHz depending on temp and voltage
 *
 * NOTE: this function needs to be run before dwt_configuresleepcnt, so that we know what the counter units are
 *
 * input parameters
 *
 * output parameters
 *
 * returns the number of XTAL/2 cycles per low-power oscillator cycle. LP OSC frequency = 19.2 MHz/return value
 */
uint16 dwt_calibratesleepcnt(void)
{
    uint16 result;

    // Enable calibration of the sleep counter
    dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, AON_CFG1_LPOSC_CAL);
    _dwt_aonconfigupload();

    // Disable calibration of the sleep counter
    dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, 0x00);
    _dwt_aonconfigupload();

    // Force system clock to crystal
    _dwt_enableclocks(FORCE_SYS_XTI);

    deca_sleep(1);

    // Read the number of XTAL/2 cycles one LP oscillator cycle took.
    // Set up address - Read upper byte first
    dwt_write8bitoffsetreg(AON_ID, AON_ADDR_OFFSET, AON_ADDR_LPOSC_CAL_1);

    // Enable manual override
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_DCA_ENAB);

    // Read confirm data that was written
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_DCA_ENAB | AON_CTRL_DCA_READ);

    // Read back byte from AON
    result = dwt_read8bitoffsetreg(AON_ID, AON_RDAT_OFFSET);
    result <<= 8;

    // Set up address - Read lower byte
    dwt_write8bitoffsetreg(AON_ID, AON_ADDR_OFFSET, AON_ADDR_LPOSC_CAL_0);

    // Enable manual override
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_DCA_ENAB);

    // Read confirm data that was written
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_DCA_ENAB | AON_CTRL_DCA_READ);

    // Read back byte from AON
    result |= dwt_read8bitoffsetreg(AON_ID, AON_RDAT_OFFSET);

    // Disable manual override
    dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, 0x00);

    // Put system PLL back on
    _dwt_enableclocks(ENABLE_ALL_SEQ);

    // Returns the number of XTAL/2 cycles per one LP OSC cycle
    // This can be converted into LP OSC frequency by 19.2 MHz/result
    return result;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuresleep()
 *
 * @brief configures the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 * i.e. before entering the sleep, the device should be programmed for TX or RX, then upon "waking up" the TX/RX settings
 * will be preserved and the device can immediately perform the desired action TX/RX
 *
 * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame
 *
 *
 *      mode: the array and LDE code (OTP/ROM) and LDO tune, and set sleep persist
 *      DWT_PRESRV_SLEEP 0x0100 - preserve sleep
 *      DWT_LOADOPSET    0x0080 - load operating parameter set on wakeup
 *      DWT_CONFIG       0x0040 - download the AON array into the HIF (configuration download)
 *      DWT_LOADEUI      0x0008
 *      DWT_GOTORX       0x0002
 *      DWT_TANDV        0x0001
 *
 *      wake: wake up parameters
 *      DWT_XTAL_EN      0x10 - keep XTAL running during sleep
 *      DWT_WAKE_SLPCNT  0x8 - wake up after sleep count
 *      DWT_WAKE_CS      0x4 - wake up on chip select
 *      DWT_WAKE_WK      0x2 - wake up on WAKEUP PIN
 *      DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
 *
 * input parameters
 * @param mode - config on-wake parameters
 * @param wake - config wake up parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuresleep(uint16 mode, uint8 wake)
{
    // Add predefined sleep settings before writing the mode
    mode |= pdw1000local->sleep_mode;
    dwt_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, mode);

    dwt_write8bitoffsetreg(AON_ID, AON_CFG0_OFFSET, wake);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_entersleepaftertx(int enable)
 *
 * @brief sets the auto TX to sleep bit. This means that after a frame
 * transmission the device will enter deep sleep mode. The dwt_configuresleep() function
 * needs to be called before this to configure the on-wake settings
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * input parameters
 * @param enable - 1 to configure the device to enter deep sleep after TX, 0 - disables the configuration
 *
 * output parameters
 *
 * no return value
 */
void dwt_entersleepaftertx(int enable)
{
    uint32 reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
    // Set the auto TX -> sleep bit
    if(enable)
    {
        reg |= PMSC_CTRL1_ATXSLP;
    }
    else
    {
        reg &= ~(PMSC_CTRL1_ATXSLP);
    }
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, reg);
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_spicswakeup()
 *
 * @brief wake up the device from sleep mode using the SPI read,
 * the device will wake up on chip select line going low if the line is held low for at least 500us.
 * To define the length depending on the time one wants to hold
 * the chip select line low, use the following formula:
 *
 *      length (bytes) = time (s) * byte_rate (Hz)
 *
 * where fastest byte_rate is spi_rate (Hz) / 8 if the SPI is sending the bytes back-to-back.
 * To save time and power, a system designer could determine byte_rate value more precisely.
 *
 * NOTE: Alternatively the device can be waken up with WAKE_UP pin if configured for that operation
 *
 * input parameters
 * @param buff   - this is a pointer to the dummy buffer which will be used in the SPI read transaction used for the WAKE UP of the device
 * @param length - this is the length of the dummy buffer
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_spicswakeup(uint8 *buff, uint16 length)
{
    if(dwt_readdevid() != DWT_DEVICE_ID) // Device was in deep sleep (the first read fails)
    {
        // Need to keep chip select line low for at least 500us
        dwt_readfromdevice(0x0, 0x0, length, buff); // Do a long read to wake up the chip (hold the chip select low)

        // Need 5ms for XTAL to start and stabilise (could wait for PLL lock IRQ status bit !!!)
        // NOTE: Polling of the STATUS register is not possible unless frequency is < 3MHz
        deca_sleep(5);
    }
    else
    {
        return DWT_SUCCESS;
    }
    // DEBUG - check if still in sleep mode
    if(dwt_readdevid() != DWT_DEVICE_ID)
    {
        return DWT_ERROR;
    }

    return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_configlde()
 *
 * @brief configure LDE algorithm parameters
 *
 * input parameters
 * @param prf   -   this is the PRF index (0 or 1) 0 corresponds to 16 and 1 to 64 PRF
 *
 * output parameters
 *
 * no return value
 */
void _dwt_configlde(int prfIndex)
{
    dwt_write8bitoffsetreg(LDE_IF_ID, LDE_CFG1_OFFSET, LDE_PARAM1); // 8-bit configuration register

    if(prfIndex)
    {
        dwt_write16bitoffsetreg( LDE_IF_ID, LDE_CFG2_OFFSET, (uint16) LDE_PARAM3_64); // 16-bit LDE configuration tuning register
    }
    else
    {
        dwt_write16bitoffsetreg( LDE_IF_ID, LDE_CFG2_OFFSET, (uint16) LDE_PARAM3_16);
    }
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_loaducodefromrom()
 *
 * @brief  load ucode from OTP MEMORY or ROM
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dwt_loaducodefromrom(void)
{
    // Set up clocks
    _dwt_enableclocks(FORCE_LDE);

    // Kick off the LDE load
    dwt_write16bitoffsetreg(OTP_IF_ID, OTP_CTRL, OTP_CTRL_LDELOAD); // Set load LDE kick bit

    deca_sleep(1); // Allow time for code to upload (should take up to 120 us)

    // Default clocks (ENABLE_ALL_SEQ)
    _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_loadopsettabfromotp()
 *
 * @brief This is used to select which Operational Parameter Set table to load from OTP memory
 *
 * input parameters
 * @param ops_sel - Operational Parameter Set table to load:
 *                  DWT_OPSET_64LEN = 0x0 - load the operational parameter set table for 64 length preamble configuration
 *                  DWT_OPSET_TIGHT = 0x1 - load the operational parameter set table for tight xtal offsets (<1ppm)
 *                  DWT_OPSET_DEFLT = 0x2 - load the default operational parameter set table (this is loaded from reset)
 *
 * output parameters
 *
 * no return value
 */
void dwt_loadopsettabfromotp(uint8 ops_sel)
{
    uint16 reg = ((ops_sel << OTP_SF_OPS_SEL_SHFT) & OTP_SF_OPS_SEL_MASK) | OTP_SF_OPS_KICK; // Select defined OPS table and trigger its loading

    // Set up clocks
    _dwt_enableclocks(FORCE_LDE);

    dwt_write16bitoffsetreg(OTP_IF_ID, OTP_SF, reg);

    // Default clocks (ENABLE_ALL_SEQ)
    _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setsmarttxpower()
 *
 * @brief This call enables or disables the smart TX power feature.
 *
 * input parameters
 * @param enable - this enables or disables the TX smart power (1 = enable, 0 = disable)
 *
 * output parameters
 *
 * no return value
 */
void dwt_setsmarttxpower(int enable)
{
    // Config system register
    pdw1000local->sysCFGreg = dwt_read32bitreg(SYS_CFG_ID) ; // Read sysconfig register

    // Disable smart power configuration
    if(enable)
    {
        pdw1000local->sysCFGreg &= ~(SYS_CFG_DIS_STXP) ;
    }
    else
    {
        pdw1000local->sysCFGreg |= SYS_CFG_DIS_STXP ;
    }

    dwt_write32bitreg(SYS_CFG_ID,pdw1000local->sysCFGreg) ;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_enableautoack()
 *
 * @brief This call enables the auto-ACK feature. If the responseDelayTime (parameter) is 0, the ACK will be sent a.s.a.p.
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well
 *
 * input parameters
 * @param responseDelayTime - if non-zero the ACK is sent after this delay, max is 255.
 *
 * output parameters
 *
 * no return value
 */
void dwt_enableautoack(uint8 responseDelayTime)
{
    // Set auto ACK reply delay
    dwt_write8bitoffsetreg(ACK_RESP_T_ID, ACK_RESP_T_ACK_TIM_OFFSET, responseDelayTime); // In symbols
    // Enable auto ACK
    pdw1000local->sysCFGreg |= SYS_CFG_AUTOACK;
    dwt_write32bitreg(SYS_CFG_ID,pdw1000local->sysCFGreg) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setdblrxbuffmode()
 *
 * @brief This call enables the double receive buffer mode
 *
 * input parameters
 * @param enable - 1 to enable, 0 to disable the double buffer mode
 *
 * output parameters
 *
 * no return value
 */
void dwt_setdblrxbuffmode(int enable)
{
    if(enable)
    {
        // Enable double RX buffer mode
        pdw1000local->sysCFGreg &= ~SYS_CFG_DIS_DRXB;
        pdw1000local->dblbuffon = 1;
    }
    else
    {
        // Disable double RX buffer mode
        pdw1000local->sysCFGreg |= SYS_CFG_DIS_DRXB;
        pdw1000local->dblbuffon = 0;
    }

    dwt_write32bitreg(SYS_CFG_ID,pdw1000local->sysCFGreg) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxaftertxdelay()
 *
 * @brief This sets the receiver turn on delay time after a transmission of a frame
 *
 * input parameters
 * @param rxDelayTime - (20 bits) - the delay is in UWB microseconds
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxaftertxdelay(uint32 rxDelayTime)
{
    uint32 val = dwt_read32bitreg(ACK_RESP_T_ID) ; // Read ACK_RESP_T_ID register

    val &= ~(ACK_RESP_T_W4R_TIM_MASK) ; // Clear the timer (19:0)

    val |= (rxDelayTime & ACK_RESP_T_W4R_TIM_MASK) ; // In UWB microseconds (e.g. turn the receiver on 20uus after TX)

    dwt_write32bitreg(ACK_RESP_T_ID, val) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setcallbacks()
 *
 * @brief This function is used to register the different callbacks called when one of the corresponding event occurs.
 *
 * NOTE: Callbacks can be undefined (set to NULL). In this case, dwt_isr() will process the event as usual but the 'null'
 * callback will not be called.
 *
 * input parameters
 * @param cbTxDone - the pointer to the TX confirmation event callback function
 * @param cbRxOk - the pointer to the RX good frame event callback function
 * @param cbRxTo - the pointer to the RX timeout events callback function
 * @param cbRxErr - the pointer to the RX error events callback function
 *
 * output parameters
 *
 * no return value
 */
void dwt_setcallbacks(dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr)
{
    pdw1000local->cbTxDone = cbTxDone;
    pdw1000local->cbRxOk = cbRxOk;
    pdw1000local->cbRxTo = cbRxTo;
    pdw1000local->cbRxErr = cbRxErr;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_checkirq()
 *
 * @brief This function checks if the IRQ line is active - this is used instead of interrupt handler
 *
 * input parameters
 *
 * output parameters
 *
 * return value is 1 if the IRQS bit is set and 0 otherwise
 */
uint8 dwt_checkirq(void)
{
    return (dwt_read8bitoffsetreg(SYS_STATUS_ID, SYS_STATUS_OFFSET) & SYS_STATUS_IRQS); // Reading the lower byte only is enough for this operation
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_isr()
 *
 * @brief This is the DW1000's general Interrupt Service Routine. It will process/report the following events:
 *          - RXFCG (through cbRxOk callback)
 *          - TXFRS (through cbTxDone callback)
 *          - RXRFTO/RXPTO (through cbRxTo callback)
 *          - RXPHE/RXFCE/RXRFSL/RXSFDTO/AFFREJ/LDEERR (through cbRxTo cbRxErr)
 *        For all events, corresponding interrupts are cleared and necessary resets are performed. In addition, in the RXFCG case,
 *        received frame information and frame control are read before calling the callback. If double buffering is activated, it
 *        will also toggle between reception buffers once the reception callback processing has ended.
 *
 *        /!\ This version of the ISR supports double buffering but does not support automatic RX re-enabling!
 *
 * NOTE:  In PC based system using (Cheetah or ARM) USB to SPI converter there can be no interrupts, however we still need something
 *        to take the place of it and operate in a polled way. In an embedded system this function should be configured to be triggered
 *        on any of the interrupts described above.

 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_isr(void)
{
    uint32 status = pdw1000local->cbData.status = dwt_read32bitreg(SYS_STATUS_ID); // Read status register low 32bits

    // Handle RX good frame event
    if(status & SYS_STATUS_RXFCG)
    {
        uint16 finfo16;
        uint16 len;

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD); // Clear all receive status bits

        pdw1000local->cbData.rx_flags = 0;

        // Read frame info - Only the first two bytes of the register are used here.
        finfo16 = dwt_read16bitoffsetreg(RX_FINFO_ID, RX_FINFO_OFFSET);

        // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
        len = finfo16 & RX_FINFO_RXFL_MASK_1023;
        if(pdw1000local->longFrames == 0)
        {
            len &= RX_FINFO_RXFLEN_MASK;
        }
        pdw1000local->cbData.datalength = len;

        // Report ranging bit
        if(finfo16 & RX_FINFO_RNG)
        {
            pdw1000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_RNG;
        }

        // Report frame control - First bytes of the received frame.
        dwt_readfromdevice(RX_BUFFER_ID, 0, FCTRL_LEN_MAX, pdw1000local->cbData.fctrl);

        // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
        // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
        // implementation works only for IEEE802.15.4-2011 compliant frames).
        // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
        if((status & SYS_STATUS_AAT) && ((pdw1000local->cbData.fctrl[0] & FCTRL_ACK_REQ_MASK) == 0))
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_AAT); // Clear AAT status bit in register
            pdw1000local->cbData.status &= ~SYS_STATUS_AAT; // Clear AAT status bit in callback data register copy
            pdw1000local->wait4resp = 0;
        }

        // Call the corresponding callback if present
        if(pdw1000local->cbRxOk != NULL)
        {
            pdw1000local->cbRxOk(&pdw1000local->cbData);
        }

        if (pdw1000local->dblbuffon)
        {
            // Toggle the Host side Receive Buffer Pointer
            dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
        }
    }

    // Handle TX confirmation event
    if(status & SYS_STATUS_TXFRS)
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); // Clear TX event bits

        // In the case where this TXFRS interrupt is due to the automatic transmission of an ACK solicited by a response (with ACK request bit set)
        // that we receive through using wait4resp to a previous TX (and assuming that the IRQ processing of that TX has already been handled), then
        // we need to handle the IC issue which turns on the RX again in this situation (i.e. because it is wrongly applying the wait4resp after the
        // ACK TX).
        // See section "Transmit and automatically wait for response" in DW1000 User Manual
        if((status & SYS_STATUS_AAT) && pdw1000local->wait4resp)
        {
            dwt_forcetrxoff(); // Turn the RX off
            dwt_rxreset(); // Reset in case we were late and a frame was already being received
        }

        // Call the corresponding callback if present
        if(pdw1000local->cbTxDone != NULL)
        {
            pdw1000local->cbTxDone(&pdw1000local->cbData);
        }
    }

    // Handle frame reception/preamble detect timeout events
    if(status & SYS_STATUS_ALL_RX_TO)
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO); // Clear RX timeout event bits

        pdw1000local->wait4resp = 0;

        // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
        // the next good frame's timestamp is computed correctly.
        // See section "RX Message timestamp" in DW1000 User Manual.
        dwt_forcetrxoff();
        dwt_rxreset();

        // Call the corresponding callback if present
        if(pdw1000local->cbRxTo != NULL)
        {
            pdw1000local->cbRxTo(&pdw1000local->cbData);
        }
    }

    // Handle RX errors events
    if(status & SYS_STATUS_ALL_RX_ERR)
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); // Clear RX error event bits

        pdw1000local->wait4resp = 0;

        // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
        // the next good frame's timestamp is computed correctly.
        // See section "RX Message timestamp" in DW1000 User Manual.
        dwt_forcetrxoff();
        dwt_rxreset();

        // Call the corresponding callback if present
        if(pdw1000local->cbRxErr != NULL)
        {
            pdw1000local->cbRxErr(&pdw1000local->cbData);
        }
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_isr_lplisten()
 *
 * @brief This is the DW1000's Interrupt Service Routine to use when low-power listening scheme is implemented. It will
 *        only process/report the RXFCG event (through cbRxOk callback).
 *        It clears RXFCG interrupt and reads received frame information and frame control before calling the callback.
 *
 *        /!\ This version of the ISR is designed for single buffering case only!
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_lowpowerlistenisr(void)
{
    uint32 status = pdw1000local->cbData.status = dwt_read32bitreg(SYS_STATUS_ID); // Read status register low 32bits
    uint16 finfo16;
    uint16 len;

    // The only interrupt handled when in low-power listening mode is RX good frame so proceed directly to the handling of the received frame.

    // Deactivate low-power listening before clearing the interrupt. If not, the DW1000 will go back to sleep as soon as the interrupt is cleared.
    dwt_setlowpowerlistening(0);

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD); // Clear all receive status bits

    pdw1000local->cbData.rx_flags = 0;

    // Read frame info - Only the first two bytes of the register are used here.
    finfo16 = dwt_read16bitoffsetreg(RX_FINFO_ID, 0);

    // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
    len = finfo16 & RX_FINFO_RXFL_MASK_1023;
    if(pdw1000local->longFrames == 0)
    {
        len &= RX_FINFO_RXFLEN_MASK;
    }
    pdw1000local->cbData.datalength = len;

    // Report ranging bit
    if(finfo16 & RX_FINFO_RNG)
    {
        pdw1000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_RNG;
    }

    // Report frame control - First bytes of the received frame.
    dwt_readfromdevice(RX_BUFFER_ID, 0, FCTRL_LEN_MAX, pdw1000local->cbData.fctrl);

    // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
    // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
    // implementation works only for IEEE802.15.4-2011 compliant frames).
    // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
    if((status & SYS_STATUS_AAT) && ((pdw1000local->cbData.fctrl[0] & FCTRL_ACK_REQ_MASK) == 0))
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_AAT); // Clear AAT status bit in register
        pdw1000local->cbData.status &= ~SYS_STATUS_AAT; // Clear AAT status bit in callback data register copy
        pdw1000local->wait4resp = 0;
    }

    // Call the corresponding callback if present
    if(pdw1000local->cbRxOk != NULL)
    {
        pdw1000local->cbRxOk(&pdw1000local->cbData);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setleds()
 *
 * @brief This is used to set up Tx/Rx GPIOs which could be used to control LEDs
 * Note: not completely IC dependent, also needs board with LEDS fitted on right I/O lines
 *       this function enables GPIOs 2 and 3 which are connected to LED3 and LED4 on EVB1000
 *
 * input parameters
 * @param mode - this is a bit field interpreted as follows:
 *          - bit 0: 1 to enable LEDs, 0 to disable them
 *          - bit 1: 1 to make LEDs blink once on init. Only valid if bit 0 is set (enable LEDs)
 *          - bit 2 to 7: reserved
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setleds(uint8 mode)
{
    uint32 reg;

    if (mode & DWT_LEDS_ENABLE)
    {
        // Set up MFIO for LED output.
        reg = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
        reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
        reg |= (GPIO_PIN2_RXLED | GPIO_PIN3_TXLED);
        dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);

        // Enable LP Oscillator to run from counter and turn on de-bounce clock.
        reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
        reg |= (PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLEN);
        dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, reg);

        // Enable LEDs to blink and set default blink time.
        reg = PMSC_LEDC_BLNKEN | PMSC_LEDC_BLINK_TIME_DEF;
        // Make LEDs blink once if requested.
        if (mode & DWT_LEDS_INIT_BLINK)
        {
            reg |= PMSC_LEDC_BLINK_NOW_ALL;
        }
        dwt_write32bitoffsetreg(PMSC_ID, PMSC_LEDC_OFFSET, reg);
        // Clear force blink bits if needed.
        if(mode & DWT_LEDS_INIT_BLINK)
        {
            reg &= ~PMSC_LEDC_BLINK_NOW_ALL;
            dwt_write32bitoffsetreg(PMSC_ID, PMSC_LEDC_OFFSET, reg);
        }
    }
    else
    {
        // Clear the GPIO bits that are used for LED control.
        reg = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
        reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
        dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_enableclocks()
 *
 * @brief function to enable/disable clocks to particular digital blocks/system
 *
 * input parameters
 * @param clocks - set of clocks to enable/disable
 *
 * output parameters none
 *
 * no return value
 */
void _dwt_enableclocks(int clocks)
{
    uint8 reg[2];

    dwt_readfromdevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, reg);
    switch(clocks)
    {
        case ENABLE_ALL_SEQ:
        {
            reg[0] = 0x00 ;
            reg[1] = reg[1] & 0xfe;
        }
        break;
        case FORCE_SYS_XTI:
        {
            // System and RX
            reg[0] = 0x01 | (reg[0] & 0xfc);
        }
        break;
        case FORCE_SYS_PLL:
        {
            // System
            reg[0] = 0x02 | (reg[0] & 0xfc);
        }
        break;
        case READ_ACC_ON:
        {
            reg[0] = 0x48 | (reg[0] & 0xb3);
            reg[1] = 0x80 | reg[1];
        }
        break;
        case READ_ACC_OFF:
        {
            reg[0] = reg[0] & 0xb3;
            reg[1] = 0x7f & reg[1];
        }
        break;
        case FORCE_OTP_ON:
        {
            reg[1] = 0x02 | reg[1];
        }
        break;
        case FORCE_OTP_OFF:
        {
            reg[1] = reg[1] & 0xfd;
        }
        break;
        case FORCE_TX_PLL:
        {
            reg[0] = 0x20 | (reg[0] & 0xcf);
        }
        break;
        case FORCE_LDE:
        {
            reg[0] = 0x01;
            reg[1] = 0x03;
        }
        break;
        default:
        break;
    }


    // Need to write lower byte separately before setting the higher byte(s)
    dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, &reg[0]);
    dwt_writetodevice(PMSC_ID, 0x1, 1, &reg[1]);

} // end _dwt_enableclocks()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_disablesequencing()
 *
 * @brief This function disables the TX blocks sequencing, it disables PMSC control of RF blocks, system clock is also set to XTAL
 *
 * input parameters none
 *
 * output parameters none
 *
 * no return value
 */
void _dwt_disablesequencing(void) // Disable sequencing and go to state "INIT"
{
    _dwt_enableclocks(FORCE_SYS_XTI); // Set system clock to XTI

    dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE); // Disable PMSC ctrl of RF and RX clk blocks
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setdelayedtrxtime()
 *
 * @brief This API function configures the delayed transmit time or the delayed RX on time
 *
 * input parameters
 * @param starttime - the TX/RX start time (the 32 bits should be the high 32 bits of the system time at which to send the message,
 * or at which to turn on the receiver)
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setdelayedtrxtime(uint32 starttime)
{
    dwt_write32bitoffsetreg(DX_TIME_ID, 1, starttime); // Write at offset 1 as the lower 9 bits of this register are ignored

} // end dwt_setdelayedtrxtime()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_starttx()
 *
 * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
 *
 * input parameters:
 * @param mode - if 0 immediate TX (no response expected)
 *               if 1 delayed TX (no response expected)
 *               if 2 immediate TX (response expected - so the receiver will be automatically turned on after TX is done)
 *               if 3 delayed TX (response expected - so the receiver will be automatically turned on after TX is done)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed transmission will fail if the delayed time has passed)
 */
int dwt_starttx(uint8 mode)
{
    int retval = DWT_SUCCESS ;
    uint8 temp  = 0x00;
    uint16 checkTxOK = 0 ;

    if(mode & DWT_RESPONSE_EXPECTED)
    {
        temp = (uint8)SYS_CTRL_WAIT4RESP ; // Set wait4response bit
        dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, temp);
        pdw1000local->wait4resp = 1;
    }

    if (mode & DWT_START_TX_DELAYED)
    {
        // Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
        temp |= (uint8)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT) ;
        dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, temp);
        checkTxOK = dwt_read16bitoffsetreg(SYS_STATUS_ID, 3); // Read at offset 3 to get the upper 2 bytes out of 5
        if ((checkTxOK & SYS_STATUS_TXERR) == 0) // Transmit Delayed Send set over Half a Period away or Power Up error (there is enough time to send but not to power up individual blocks).
        {
            retval = DWT_SUCCESS ; // All okay
        }
        else
        {
            // I am taking DSHP set to Indicate that the TXDLYS was set too late for the specified DX_TIME.
            // Remedial Action - (a) cancel delayed send
            temp = (uint8)SYS_CTRL_TRXOFF; // This assumes the bit is in the lowest byte
            dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, temp);
            // Note event Delayed TX Time too Late
            // Could fall through to start a normal send (below) just sending late.....
            // ... instead return and assume return value of 1 will be used to detect and recover from the issue.
            pdw1000local->wait4resp = 0;
            retval = DWT_ERROR ; // Failed !
        }
    }
    else
    {
        temp |= (uint8)SYS_CTRL_TXSTRT ;
        dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, temp);
    }

    return retval;

} // end dwt_starttx()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_forcetrxoff()
 *
 * @brief This is used to turn off the transceiver
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_forcetrxoff(void)
{
    decaIrqStatus_t stat ;
    uint32 mask;

    mask = dwt_read32bitreg(SYS_MASK_ID) ; // Read set interrupt mask

    // Need to beware of interrupts occurring in the middle of following read modify write cycle
    // We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
    // event has just happened before the radio was disabled)
    // thus we need to disable interrupt during this operation
    stat = decamutexon() ;

    dwt_write32bitreg(SYS_MASK_ID, 0) ; // Clear interrupt mask - so we don't get any unwanted events

    dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8)SYS_CTRL_TRXOFF) ; // Disable the radio

    // Forcing Transceiver off - so we do not want to see any new events that may have happened
    dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD));

    dwt_syncrxbufptrs();

    dwt_write32bitreg(SYS_MASK_ID, mask) ; // Set interrupt mask to what it was

    // Enable/restore interrupts again...
    decamutexoff(stat) ;
    pdw1000local->wait4resp = 0;

} // end deviceforcetrxoff()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_syncrxbufptrs()
 *
 * @brief this function synchronizes rx buffer pointers
 * need to make sure that the host/IC buffer pointers are aligned before starting RX
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dwt_syncrxbufptrs(void)
{
    uint8  buff ;
    // Need to make sure that the host/IC buffer pointers are aligned before starting RX
    buff = dwt_read8bitoffsetreg(SYS_STATUS_ID, 3); // Read 1 byte at offset 3 to get the 4th byte out of 5

    if((buff & (SYS_STATUS_ICRBP >> 24)) !=     // IC side Receive Buffer Pointer
       ((buff & (SYS_STATUS_HSRBP>>24)) << 1) ) // Host Side Receive Buffer Pointer
    {
        dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0x01) ; // We need to swap RX buffer status reg (write one to toggle internally)
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setsniffmode()
 *
 * @brief enable/disable and configure SNIFF mode.
 *
 * SNIFF mode is a low-power reception mode where the receiver is sequenced on and off instead of being on all the time.
 * The time spent in each state (on/off) is specified through the parameters below.
 * See DW1000 User Manual section 4.5 "Low-Power SNIFF mode" for more details.
 *
 * input parameters:
 * @param enable - 1 to enable SNIFF mode, 0 to disable. When 0, all other parameters are not taken into account.
 * @param timeOn - duration of receiver ON phase, expressed in multiples of PAC size. The counter automatically adds 1 PAC
 *                 size to the value set. Min value that can be set is 1 (i.e. an ON time of 2 PAC size), max value is 15.
 * @param timeOff - duration of receiver OFF phase, expressed in multiples of 128/125 탎 (~1 탎). Max value is 255.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setsniffmode(int enable, uint8 timeOn, uint8 timeOff)
{
    uint32 pmsc_reg;
    if (enable)
    {
        /* Configure ON/OFF times and enable PLL2 on/off sequencing by SNIFF mode. */
        uint16 sniff_reg = ((timeOff << 8) | timeOn) & RX_SNIFF_MASK;
        dwt_write16bitoffsetreg(RX_SNIFF_ID, RX_SNIFF_OFFSET, sniff_reg);
        pmsc_reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
        pmsc_reg |= PMSC_CTRL0_PLL2_SEQ_EN;
        dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_reg);
    }
    else
    {
        /* Clear ON/OFF times and disable PLL2 on/off sequencing by SNIFF mode. */
        dwt_write16bitoffsetreg(RX_SNIFF_ID, RX_SNIFF_OFFSET, 0x0000);
        pmsc_reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
        pmsc_reg &= ~PMSC_CTRL0_PLL2_SEQ_EN;
        dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_reg);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setlowpowerlistening()
 *
 * @brief enable/disable low-power listening mode.
 *
 * Low-power listening is a feature whereby the DW1000 is predominantly in the SLEEP state but wakes periodically, (after
 * this "long sleep"), for a very short time to sample the air for a preamble sequence. This preamble sampling "listening"
 * phase is actually two reception phases separated by a "short sleep" time. See DW1000 User Manual section "Low-Power
 * Listening" for more details.
 *
 * NOTE: Before enabling low-power listening, the following functions have to be called to fully configure it:
 *           - dwt_configuresleep() to configure long sleep phase. "mode" parameter should at least have DWT_PRESRV_SLEEP,
 *             DWT_CONFIG and DWT_RX_EN set and "wake" parameter should at least have both DWT_WAKE_SLPCNT and DWT_SLP_EN set.
 *           - dwt_calibratesleepcnt() and dwt_configuresleepcnt() to define the "long sleep" phase duration.
 *           - dwt_setsnoozetime() to define the "short sleep" phase duration.
 *           - dwt_setpreambledetecttimeout() to define the reception phases duration.
 *           - dwt_setinterrupt() to activate RX good frame interrupt (DWT_INT_RFCG) only.
 *       When configured, low-power listening mode can be triggered either by putting the DW1000 to sleep (using
 *       dwt_entersleep()) or by activating reception (using dwt_rxenable()).
 *
 *       Please refer to the low-power listening examples (examples 8a/8b accompanying the API distribution on Decawave's
 *       website). They form a working example code that shows how to use low-power listening correctly.
 *
 * input parameters:
 * @param enable - 1 to enable low-power listening, 0 to disable.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setlowpowerlistening(int enable)
{
    uint32 pmsc_reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
    if (enable)
    {
        /* Configure RX to sleep and snooze features. */
        pmsc_reg |= (PMSC_CTRL1_ARXSLP | PMSC_CTRL1_SNOZE);
    }
    else
    {
        /* Reset RX to sleep and snooze features. */
        pmsc_reg &= ~(PMSC_CTRL1_ARXSLP | PMSC_CTRL1_SNOZE);
    }
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, pmsc_reg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setsnoozetime()
 *
 * @brief Set duration of "short sleep" phase when in low-power listening mode.
 *
 * input parameters:
 * @param snooze_time - "short sleep" phase duration, expressed in multiples of 512/19.2 탎 (~26.7 탎). The counter
 *                      automatically adds 1 to the value set. The smallest working value that should be set is 1,
 *                      i.e. giving a snooze time of 2 units (or ~53 탎).
 *
 * output parameters
 *
 * no return value
 */
void dwt_setsnoozetime(uint8 snooze_time)
{
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_SNOZT_OFFSET, snooze_time);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_rxenable()
 *
 * @brief This call turns on the receiver, can be immediate or delayed (depending on the mode parameter). In the case of a
 * "late" error the receiver will only be turned on if the DWT_IDLE_ON_DLY_ERR is not set.
 * The receiver will stay turned on, listening to any messages until
 * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or  it times out (SFD, Preamble or Frame).
 *
 * input parameters
 * @param mode - this can be one of the following allowed values:
 *
 * DWT_START_RX_IMMEDIATE      0 used to enbale receiver immediately
 * DWT_START_RX_DELAYED        1 used to set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
 * (DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR) 3 used to disable re-enabling of receiver if delayed RX failed due to "late" error
 * (DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS) 4 used to re-enable RX without trying to sync IC and host side buffer pointers, typically when
 *                                               performing manual RX re-enabling in double buffering mode
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed receive enable will be too far in the future if delayed time has passed)
 */
int dwt_rxenable(int mode)
{
    uint16 temp ;
    uint8 temp1 ;

    if ((mode & DWT_NO_SYNC_PTRS) == 0)
    {
        dwt_syncrxbufptrs();
    }

    temp = (uint16)SYS_CTRL_RXENAB ;

    if (mode & DWT_START_RX_DELAYED)
    {
        temp |= (uint16)SYS_CTRL_RXDLYE ;
    }

    dwt_write16bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, temp);

    if (mode & DWT_START_RX_DELAYED) // check for errors
    {
        temp1 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 3); // Read 1 byte at offset 3 to get the 4th byte out of 5
        if ((temp1 & (SYS_STATUS_HPDWARN >> 24)) != 0) // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
        {
            dwt_forcetrxoff(); // turn the delayed receive off

            if((mode & DWT_IDLE_ON_DLY_ERR) == 0) // if DWT_IDLE_ON_DLY_ERR not set then re-enable receiver
            {
                dwt_write16bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB);
            }
            return DWT_ERROR; // return warning indication
        }
    }

    return DWT_SUCCESS;
} // end dwt_rxenable()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxtimeout()
 *
 * @brief This call enables RX timeout (SY_STAT_RFTO event)
 *
 * input parameters
 * @param time - how long the receiver remains on from the RX enable command
 *               The time parameter used here is in 1.0256 us (512/499.2MHz) units
 *               If set to 0 the timeout is disabled.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxtimeout(uint16 time)
{
    uint8 temp ;

    temp = dwt_read8bitoffsetreg(SYS_CFG_ID, 3); // Read at offset 3 to get the upper byte only

    if(time > 0)
    {
        dwt_write16bitoffsetreg(RX_FWTO_ID, RX_FWTO_OFFSET, time) ;

        temp |= (uint8)(SYS_CFG_RXWTOE>>24); // Shift RXWTOE mask as we read the upper byte only
        // OR in 32bit value (1 bit set), I know this is in high byte.
        pdw1000local->sysCFGreg |= SYS_CFG_RXWTOE;

        dwt_write8bitoffsetreg(SYS_CFG_ID, 3, temp); // Write at offset 3 to write the upper byte only
    }
    else
    {
        temp &= ~((uint8)(SYS_CFG_RXWTOE>>24)); // Shift RXWTOE mask as we read the upper byte only
        // AND in inverted 32bit value (1 bit clear), I know this is in high byte.
        pdw1000local->sysCFGreg &= ~(SYS_CFG_RXWTOE);

        dwt_write8bitoffsetreg(SYS_CFG_ID, 3, temp); // Write at offset 3 to write the upper byte only
    }

} // end dwt_setrxtimeout()


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setpreambledetecttimeout()
 *
 * @brief This call enables preamble timeout (SY_STAT_RXPTO event)
 *
 * input parameters
 * @param  timeout - Preamble detection timeout, expressed in multiples of PAC size. The counter automatically adds 1 PAC
 *                   size to the value set. Min value that can be set is 1 (i.e. a timeout of 2 PAC size).
 *
 * output parameters
 *
 * no return value
 */
void dwt_setpreambledetecttimeout(uint16 timeout)
{
    dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_PRETOC_OFFSET, timeout);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn void dwt_setinterrupt()
 *
 * @brief This function enables the specified events to trigger an interrupt.
 * The following events can be enabled:
 * DWT_INT_TFRS         0x00000080          // frame sent
 * DWT_INT_RFCG         0x00004000          // frame received with good CRC
 * DWT_INT_RPHE         0x00001000          // receiver PHY header error
 * DWT_INT_RFCE         0x00008000          // receiver CRC error
 * DWT_INT_RFSL         0x00010000          // receiver sync loss error
 * DWT_INT_RFTO         0x00020000          // frame wait timeout
 * DWT_INT_RXPTO        0x00200000          // preamble detect timeout
 * DWT_INT_SFDT         0x04000000          // SFD timeout
 * DWT_INT_ARFE         0x20000000          // frame rejected (due to frame filtering configuration)
 *
 *
 * input parameters:
 * @param bitmask - sets the events which will generate interrupt
 * @param enable - if set the interrupts are enabled else they are cleared
 *
 * output parameters
 *
 * no return value
 */
void dwt_setinterrupt(uint32 bitmask, uint8 enable)
{
    decaIrqStatus_t stat ;
    uint32 mask ;

    // Need to beware of interrupts occurring in the middle of following read modify write cycle
    stat = decamutexon() ;

    mask = dwt_read32bitreg(SYS_MASK_ID) ; // Read register

    if(enable)
    {
        mask |= bitmask ;
    }
    else
    {
        mask &= ~bitmask ; // Clear the bit
    }
    dwt_write32bitreg(SYS_MASK_ID,mask) ; // New value

    decamutexoff(stat) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configeventcounters()
 *
 * @brief This is used to enable/disable the event counter in the IC
 *
 * input parameters
 * @param - enable - 1 enables (and reset), 0 disables the event counters
 * output parameters
 *
 * no return value
 */
void dwt_configeventcounters(int enable)
{
    // Need to clear and disable, can't just clear
    dwt_write8bitoffsetreg(DIG_DIAG_ID, EVC_CTRL_OFFSET, (uint8)(EVC_CLR));

    if(enable)
    {
        dwt_write8bitoffsetreg(DIG_DIAG_ID, EVC_CTRL_OFFSET, (uint8)(EVC_EN)); // Enable
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readeventcounters()
 *
 * @brief This is used to read the event counters in the IC
 *
 * input parameters
 * @param counters - pointer to the dwt_deviceentcnts_t structure which will hold the read data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readeventcounters(dwt_deviceentcnts_t *counters)
{
    uint32 temp;

    temp= dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_PHE_OFFSET); // Read sync loss (31-16), PHE (15-0)
    counters->PHE = temp & 0xFFF;
    counters->RSL = (temp >> 16) & 0xFFF;

    temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FCG_OFFSET); // Read CRC bad (31-16), CRC good (15-0)
    counters->CRCG = temp & 0xFFF;
    counters->CRCB = (temp >> 16) & 0xFFF;

    temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FFR_OFFSET); // Overruns (31-16), address errors (15-0)
    counters->ARFE = temp & 0xFFF;
    counters->OVER = (temp >> 16) & 0xFFF;

    temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_STO_OFFSET); // Read PTO (31-16), SFDTO (15-0)
    counters->PTO = (temp >> 16) & 0xFFF;
    counters->SFDTO = temp & 0xFFF;

    temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FWTO_OFFSET); // Read RX TO (31-16), TXFRAME (15-0)
    counters->TXF = (temp >> 16) & 0xFFF;
    counters->RTO = temp & 0xFFF;

    temp = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_HPW_OFFSET); // Read half period warning events
    counters->HPW = temp & 0xFFF;
    counters->TXW = (temp >> 16) & 0xFFF;                       // Power-up warning events

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_rxreset()
 *
 * @brief this function resets the receiver of the DW1000
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dwt_rxreset(void)
{
    // Set RX reset
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_RX);

    // Clear RX reset
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_softreset()
 *
 * @brief this function resets the DW1000
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dwt_softreset(void)
{
    _dwt_disablesequencing();

    // Clear any AON auto download bits (as reset will trigger AON download)
    dwt_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, 0x00);
    // Clear the wake-up configuration
    dwt_write8bitoffsetreg(AON_ID, AON_CFG0_OFFSET, 0x00);
    // Upload the new configuration
    _dwt_aonarrayupload();

    // Reset HIF, TX, RX and PMSC
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_ALL);

    // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
    // Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier
    deca_sleep(1);

    // Clear reset
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR);

    pdw1000local->wait4resp = 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setxtaltrim()
 *
 * @brief This is used to adjust the crystal frequency
 *
 * input parameters:
 * @param   value - crystal trim value (in range 0x0 to 0x1F) 31 steps (~1.5ppm per step)
 *
 * output parameters
 *
 * no return value
 */
void dwt_setxtaltrim(uint8 value)
{
    // The 3 MSb in this 8-bit register must be kept to 0b011 to avoid any malfunction.
    uint8 reg_val = (3 << 5) | (value & FS_XTALT_MASK);
    dwt_write8bitoffsetreg(FS_CTRL_ID, FS_XTALT_OFFSET, reg_val);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getinitxtaltrim()
 *
 * @brief This function returns the value of XTAL trim that has been applied during initialisation (dwt_init). This can
 *        be either the value read in OTP memory or a default value.
 *
 * NOTE: The value returned by this function is the initial value only! It is not updated on dwt_setxtaltrim calls.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the XTAL trim value set upon initialisation
 */
uint8 dwt_getinitxtaltrim(void)
{
    return pdw1000local->init_xtrim;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configcwmode()
 *
 * @brief this function sets the DW1000 to transmit cw signal at specific channel frequency
 *
 * input parameters:
 * @param chan - specifies the operating channel (e.g. 1, 2, 3, 4, 5, 6 or 7)
 *
 * output parameters
 *
 * no return value
 */
void dwt_configcwmode(uint8 chan)
{
#ifdef DWT_API_ERROR_CHECK
    assert((chan >= 1) && (chan <= 7) && (chan != 6));
#endif

    //
    // Disable TX/RX RF block sequencing (needed for cw frame mode)
    //
    _dwt_disablesequencing();

    // Config RF pll (for a given channel)
    // Configure PLL2/RF PLL block CFG/TUNE
    dwt_write32bitoffsetreg(FS_CTRL_ID, FS_PLLCFG_OFFSET, fs_pll_cfg[chan_idx[chan]]);
    dwt_write8bitoffsetreg(FS_CTRL_ID, FS_PLLTUNE_OFFSET, fs_pll_tune[chan_idx[chan]]);
    // PLL wont be enabled until a TX/RX enable is issued later on
    // Configure RF TX blocks (for specified channel and prf)
    // Config RF TX control
    dwt_write32bitoffsetreg(RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config[chan_idx[chan]]);

    //
    // Enable RF PLL
    //
    dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPLLPOWEN_MASK); // Enable LDO and RF PLL blocks
    dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXALLEN_MASK); // Enable the rest of TX blocks

    //
    // Configure TX clocks
    //
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, 0x22);
    dwt_write8bitoffsetreg(PMSC_ID, 0x1, 0x07);

    // Disable fine grain TX sequencing
    dwt_setfinegraintxseq(0);

    // Configure CW mode
    dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGTEST_OFFSET, TC_PGTEST_CW);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configcontinuousframemode()
 *
 * @brief this function sets the DW1000 to continuous tx frame mode for regulatory approvals testing.
 *
 * input parameters:
 * @param framerepetitionrate - This is a 32-bit value that is used to set the interval between transmissions.
*  The minimum value is 4. The units are approximately 8 ns. (or more precisely 512/(499.2e6*128) seconds)).
 *
 * output parameters
 *
 * no return value
 */
void dwt_configcontinuousframemode(uint32 framerepetitionrate)
{
    //
    // Disable TX/RX RF block sequencing (needed for continuous frame mode)
    //
    _dwt_disablesequencing();

    //
    // Enable RF PLL and TX blocks
    //
    dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPLLPOWEN_MASK); // Enable LDO and RF PLL blocks
    dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXALLEN_MASK); // Enable the rest of TX blocks

    //
    // Configure TX clocks
    //
    _dwt_enableclocks(FORCE_SYS_PLL);
    _dwt_enableclocks(FORCE_TX_PLL);

    // Set the frame repetition rate
    if(framerepetitionrate < 4)
    {
        framerepetitionrate = 4;
    }
    dwt_write32bitreg(DX_TIME_ID, framerepetitionrate);

    //
    // Configure continuous frame TX
    //
    dwt_write8bitoffsetreg(DIG_DIAG_ID, DIAG_TMC_OFFSET, (uint8)(DIAG_TMC_TX_PSTM)); // Turn the tx power spectrum test mode - continuous sending of frames
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtempvbat()
 *
 * @brief this function reads the battery voltage and temperature of the MP
 * The values read here will be the current values sampled by DW1000 AtoD converters.
 * Note on Temperature: the temperature value needs to be converted to give the real temperature
 * the formula is: 1.13 * reading - 113.0
 * Note on Voltage: the voltage value needs to be converted to give the real voltage
 * the formula is: 0.0057 * reading + 2.3
 *
 * NB: To correctly read the temperature this read should be done with xtal clock
 * however that means that the receiver will be switched off, if receiver needs to be on then
 * the timer is used to make sure the value is stable before reading
 *
 * input parameters:
 * @param fastSPI - set to 1 if SPI rate > than 3MHz is used
 *
 * output parameters
 *
 * returns  (temp_raw<<8)|(vbat_raw)
 */
uint16 dwt_readtempvbat(uint8 fastSPI)
{
    uint8 wr_buf[2];
    uint8 vbat_raw;
    uint8 temp_raw;

    // These writes should be single writes and in sequence
    wr_buf[0] = 0x80; // Enable TLD Bias
    dwt_writetodevice(RF_CONF_ID,0x11,1,wr_buf);

    wr_buf[0] = 0x0A; // Enable TLD Bias and ADC Bias
    dwt_writetodevice(RF_CONF_ID,0x12,1,wr_buf);

    wr_buf[0] = 0x0f; // Enable Outputs (only after Biases are up and running)
    dwt_writetodevice(RF_CONF_ID,0x12,1,wr_buf);    //

    // Reading All SAR inputs
    wr_buf[0] = 0x00;
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);
    wr_buf[0] = 0x01; // Set SAR enable
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);

    if(fastSPI == 1)
    {
        deca_sleep(1); // If using PLL clocks(and fast SPI rate) then this sleep is needed
        // Read voltage and temperature.
        dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET,2,wr_buf);
    }
    else //change to a slow clock
    {
        _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is necessary to make sure the values read are reliable
        // Read voltage and temperature.
        dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET,2,wr_buf);
        // Default clocks (ENABLE_ALL_SEQ)
        _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
    }

    vbat_raw = wr_buf[0];
    temp_raw = wr_buf[1];

    wr_buf[0] = 0x00; // Clear SAR enable
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);

    return ((temp_raw<<8)|(vbat_raw));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readwakeuptemp()
 *
 * @brief this function reads the temperature of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns: 8-bit raw temperature sensor value
 */
uint8 dwt_readwakeuptemp(void)
{
    return dwt_read8bitoffsetreg(TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readwakeupvbat()
 *
 * @brief this function reads the battery voltage of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns: 8-bit raw battery voltage sensor value
 */
uint8 dwt_readwakeupvbat(void)
{
    return dwt_read8bitoffsetreg(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_calcbandwidthtempadj()
 *
 * @brief this function determines the corrected bandwidth setting (PG_DELAY register setting)
 * of the DW1000 which changes over temperature.
 *
 * input parameters:
 * @param target_count - uint16 - the PG count target to reach in order to correct the bandwidth
 *
 * output parameters:
 *
 * returns: (uint32) The setting to be programmed into the PG_DELAY value
 */
uint32 dwt_calcbandwidthtempadj(uint16 target_count)
{
    int i;
    uint32 bit_field, curr_bw;
    int32 delta_count = 0;
    uint32 best_bw = 0;
    uint16 raw_count = 0;
    int32 delta_lowest;

    // Used to store the current values of the registers so that they can be restored after
    uint8 old_pmsc_ctrl0;
    uint16 old_pmsc_ctrl1;
    uint32 old_rf_conf_txpow_mask;

    // Record the current values of these registers, to restore later
    old_pmsc_ctrl0 = dwt_read8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
    old_pmsc_ctrl1 = dwt_read16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
    old_rf_conf_txpow_mask = dwt_read32bitreg(RF_CONF_ID);

    //  Set clock to XTAL
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_SYSCLKS_19M);

    //  Disable sequencing
    dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE);

    //  Turn on CLK PLL, Mix Bias and PG
    dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK);

    //  Set sys and TX clock to PLL
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_SYSCLKS_125M | PMSC_CTRL0_TXCLKS_125M);

    // Set the MSB high for first guess
    curr_bw = 0x80;
    // Set starting bit
    bit_field = 0x80;
    // Initial lowest delta is the maximum difference that we should allow the count value to be from the target.
    // If the algorithm is successful, it will be overwritten by a smaller value where the count value is closer
    // to the target
    delta_lowest = 300;

    for (i = 0; i < 7; i++)
    {
        // start with 0xc0 and test.
        bit_field = bit_field >> 1;
        curr_bw = curr_bw | bit_field;

        // Write bw setting to PG_DELAY register
        dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGDELAY_OFFSET, curr_bw);

        // Set cal direction and time
        dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGCCTRL_OFFSET, TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK);

        // Start cal
        dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGCCTRL_OFFSET, TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK | TC_PGCCTRL_CALSTART);
        // Allow cal to complete
        deca_sleep(100);

        // Read count value from the PG cal block
        raw_count = dwt_read16bitoffsetreg(TX_CAL_ID, TC_PGCAL_STATUS_OFFSET) & TC_PGCAL_STATUS_DELAY_MASK;

        // lets keep track of the closest value to the target in case we overshoot
        delta_count = abs((int)raw_count - (int)target_count);
        if (delta_count < delta_lowest)
        {
            delta_lowest = delta_count;
            best_bw = curr_bw;
        }

        // Test the count results
        if (raw_count > target_count)
            // Count was lower, BW was lower so increase PG DELAY
            curr_bw = curr_bw | bit_field;
        else
            // Count was higher
            curr_bw = curr_bw & (~(bit_field));
    }

    // Restore old register values
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, old_pmsc_ctrl0);
    dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, old_pmsc_ctrl1);
    dwt_write32bitreg(RF_CONF_ID, old_rf_conf_txpow_mask);

    // Returns the best PG_DELAY setting
    return best_bw;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_computetxpowersetting()
 *
 * @brief this function calculates the appropriate change to the TX_POWER register to compensate
 * the TX power output at different temperatures.
 *
 * input parameters:
 * @param ref_powerreg - uint32 - the TX_POWER register value recorded when reference measurements were made
 * @param power_adj - uint32 - the adjustment in power level to be made, in 0.5dB steps
 *
 * output parameters:
 *
 * returns: (uint32) The setting to be programmed into the TX_POWER register
 */
uint32 _dwt_computetxpowersetting(uint32 ref_powerreg, int32 power_adj)
{
    int32 da_attn_change, mixer_gain_change;
    uint8 current_da_attn, current_mixer_gain;
    uint8 new_da_attn, new_mixer_gain;
    uint32 new_regval = 0;
    int i;

    for(i = 0; i < 4; i++)
    {
        da_attn_change = 0;
        mixer_gain_change = power_adj;
        current_da_attn = ((ref_powerreg >> (i*8)) & 0xE0) >> 5;
        current_mixer_gain = (ref_powerreg >> (i*8)) & 0x1F;

        // Mixer gain gives best performance between 4 and 20
        while((current_mixer_gain + mixer_gain_change < 4) ||
              (current_mixer_gain + mixer_gain_change > 20))
        {
            // If mixer gain goes outside bounds, adjust the DA attenuation to compensate
            if(current_mixer_gain + mixer_gain_change > 20)
            {
                da_attn_change += 1;
                mixer_gain_change -= (int) (DA_ATTN_STEP / MIXER_GAIN_STEP);
            }
            else if(current_mixer_gain + mixer_gain_change < 4)
            {
                da_attn_change += 1;
                mixer_gain_change += (int) (DA_ATTN_STEP / MIXER_GAIN_STEP);
            }
        }

        new_da_attn = current_da_attn + da_attn_change;
        new_mixer_gain = current_mixer_gain + mixer_gain_change;

        new_regval |= ((uint32) ((new_da_attn << 5) | new_mixer_gain)) << (i * 8);
    }

    return (uint32)new_regval;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_calcpowertempadj()
 *
 * @brief this function determines the corrected power setting (TX_POWER setting) for the
 * DW1000 which changes over temperature.
 *
 * input parameters:
 * @param channel - uint8 - the channel at which compensation of power level will be applied
 * @param ref_powerreg - uint32 - the TX_POWER register value recorded when reference measurements were made
 * @param current_temperature - double - the current ambient temperature in degrees Celcius
 * @param reference_temperature - double - the temperature at which reference measurements were made
 * output parameters: None
 *
 * returns: (uint32) The corrected TX_POWER register value
 */
 uint32 dwt_calcpowertempadj
(
       uint8 channel,
       uint32 ref_powerreg,
       double curr_temp,
       double ref_temp
)
{
    double delta_temp;
    double delta_power;

    // Find the temperature differential
    delta_temp = curr_temp - ref_temp;

    // Calculate the expected power differential at the current temperature
    delta_power = delta_temp * txpwr_compensation[chan_idx[channel]];

    // Adjust the TX_POWER register value
    return _dwt_computetxpowersetting(ref_powerreg, (int32)(delta_power / MIXER_GAIN_STEP));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_calcpgcount()
 *
 * @brief this function calculates the value in the pulse generator counter register (PGC_STATUS) for a given PG_DELAY
 * This is used to take a reference measurement, and the value recorded as the reference is used to adjust the
 * bandwidth of the device when the temperature changes.
 *
 * input parameters:
 * @param pgdly - uint8 - the PG_DELAY to set (to control bandwidth), and to find the corresponding count value for
 * output parameters: None
 *
 * returns: (uint16) PGC_STATUS count value calculated from the provided PG_DELAY value - used as reference for later
 * bandwidth adjustments
 */
uint16 dwt_calcpgcount(uint8 pgdly)
{
    // Perform PG count read ten times and take an average to smooth out any noise
    const int NUM_SAMPLES = 10;
    uint32 sum_count = 0;
    uint16 average_count = 0, count = 0;
    int i = 0;

    // Used to store the current values of the registers so that they can be restored after
    uint8 old_pmsc_ctrl0;
    uint16 old_pmsc_ctrl1;
    uint32 old_rf_conf_txpow_mask;

    // Record the current values of these registers, to restore later
    old_pmsc_ctrl0 = dwt_read8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
    old_pmsc_ctrl1 = dwt_read16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
    old_rf_conf_txpow_mask = dwt_read32bitreg(RF_CONF_ID);

    //  Set clock to XTAL
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_SYSCLKS_19M);
    //  Disable sequencing
    dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE);
    //  Turn on CLK PLL, Mix Bias and PG
    dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK);
    //  Set sys and TX clock to PLL
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_SYSCLKS_125M | PMSC_CTRL0_TXCLKS_125M);

    for(i = 0; i < NUM_SAMPLES; i++) {
        // Write bw setting to PG_DELAY register
        dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGDELAY_OFFSET, pgdly);

        // Set cal direction and time
        dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGCCTRL_OFFSET, TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK);

        // Start cal
        dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGCCTRL_OFFSET, TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK | TC_PGCCTRL_CALSTART);

        // Allow cal to complete - the TC_PGCCTRL_CALSTART bit will clear automatically
        deca_sleep(100);

        // Read count value from the PG cal block
        count = dwt_read16bitoffsetreg(TX_CAL_ID, TC_PGCAL_STATUS_OFFSET) & TC_PGCAL_STATUS_DELAY_MASK;

        sum_count += count;
    }

     // Restore old register values
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, old_pmsc_ctrl0);
    dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, old_pmsc_ctrl1);
    dwt_write32bitreg(RF_CONF_ID, old_rf_conf_txpow_mask);

    average_count = (int)(sum_count / NUM_SAMPLES);
    return average_count;
}


/* ===============================================================================================
   List of expected (known) device ID handled by this software
   ===============================================================================================

    0xDECA0130                               // DW1000 - MP

   ===============================================================================================
*/

