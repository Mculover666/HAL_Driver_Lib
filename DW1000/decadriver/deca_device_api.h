/*! ----------------------------------------------------------------------------
 * @file    deca_device_api.h
 * @brief   DW1000 API Functions
 *
 * @attention
 *
 * Copyright 2013 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _DECA_DEVICE_API_H_
#define _DECA_DEVICE_API_H_

#ifdef __cplusplus
extern "C" {
#endif


#ifndef uint8
#ifndef _DECA_UINT8_
#define _DECA_UINT8_
typedef unsigned char uint8;
#endif
#endif

#ifndef uint16
#ifndef _DECA_UINT16_
#define _DECA_UINT16_
typedef unsigned short uint16;
#endif
#endif

#ifndef uint32
#ifndef _DECA_UINT32_
#define _DECA_UINT32_
typedef unsigned long uint32;
#endif
#endif

#ifndef int8
#ifndef _DECA_INT8_
#define _DECA_INT8_
typedef signed char int8;
#endif
#endif

#ifndef int16
#ifndef _DECA_INT16_
#define _DECA_INT16_
typedef signed short int16;
#endif
#endif

#ifndef int32
#ifndef _DECA_INT32_
#define _DECA_INT32_
typedef signed long int32;
#endif
#endif

#ifndef DWT_NUM_DW_DEV
#define DWT_NUM_DW_DEV (1)
#endif

#define DWT_SUCCESS (0)
#define DWT_ERROR   (-1)

#define DWT_TIME_UNITS          (1.0/499.2e6/128.0) //!< = 15.65e-12 s

#define DWT_DEVICE_ID   (0xDECA0130)        //!< DW1000 MP device ID

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_110K     0   //!< UWB bit rate 110 kbits/s
#define DWT_BR_850K     1   //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8      2   //!< UWB bit rate 6.8 Mbits/s

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
#define DWT_PRF_16M     1   //!< UWB PRF 16 MHz
#define DWT_PRF_64M     2   //!< UWB PRF 64 MHz

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8        0   //!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16       1   //!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32       2   //!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC64       3   //!< PAC 64 (recommended for RX of preamble length 1024 and up

//! constants for specifying TX Preamble length in symbols
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096   0x0C    //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048   0x28    //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536   0x18    //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024   0x08    //! Standard preamble length 1024 symbols
#define DWT_PLEN_512    0x34    //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256    0x24    //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128    0x14    //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64     0x04    //! Standard preamble length 64 symbols

#define DWT_SFDTOC_DEF              0x1041  // default SFD timeout value

#define DWT_PHRMODE_STD             0x0     // standard PHR mode
#define DWT_PHRMODE_EXT             0x3     // DW proprietary extended frames PHR mode

// Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
#define DWT_START_TX_IMMEDIATE      0
#define DWT_START_TX_DELAYED        1
#define DWT_RESPONSE_EXPECTED       2

#define DWT_START_RX_IMMEDIATE  0
#define DWT_START_RX_DELAYED    1    // Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
#define DWT_IDLE_ON_DLY_ERR     2    // If delayed RX failed due to "late" error then if this
                                     // flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
#define DWT_NO_SYNC_PTRS        4    // Do not try to sync IC side and Host side buffer pointers when enabling RX. This is used to perform manual RX
                                     // re-enabling when receiving a frame in double buffer mode.

// Defined constants for "mode" bit field parameter passed to dwt_setleds() function.
#define DWT_LEDS_DISABLE     0x00
#define DWT_LEDS_ENABLE      0x01
#define DWT_LEDS_INIT_BLINK  0x02

//frame filtering configuration options
#define DWT_FF_NOTYPE_EN            0x000           // no frame types allowed (FF disabled)
#define DWT_FF_COORD_EN             0x002           // behave as coordinator (can receive frames with no dest address (PAN ID has to match))
#define DWT_FF_BEACON_EN            0x004           // beacon frames allowed
#define DWT_FF_DATA_EN              0x008           // data frames allowed
#define DWT_FF_ACK_EN               0x010           // ack frames allowed
#define DWT_FF_MAC_EN               0x020           // mac control frames allowed
#define DWT_FF_RSVD_EN              0x040           // reserved frame types allowed

//DW1000 interrupt events
#define DWT_INT_TFRS            0x00000080          // frame sent
#define DWT_INT_LDED            0x00000400          // micro-code has finished execution
#define DWT_INT_RFCG            0x00004000          // frame received with good CRC
#define DWT_INT_RPHE            0x00001000          // receiver PHY header error
#define DWT_INT_RFCE            0x00008000          // receiver CRC error
#define DWT_INT_RFSL            0x00010000          // receiver sync loss error
#define DWT_INT_RFTO            0x00020000          // frame wait timeout
#define DWT_INT_RXOVRR          0x00100000          // receiver overrun
#define DWT_INT_RXPTO           0x00200000          // preamble detect timeout
#define DWT_INT_SFDT            0x04000000          // SFD timeout
#define DWT_INT_ARFE            0x20000000          // frame rejected (due to frame filtering configuration)


//DW1000 SLEEP and WAKEUP configuration parameters
#define DWT_PRESRV_SLEEP 0x0100                      // PRES_SLEEP - on wakeup preserve sleep bit
#define DWT_LOADOPSET    0x0080                      // ONW_L64P - on wakeup load operating parameter set for 64 PSR
#define DWT_CONFIG       0x0040                      // ONW_LDC - on wakeup restore (load) the saved configurations (from AON array into HIF)
#define DWT_LOADEUI      0x0008                      // ONW_LEUI - on wakeup load EUI
#define DWT_RX_EN        0x0002                      // ONW_RX - on wakeup activate reception
#define DWT_TANDV        0x0001                      // ONW_RADC - on wakeup run ADC to sample temperature and voltage sensor values

#define DWT_XTAL_EN      0x10                       // keep XTAL running during sleep
#define DWT_WAKE_SLPCNT  0x8                        // wake up after sleep count
#define DWT_WAKE_CS      0x4                        // wake up on chip select
#define DWT_WAKE_WK      0x2                        // wake up on WAKEUP PIN
#define DWT_SLP_EN       0x1                        // enable sleep/deep sleep functionality

//DW1000 INIT configuration parameters
#define DWT_LOADUCODE     0x1
#define DWT_LOADNONE      0x0

//DW1000 OTP operating parameter set selection
#define DWT_OPSET_64LEN   0x0
#define DWT_OPSET_TIGHT   0x1
#define DWT_OPSET_DEFLT   0x2

// Call-back data RX frames flags
#define DWT_CB_DATA_RX_FLAG_RNG 0x1 // Ranging bit

// TX/RX call-back data
typedef struct
{
    uint32 status;      //initial value of register as ISR is entered
    uint16 datalength;  //length of frame
    uint8  fctrl[2];    //frame control bytes
    uint8  rx_flags;    //RX frame flags, see above
} dwt_cb_data_t;

// Call-back type for all events
typedef void (*dwt_cb_t)(const dwt_cb_data_t *);

/*! ------------------------------------------------------------------------------------------------------------------
 * Structure typedef: dwt_config_t
 *
 * Structure for setting device configuration via dwt_configure() function
 *
 */
typedef struct
{
    uint8 chan ;           //!< channel number {1, 2, 3, 4, 5, 7 }
    uint8 prf ;            //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
    uint8 txPreambLength ; //!< DWT_PLEN_64..DWT_PLEN_4096
    uint8 rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8 txCode ;         //!< TX preamble code
    uint8 rxCode ;         //!< RX preamble code
    uint8 nsSFD ;          //!< Boolean should we use non-standard SFD for better performance
    uint8 dataRate ;       //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
    uint8 phrMode ;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint16 sfdTO ;         //!< SFD timeout value (in symbols)
} dwt_config_t ;


typedef struct
{
    uint8   PGdly;
    //TX POWER
    //31:24     BOOST_0.125ms_PWR
    //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
    //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
    //7:0       DEFAULT_PWR-TX_DATA_PWR
    uint32  power;
}
dwt_txconfig_t ;


typedef struct
{

    uint16      maxNoise ;          // LDE max value of noise
    uint16      firstPathAmp1 ;     // Amplitude at floor(index FP) + 1
    uint16      stdNoise ;          // Standard deviation of noise
    uint16      firstPathAmp2 ;     // Amplitude at floor(index FP) + 2
    uint16      firstPathAmp3 ;     // Amplitude at floor(index FP) + 3
    uint16      maxGrowthCIR ;      // Channel Impulse Response max growth CIR
    uint16      rxPreamCount ;      // Count of preamble symbols accumulated
    uint16      firstPath ;         // First path index (10.6 bits fixed point integer)
}dwt_rxdiag_t ;


typedef struct
{
    //all of the below are mapped to a 12-bit register in DW1000
    uint16 PHE ;                    //number of received header errors
    uint16 RSL ;                    //number of received frame sync loss events
    uint16 CRCG ;                   //number of good CRC received frames
    uint16 CRCB ;                   //number of bad CRC (CRC error) received frames
    uint16 ARFE ;                   //number of address filter errors
    uint16 OVER ;                   //number of receiver overflows (used in double buffer mode)
    uint16 SFDTO ;                  //SFD timeouts
    uint16 PTO ;                    //Preamble timeouts
    uint16 RTO ;                    //RX frame wait timeouts
    uint16 TXF ;                    //number of transmitted frames
    uint16 HPW ;                    //half period warn
    uint16 TXW ;                    //power up warn

} dwt_deviceentcnts_t ;


/********************************************************************************************************************/
/*                                                 REMOVED API LIST                                                 */
/********************************************************************************************************************/
/*
 * From version 4.0.0:
 *  - dwt_setGPIOforEXTTRX: Replaced by dwt_setlnapamode to get equivalent functionality.
 *  - dwt_setGPIOdirection: Renamed to dwt_setgpiodirection.
 *  - dwt_setGPIOvalue: Renamed to dwt_setgpiovalue.
 *  - dwt_setrxmode: Replaced by dwt_setsniffmode and dwt_setlowpowerlistening depending on the RX mode the user
 *    wants to set up.
 *  - dwt_checkoverrun: As automatic RX re-enabling is not supported anymore, this functions has become useless.
 *  - dwt_setautorxreenable: As automatic RX re-enabling is not supported anymore, this functions has become
 *    useless.
 *  - dwt_getrangebias: Range bias correction values are platform dependent and should therefore be managed at user
 *    application level.
 *  - dwt_xtaltrim: Renamed to dwt_setxtaltrim.
 *  - dwt_checkIRQ: Renamed to dwt_checkirq.
 *
 * From version 3.0.0:
 *  - dwt_getldotune: As LDO loading is now automatically managed by the driver, this function has become useless.
 *  - dwt_getotptxpower: TX power values and location in OTP memory are platform dependent and should therefore be
 *    managed at user application level.
 *  - dwt_readantennadelay: Antenna delay values and location in OTP memory are platform dependent and should
 *    therefore be managed at user application level.
 *  - dwt_readdignostics: Renamed to dwt_readdiagnostics.
 */

/********************************************************************************************************************/
/*                                                     API LIST                                                     */
/********************************************************************************************************************/

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
int dwt_setlocaldataptr(unsigned int index);

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
uint32 dwt_getpartid(void);

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
uint32 dwt_getlotid(void);

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
uint32 dwt_readdevid(void);

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
uint8 dwt_otprevision(void);

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
void dwt_setfinegraintxseq(int enable);

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
void dwt_setlnapamode(int lna, int pa);

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
void dwt_setgpiodirection(uint32 gpioNum, uint32 direction);

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
void dwt_setgpiovalue(uint32 gpioNum, uint32 value);

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
int dwt_initialise(uint16 config) ;

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
void dwt_configure(dwt_config_t *config) ;

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
void dwt_configuretxrf(dwt_txconfig_t *config) ;

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
void dwt_setrxantennadelay(uint16 antennaDly);

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
void dwt_settxantennadelay(uint16 antennaDly);

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
void dwt_setsmarttxpower(int enable);

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
int dwt_writetxdata(uint16 txFrameLength, uint8 *txFrameBytes, uint16 txBufferOffset) ;

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
void dwt_writetxfctrl(uint16 txFrameLength, uint16 txBufferOffset, int ranging);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_starttx()
 *
 * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
 *
 * input parameters:
 * @param mode - is a bitmask for which the following values can be combined to define the operation
 *               DWT_START_TX_IMMEDIATE (0)            - to begin transmission immediatelty.
 *               DWT_START_TX_DELAYED   (to set bit 0) - to begin TX at pre-configured delay time
 *               DWT_RESPONSE_EXPECTED  (to set bit 1) - to turn the receiver on automatically (after the TX) after a pre-programmed delay
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed transmission will fail if the delayed time has passed)
 */
int dwt_starttx(uint8 mode) ;

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
void dwt_setdelayedtrxtime(uint32 starttime) ;

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
void dwt_readtxtimestamp(uint8 * timestamp);

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
uint32 dwt_readtxtimestamphi32(void);

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
uint32 dwt_readtxtimestamplo32(void);

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
void dwt_readrxtimestamp(uint8 * timestamp);

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
uint32 dwt_readrxtimestamphi32(void);

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
uint32 dwt_readrxtimestamplo32(void);

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
uint32 dwt_readsystimestamphi32(void);

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
void dwt_readsystime(uint8 * timestamp);

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
void dwt_forcetrxoff(void);

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
void dwt_syncrxbufptrs(void);

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
int dwt_rxenable(int mode);

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
void dwt_setsniffmode(int enable, uint8 timeOn, uint8 timeOff);

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
void dwt_setlowpowerlistening(int enable);

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
void dwt_setsnoozetime(uint8 snooze_time);

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
void dwt_setdblrxbuffmode(int enable);

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
void dwt_setrxtimeout(uint16 time);

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
void dwt_setpreambledetecttimeout(uint16 timeout);


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
uint16 dwt_calibratesleepcnt(void);

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
 void dwt_configuresleepcnt(uint16 sleepcnt);

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
void dwt_configuresleep(uint16 mode, uint8 wake);

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
void dwt_entersleep(void);

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
void dwt_entersleepaftertx(int enable);

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
int dwt_spicswakeup(uint8 *buff, uint16 length);

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
void dwt_setcallbacks(dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr);

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
uint8 dwt_checkirq(void);

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
void dwt_isr(void);

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
void dwt_lowpowerlistenisr(void);

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
void dwt_setinterrupt( uint32 bitmask, uint8 enable);

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
void dwt_setpanid(uint16 panID);

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
void dwt_setaddress16(uint16 shortAddress);

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
void dwt_seteui(uint8 *eui64);

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
void dwt_geteui(uint8 *eui64);

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
void dwt_otpread(uint32 address, uint32 *array, uint8 length);

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
void dwt_enableframefilter(uint16 bitmask);

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
void dwt_enableautoack(uint8 responseDelayTime);

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
void dwt_setrxaftertxdelay(uint32 rxDelayTime);

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
void dwt_rxreset(void);

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
void dwt_softreset(void) ;

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
void dwt_readrxdata(uint8 *buffer, uint16 length, uint16 rxBufferOffset);

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
void dwt_readaccdata(uint8 *buffer, uint16 length, uint16 rxBufferOffset);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readcarrierintegrator()
 *
 * @brief This is used to read the RX carrier integrator value (relating to the frequency offset of the TX node)
 *
 * input parameters - NONE
 *
 * return value - the (int32) signed carrier integrator value.
 *                A positive value means the local RX clock is running faster than the remote TX device.
 */

int32 dwt_readcarrierintegrator(void) ;

// Multiplication factors to convert carrier integrator value to a frequency offset in Hertz

#define FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)
#define FREQ_OFFSET_MULTIPLIER_110KB    (998.4e6/2.0/8192.0/131072.0)

// Multiplication factors to convert frequency offset in Hertz to PPM crystal offset
// NB: also changes sign so a positive value means the local RX clock is running slower than the remote TX device.

#define HERTZ_TO_PPM_MULTIPLIER_CHAN_1     (-1.0e6/3494.4e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_2     (-1.0e6/3993.6e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_3     (-1.0e6/4492.8e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_5     (-1.0e6/6489.6e6)

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
void dwt_readdiagnostics(dwt_rxdiag_t * diagnostics);

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
void dwt_loadopsettabfromotp(uint8 ops_sel);

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
void dwt_configeventcounters(int enable);

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
void dwt_readeventcounters(dwt_deviceentcnts_t *counters);

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
int dwt_otpwriteandverify(uint32 value, uint16 address);

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
void dwt_setleds(uint8 mode);

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
void dwt_setxtaltrim(uint8 value);

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
uint8 dwt_getinitxtaltrim(void);

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
void dwt_configcwmode(uint8 chan);

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
void dwt_configcontinuousframemode(uint32 framerepetitionrate);

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
uint16 dwt_readtempvbat(uint8 fastSPI);

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
uint8 dwt_readwakeuptemp(void) ;

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
uint8 dwt_readwakeupvbat(void) ;

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
uint32 dwt_calcbandwidthtempadj(uint16 target_count);

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
uint32 dwt_calcpowertempadj(uint8 channel, uint32 ref_powerreg, double current_temperature, double reference_temperature);

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
uint16 dwt_calcpgcount(uint8 pgdly);

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
    uint16      recordNumber,   // input parameter - ID of register file or buffer being accessed
    uint16      index,          // input parameter - byte index into register file or buffer being accessed
    uint32      length,         // input parameter - number of bytes being written
    const uint8 *buffer         // input parameter - pointer to buffer containing the 'length' bytes to be written
) ;

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
    uint16  recordNumber,       // input parameter - ID of register file or buffer being accessed
    uint16  index,              // input parameter - byte index into register file or buffer being accessed
    uint32  length,             // input parameter - number of bytes being read
    uint8   *buffer             // input parameter - pointer to buffer in which to return the read data.
) ;

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
uint32 dwt_read32bitoffsetreg(int regFileID, int regOffset) ;

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
void dwt_write32bitoffsetreg(int regFileID, int regOffset, uint32 regval);

#define dwt_write32bitreg(x,y)  dwt_write32bitoffsetreg(x,0,y)
#define dwt_read32bitreg(x)     dwt_read32bitoffsetreg(x,0)

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
uint16 dwt_read16bitoffsetreg(int regFileID, int regOffset);

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
void dwt_write16bitoffsetreg(int regFileID, int regOffset, uint16 regval) ;

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
uint8 dwt_read8bitoffsetreg(int regFileID, int regOffset);

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
void dwt_write8bitoffsetreg(int regFileID, int regOffset, uint8 regval);


/****************************************************************************************************************************************************
 *
 * Declaration of platform-dependent lower level functions.
 *
 ****************************************************************************************************************************************************/


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn writetospi()
 *
 * @brief
 * NB: In porting this to a particular microprocessor, the implementer needs to define the two low
 * level abstract functions to write to and read from the SPI the definitions should be in deca_spi.c file.
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 *
 * Note: The body of this function is defined in deca_spi.c and is platform specific
 *
 * input parameters:
 * @param headerLength  - number of bytes header being written
 * @param headerBuffer  - pointer to buffer containing the 'headerLength' bytes of header to be written
 * @param bodylength    - number of bytes data being written
 * @param bodyBuffer    - pointer to buffer containing the 'bodylength' bytes od data to be written
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn readfromspi()
 *
 * @brief
 * NB: In porting this to a particular microprocessor, the implementer needs to define the two low
 * level abstract functions to write to and read from the SPI the definitions should be in deca_spi.c file.
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 *
 * Note: The body of this function is defined in deca_spi.c and is platform specific
 *
 * input parameters:
 * @param headerLength  - number of bytes header to write
 * @param headerBuffer  - pointer to buffer containing the 'headerLength' bytes of header to write
 * @param readlength    - number of bytes data being read
 * @param readBuffer    - pointer to buffer containing to return the data (NB: size required = headerLength + readlength)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success (and the position in the buffer at which data begins), or DWT_ERROR for error
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);

// ---------------------------------------------------------------------------
//
// NB: The purpose of the deca_mutex.c file is to provide for microprocessor interrupt enable/disable, this is used for
//     controlling mutual exclusion from critical sections in the code where interrupts and background
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines here
//     to map these calls transparently to the target system.  Alternatively the appropriate code may
//     be embedded in the functions provided in the deca_irq.c file.
//
// ---------------------------------------------------------------------------

typedef int decaIrqStatus_t ; // Type for remembering IRQ status


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn decamutexon()
 *
 * @brief This function should disable interrupts. This is called at the start of a critical section
 * It returns the IRQ state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
decaIrqStatus_t decamutexon(void) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn decamutexoff()
 *
 * @brief This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn deca_sleep()
 *
 * @brief Wait for a given amount of time.
 * NB: The body of this function is defined in deca_sleep.c and is platform specific
 *
 * input parameters:
 * @param time_ms - time to wait in milliseconds
 *
 * output parameters
 *
 * no return value
 */
void deca_sleep(unsigned int time_ms);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_DEVICE_API_H_ */


