#ifndef RF_DEFAULTS_H_
#define RF_DEFAULTS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Maximum possible packet size in byte
 */
#define RF_MAX_PKT_LENGTH        (127)

/**
 * @brief   Default addresses used if the CPUID module is not present
 * @{
 */
#define RF_DEFAULT_ADDR_SHORT    (0x0230)
#define RF_DEFAULT_ADDR_LONG     (0x1222334455667788)
/** @} */

/**
  * @brief   Channel configuration
  * @{
  */
//#ifdef MODULE_AT86RF212B
/* the AT86RF212B has a sub-1GHz radio */
//#define RF_MIN_CHANNEL           (0)
//#define RF_MAX_CHANNEL           (10)
//#define RF_DEFAULT_CHANNEL       (5)
//#else
#define RF_MIN_CHANNEL           (11U)
#define RF_MAX_CHANNEL           (26U)
#define RF_DEFAULT_CHANNEL       (11U)
//#endif
/** @} */

/**
 * @brief   Default PAN ID
 *
 * @todo    Read some global network stack specific configuration value
 */
#define RF_DEFAULT_PANID         (0x0023)

/**
 * @brief   Default TX power (0dBm)
 */
#define RF_DEFAULT_TXPOWER       (0U)

/**
 * @brief   Flags for device internal states (see datasheet)
 * @{
 */
#define RF_STATE_TRX_OFF      (0x08)     /**< idle */
#define RF_STATE_PLL_ON       (0x09)     /**< ready to transmit */
#define RF_STATE_SLEEP        (0x0f)     /**< sleep mode */
#define RF_STATE_DEEP_SLEEP   (0x10)     /**< deep sleep mode */
#define RF_STATE_BUSY_RX_AACK (0x11)     /**< busy receiving data */
#define RF_STATE_BUSY_TX_ARET (0x12)     /**< busy transmitting data */
#define RF_STATE_RX_AACK_ON   (0x16)     /**< wait for incoming data */
#define RF_STATE_TX_ARET_ON   (0x19)     /**< ready for sending data */
#define RF_STATE_IN_PROGRESS  (0x1f)     /**< ongoing state conversion */
/** @} */

/**
 * @brief   Internal device option flags
 * @{
 */
#define RF_OPT_AUTOACK        (0x0001)       /**< auto ACKs active */
#define RF_OPT_CSMA           (0x0002)       /**< CSMA active */
#define RF_OPT_PROMISCUOUS    (0x0004)       /**< promiscuous mode
                                                     *   active */
#define RF_OPT_PRELOADING     (0x0008)       /**< preloading enabled */
#define RF_OPT_TELL_TX_START  (0x0010)       /**< notify MAC layer on TX
                                                     *   start */
#define RF_OPT_TELL_TX_END    (0x0020)       /**< notify MAC layer on TX
                                                     *   finished */
#define RF_OPT_TELL_RX_START  (0x0040)       /**< notify MAC layer on RX
                                                     *   start */
#define RF_OPT_TELL_RX_END    (0x0080)       /**< notify MAC layer on RX
                                                     *   finished */
#define RF_OPT_RAWDUMP        (0x0100)       /**< pass RAW frame data to
                                                     *   upper layer */
#define RF_OPT_SRC_ADDR_LONG  (0x0200)       /**< send data using long
                                                     *   source address */
#define RF_OPT_USE_SRC_PAN    (0x0400)       /**< do not compress source
                                                     *   PAN ID */
/** @} */

/**
  * @brief   Frequency configuration for sub-GHz devices.
  * @{
  */
//typedef enum {
//    RF_FREQ_915MHZ,       /**< frequency 915MHz enabled */
//    RF_FREQ_868MHZ,       /**< frequency 868MHz enabled */
//} rf_freq_t;
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* RF_DEFAULTS_H_ */
