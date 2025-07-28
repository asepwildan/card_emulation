#ifndef __RFAL_DEFAULT_CONFIG_H__
#define __RFAL_DEFAULT_CONFIG_H__

#define RFAL_FEATURE_LISTEN_MODE 1

#define RFAL_FEATURE_NFCA 1
#define RFAL_FEATURE_NFCB 1
#define RFAL_FEATURE_NFCF 1
#define RFAL_FEATURE_NFCV 1
#define RFAL_FEATURE_ST25TB 1
#define RFAL_FEATURE_NFC_DEP 1
#define RFAL_FEATURE_ISO_DEP 1

#define RFAL_SUPPORT_MODE_POLL_NFCA 1
#define RFAL_SUPPORT_MODE_POLL_NFCB 0
#define RFAL_SUPPORT_MODE_POLL_NFCF 0
#define RFAL_SUPPORT_MODE_POLL_NFCV 0
#define RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P 0

#define RFAL_SUPPORT_MODE_LISTEN_NFCA 1
#define RFAL_SUPPORT_MODE_LISTEN_NFCB 0
#define RFAL_SUPPORT_MODE_LISTEN_NFCF 0
#define RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P 0
#define RFAL_FEATURE_ISO_DEP_LISTEN 1
#define RFAL_FEATURE_ISO_DEP_POLL 1

#define RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN    256U       /*!< ISO-DEP I-Block max length. Please use values as defined by rfalIsoDepFSx */
#define RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN      512U       /*!< ISO-DEP APDU max length. Please use multiples of I-Block max length       */

/*******************************************************************************/
#define RFAL_SUPPORT_BR_CE_A_106                    true         /*!< RFAL CE A 106 Bit Rate support switch  */
#define RFAL_SUPPORT_BR_CE_A_212                    false        /*!< RFAL CE A 212 Bit Rate support switch  */
#define RFAL_SUPPORT_BR_CE_A_424                    false        /*!< RFAL CE A 424 Bit Rate support switch  */
#define RFAL_SUPPORT_BR_CE_A_848                    false        /*!< RFAL CE A 848 Bit Rate support switch  */

#endif
