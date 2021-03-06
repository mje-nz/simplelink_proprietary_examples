#ifndef _SMARTRF_SETTINGS_H_
#define _SMARTRF_SETTINGS_H_

//*********************************************************************************
// Generated by SmartRF Studio version 2.12.0 (build#147)
// The applied template is compatible with CC13x2 SDK 2.40.xx.xx
// Device: CC1352R Rev. 2.1
//
//*********************************************************************************
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include <ti/drivers/rf/RF.h>


// TI-RTOS RF Mode Object
extern RF_Mode RF_prop;

// RF Core API commands
extern rfc_CMD_NOP_t RF_cmdNop;
extern rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup;
extern rfc_CMD_FS_t RF_cmdFs;
extern rfc_CMD_PROP_CS_t RF_cmdPropCs;
extern rfc_CMD_PROP_TX_t RF_cmdPropTx;
extern rfc_CMD_COUNT_BRANCH_t RF_cmdCountBranch;
extern rfc_CMD_PROP_RX_t RF_cmdPropRx;

// RF Core API Overrides
extern uint32_t pOverrides[];

#endif // _SMARTRF_SETTINGS_H_
