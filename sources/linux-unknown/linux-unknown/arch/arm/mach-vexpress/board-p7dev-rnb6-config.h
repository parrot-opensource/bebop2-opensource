/*
 * Configuration of RNB6 board
 * For camera slot (CAM1 on connector J800/J801), define:
 * -Camera type CAM1_TYPE: choose one of following
 * 		SOC_CAM_TYPE_MT9F002,
 * 		SOC_CAM_TYPE_MT9V117,
 * 		SOC_CAM_TYPE_MT9M114,
 * 		SOC_CAM_TYPE_AS0260
 * -Color space converter CAM1_CONV_NODE_NUMBER: possible choices:
 * 		-1: no converter used
 * 		0, 1, 2, 3: use converter 0, 1, 2, 3
 * -Synchronization type CAM1_SYNCHRO_TYPE: possible choices:
 * 		CAM_SYNCHRO_ITU_BT656 for embedded synchronization signals
 * 		CAM_SYNCHRO_ITU_BT601 for hardware signals HS/VS
 * -Synchronization signals (HS/VS) wiring
 * 		define CAM1_SYNCHRO_USE_HS_VS to use them (conflict with LCD0 synchro)
 * 		un-define CAM1_SYNCHRO_USE_HS_VS to not use them
 */
/* assignment of type of camera on slot 1 J800/J801*/
#define CAM1_TYPE SOC_CAM_TYPE_MT9F002
//#define CAM1_TYPE SOC_CAM_TYPE_MT9V117
//#define CAM1_TYPE SOC_CAM_TYPE_MT9M114
//#define CAM1_TYPE SOC_CAM_TYPE_AS0260

/* assignment of color space converter */
#define CAM1_CONV_NODE_NUMBER -1

/* assignment of synchro type */
//#define CAM1_SYNCHRO_TYPE CAM_SYNCHRO_ITU_BT656
#define CAM1_SYNCHRO_TYPE CAM_SYNCHRO_ITU_BT601

#define CAM1_SYNCHRO_USE_HS_VS 1
//#undef CAM1_SYNCHRO_USE_HS_VS

#define CAM1_OVERLAY_FB 1
//#undef CAM1_OVERLAY_FB 1

