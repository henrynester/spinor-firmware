#include "uavcan.protocol.GetNodeInfo_res.h"
//#include "dsdl/dsdl_generated/include/dronecan_msgs.h"
#include "libcanard/canard.h"

#include "isr.h"

#define CAN_BITRATE 1000000

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define APP_VERSION_MAJOR                                           0 
#define APP_VERSION_MINOR                                           1 
#define APP_NODE_NAME                                               "spinor"
#define GIT_HASH                                                    0

#define UNIQUE_ID_LENGTH_BYTES                                      16

void dronecan_init(isr_in_t *isr_in_, isr_out_t *isr_out_, uint8_t *ignore_cmds);
void dronecan_transmit(void);
void dronecan_receive(void);
void dronecan_publish_NodeStatus(void);
void dronecan_publish_debug_KeyValue(char* key, float value, uint8_t* transfer_id);
