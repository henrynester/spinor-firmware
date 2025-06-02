//#include "dsdl/dsdl_generated/include/dronecan_msgs.h"
#include "controller.h"
#include "ControllerStateMachine.h"
#include "libcanard/canard.h"

#include "isr.h"

#define CAN_BITRATE 1000000

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define APP_VERSION_MAJOR                                           0 
#define APP_VERSION_MINOR                                           1 
#define APP_NODE_NAME                                               "SPINOR"
#define GIT_HASH                                                    0

#define UNIQUE_ID_LENGTH_BYTES                                      16

void dronecan_init(controller_in_t* controller_input_port);
void dronecan_transmit(void);
void dronecan_receive(void);
void dronecan_publish_NodeStatus(void);
void dronecan_publish_SPINORFeedback(controller_out_t* controller_output_port);
void dronecan_publish_SPINORStatus(controller_out_t* controller_output_port,
		CSM_t *csm);
void dronecan_publish_debug_KeyValue(char* key, float value, uint8_t* transfer_id);
ControllerEventType_t dronecan_event_pop(void);
