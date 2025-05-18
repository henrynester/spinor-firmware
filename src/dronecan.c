#include "dronecan.h"

#include "config.h"
#include "thermistor.h"

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libcanard/canard.h"
#include "libcanard/drivers/stm32/canard_stm32.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo.h"
#include "uavcan.protocol.debug.KeyValue.h"
#include "uavcan.protocol.param.GetSet.h"

#include <string.h>

#define CANARD_SPIN_PERIOD    500
#define PUBLISHER_PERIOD_mS   25


static CanardInstance g_canard;                //The library instance
static uint8_t g_canard_memory_pool[1024];     //Arena for memory allocation, used by the library

static struct uavcan_protocol_NodeStatus node_status_msg;
static struct uavcan_protocol_debug_KeyValue key_value_msg;

//////////////////////////////////////////////////////////////////////////////////////

void dronecan_handle_param_GetSet(CanardInstance* ins, CanardRxTransfer* transfer);

bool shouldAcceptTransfer(const CanardInstance* ins,
                          uint64_t* out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id)
{
  if (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) {
    *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////

void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
  if (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) {
    dronecan_handle_param_GetSet(ins, transfer);
  }
}

void dronecan_init(void)
{

  CanardSTM32CANTimings timings;
  int result = canardSTM32ComputeCANTimings(rcc_apb1_frequency, CAN_BITRATE, &timings);
  if (result) {
	  while(1);
  }
  result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
  if (result) {
          while(1);
  }
 
  canardInit(&g_canard,                         // Uninitialized library instance
             g_canard_memory_pool,              // Raw memory chunk used for dynamic allocation
             sizeof(g_canard_memory_pool),      // Size of the above, in bytes
             onTransferReceived,                // Callback, see CanardOnTransferReception
             shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
             NULL);
 
  canardSetLocalNodeID(&g_canard, (uint8_t)0x42);
}


void dronecan_transmit(void)
{

  const CanardCANFrame* txf = canardPeekTxQueue(&g_canard); 
  uint32_t t_start = g_uptime_ms;
  while(txf && g_uptime_ms < (t_start + 10)) {
    const int tx_res = canardSTM32Transmit(txf);
    if (tx_res < 0) { //Failure - drop the frame and report
	    while(1);
    }
    if(tx_res > 0) { //Success: transmit next frame
      canardPopTxQueue(&g_canard);
      txf = canardPeekTxQueue(&g_canard); 
    }
    //tx_res=0 means we don't have a tx mailbox available
    //block in this loop until one becomes available
    //causes: If disconnected from all other nodes, we get no ACKs and
    //will keep trying to transmit what's in the mailbox forever
    //OR, bus utilization is too high and we can't find time to
    //transmit our lower priority message.
    else {
	    break;
    }
  }

}


void dronecan_receive(void)
{
  CanardCANFrame rx_frame;
  int res = canardSTM32Receive(&rx_frame);
  if(res) {
    canardHandleRxFrame(&g_canard, &rx_frame, g_uptime_ms*1000);
  } 
}

void dronecan_publish_NodeStatus(void)
{  
  static uint32_t spin_time = 0;

  if(g_uptime_ms <= spin_time + CANARD_SPIN_PERIOD) return;    // rate limiting
  spin_time = g_uptime_ms;
  
  gpio_toggle(GPIOA, GPIO4);
    
  node_status_msg.uptime_sec = (g_uptime_ms / 1000); 
  node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  node_status_msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

  uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];    
  static uint8_t transfer_id = 0;                               // This variable MUST BE STATIC; refer to the libcanard documentation for the background
  uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status_msg, buffer);
  canardBroadcast(&g_canard, 
                  UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                  UAVCAN_PROTOCOL_NODESTATUS_ID,
                  &transfer_id,
                  CANARD_TRANSFER_PRIORITY_LOW,
                  buffer, 
                  len);             //some indication
								    //
}

void dronecan_publish_debug_KeyValue(char* key, float value, uint8_t* transfer_id) {
	strcpy((char*)key_value_msg.key.data, key);
	key_value_msg.key.len = 2;
	key_value_msg.value = value;
	uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MAX_SIZE];
	uint32_t len = uavcan_protocol_debug_KeyValue_encode(&key_value_msg, buffer);
	canardBroadcast(&g_canard, 
			UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE,
			UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID,
			transfer_id,
			CANARD_TRANSFER_PRIORITY_LOW,
			buffer, 
			len);
}

parameter_t parameters[] = {
	{"vd_ref", &parameter_storage.vd_ref}, 
	{"vq_ref", &parameter_storage.vq_ref}, 
	{"id_ref", &parameter_storage.id_ref}, 
	{"iq_ref", &parameter_storage.iq_ref},
	{"omega_m_ref", &parameter_storage.omega_m_ref},
	{"theta_m_ref", &parameter_storage.theta_m_ref} 
};

parameter_storage_t parameter_storage;

void dronecan_handle_param_GetSet(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    parameter_t *p = NULL;
    if (req.name.len != 0) {
        for (uint16_t i=0; i<ARRAY_SIZE(parameters); i++) {
            if (req.name.len == strlen(parameters[i].name) &&
                strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0) {
                p = &parameters[i];
                break;
            }
        }
    } else if (req.index < ARRAY_SIZE(parameters)) {
        p = &parameters[req.index];
    }
    if (p != NULL && req.name.len != 0 && req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
        /*
          this is a parameter set command. The implementation can
          either choose to store the value in a persistent manner
          immediately or can instead store it in memory and save to permanent storage on a
         */
            *(p->value) = req.value.real_value;
    }

    /*
      for both set and get we reply with the current value
     */
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));
    	if (p != NULL) {
            pkt.value.real_value = *p->value;
	    pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
		pkt.name.len = strlen(p->name);
		strcpy((char *)pkt.name.data, p->name);
        } else {
		pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
	}
	pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
	pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
	pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;


    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}
