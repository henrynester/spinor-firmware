#include "dronecan.h"
#include "pins.h"

#include "flash.h"
#include "config.h"
#include "thermistor.h"
#include "constants.h"
#include "ControllerStateMachine.h"

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libcanard/canard.h"
#include "libcanard/drivers/stm32/canard_stm32.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.debug.KeyValue.h"
#include "uavcan.protocol.param.GetSet.h"
#include "uavcan.protocol.param.ExecuteOpcode.h"
#include "uavcan.equipment.safety.ArmingStatus.h"
#include "local.SPINORArrayCommand.h"
#include "local.SPINORFeedback.h"
#include "local.SPINORStatus.h"
#include "local.SPINORExecuteOpcode.h"

#include <string.h>

#define CANARD_SPIN_PERIOD    500
#define PUBLISHER_PERIOD_mS   25

ControllerEventType_t dronecan_event;
static CanardInstance g_canard;                //The library instance
static uint8_t g_canard_memory_pool[1024];     //Arena for memory allocation, used by the library

static struct uavcan_protocol_NodeStatus node_status_msg;
static struct local_SPINORStatus status_msg;
static struct uavcan_protocol_debug_KeyValue key_value_msg;

static config_t *config; 

static controller_in_t *isr_in;

//////////////////////////////////////////////////////////////////////////////////////

void dronecan_handle_param_GetSet(CanardInstance* ins,
	CanardRxTransfer* transfer);
void dronecan_handle_SPINORArrayCommand(CanardInstance* ins,
	CanardRxTransfer* transfer);
void dronecan_handle_equipment_safety_ArmingStatus(CanardInstance* ins, 
	CanardRxTransfer* transfer);
void dronecan_handle_SPINORExecuteOpcode(CanardInstance* ins, 
		CanardRxTransfer* transfer);
void dronecan_handle_param_ExecuteOpcode(CanardInstance* ins, 
		CanardRxTransfer* transfer);

static bool shouldAcceptTransfer(const CanardInstance* ins,
                          uint64_t* out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id)
{
  if (data_type_id == LOCAL_SPINORARRAYCOMMAND_ID) {
	  *out_data_type_signature = LOCAL_SPINORARRAYCOMMAND_SIGNATURE;
	  return true;
  }
  if(data_type_id == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID) {
	  *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
	  return true;
  }
  if (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) {
    *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
    return true;
  }
  if (data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID) {
    *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
    return true;
  }
  if(data_type_id == LOCAL_SPINOREXECUTEOPCODE_ID) {
	  *out_data_type_signature = LOCAL_SPINOREXECUTEOPCODE_SIGNATURE;
	  return true;
  }

  return false;
}

//////////////////////////////////////////////////////////////////////////////////////

static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
	if(transfer->data_type_id == LOCAL_SPINORARRAYCOMMAND_ID) {
		dronecan_handle_SPINORArrayCommand(ins, transfer);
		return;
	}
	if(transfer->data_type_id == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID) {
		dronecan_handle_equipment_safety_ArmingStatus(ins, transfer);
		return;
	}
	if (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) {
		dronecan_handle_param_GetSet(ins, transfer);
		return;
	}
	if (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID) {
		dronecan_handle_param_ExecuteOpcode(ins, transfer);
		return;
	}
	if(transfer->data_type_id == LOCAL_SPINOREXECUTEOPCODE_ID) {
		dronecan_handle_SPINORExecuteOpcode(ins, transfer);
		return;
	}
}

void dronecan_init(controller_in_t *isr_in_)
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
 
	if(config == NULL) {
		config = config_get();
	}

	isr_in = isr_in_;

  canardSetLocalNodeID(&g_canard, (uint8_t)(config_get()->node_id));
}


void dronecan_transmit(void)
{

  const CanardCANFrame* txf = canardPeekTxQueue(&g_canard); 
  uint32_t t_start = g_uptime_ms;
  while(txf && g_uptime_ms < t_start+1) {
    const int tx_res = canardSTM32Transmit(txf);
    if (tx_res < 0) { //Failure - drop the frame and report
	    while(1);
    }
    else if(tx_res > 0) { //Success: transmit next frame
      canardPopTxQueue(&g_canard);
      txf = canardPeekTxQueue(&g_canard); 
    }
    //tx_res=0 means we don't have a tx mailbox available
    //block in this loop until one becomes available
    //causes: If disconnected from all other nodes, we get no ACKs and
    //will keep trying to transmit what's in the mailbox forever
    //OR, bus utilization is too high and we can't find time to
    //transmit our lower priority message.
  }

}


void dronecan_receive(void)
{
	CanardCANFrame rx_frame;
	uint32_t t_start = g_uptime_ms;
			//gpio_clear(LED_PORT, LED_B);
	while(g_uptime_ms < t_start + 1) {
		int res = canardSTM32Receive(&rx_frame);
		if(res) {
		  canardHandleRxFrame(&g_canard, &rx_frame, g_uptime_ms*1000);
		} 
	}
		  //gpio_set(LED_PORT, LED_B);
}

void dronecan_publish_NodeStatus(void)
{  
  static uint32_t spin_time = 0;

  if(g_uptime_ms <= spin_time + CANARD_SPIN_PERIOD) return;    // rate limiting
  spin_time = g_uptime_ms;
  
  //gpio_toggle(GPIOA, GPIO4);
    
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

void dronecan_handle_param_GetSet(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    parameter_t *p = NULL;
    if (req.name.len != 0) {
	    p = config_get_param_by_name((char*)req.name.data, req.name.len);
    } else { 
        p = config_get_param_by_idx(req.index);
    }
    if (p != NULL && req.name.len != 0 && req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
        /*
          this is a parameter set command. The implementation can
          either choose to store the value in a persistent manner
          immediately or can instead store it in memory and save to permanent storage on a
         */
            *(p->value) = req.value.real_value;
	    //updates internal int config values from external float values just set
	    config_from_params();
	    //updates external values to reflecting rounding on int conversion
	    config_to_params();
    }

    /*
      for both set and get we reply with the current value
     */
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));
    	if (p != NULL) {
            pkt.value.real_value = *p->value;
	    pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
	    pkt.default_value.real_value = p->default_value;
	    pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
	    pkt.min_value.real_value = p->min_value;
	    pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
	    pkt.max_value.real_value = p->max_value;
	    pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
		pkt.name.len = strlen(p->name);
		strcpy((char *)pkt.name.data, p->name);
        } else {
		pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
		pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
		pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
		pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
	}


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
	 
void dronecan_handle_SPINORArrayCommand(CanardInstance *ins,
		CanardRxTransfer *transfer) {
	static struct local_SPINORArrayCommand arrcmd;
	if(local_SPINORArrayCommand_decode(transfer, &arrcmd)) {
		return;
	}
	if(config->actuator_index >= arrcmd.commands.len) {
	       return;
	}	       
	//only take commands while in ARMED state
	if(!status_msg.armed) {
		return;
	}
	
	switch(arrcmd.commands.data[config->actuator_index].command_type) {
		case LOCAL_SPINORCOMMAND_COMMAND_TYPE_TORQUE:
			isr_in->iq_ref = 
				arrcmd.commands.data[config->actuator_index].command
				/ TORQUE_IDQ_LSB;
			isr_in->control_mode = CONTROL_MODE_TORQUE;
			break;
		case LOCAL_SPINORCOMMAND_COMMAND_TYPE_VELOCITY:
			isr_in->omega_m_ref 
				= arrcmd.commands.data[config->actuator_index].command
				/ OMEGA_M_LSB;
			isr_in->control_mode = CONTROL_MODE_VEL;
			break;
		case LOCAL_SPINORCOMMAND_COMMAND_TYPE_POSITION:
			isr_in->theta_m_ref
				= arrcmd.commands.data[config->actuator_index].command
				/ THETA_M_LSB;
			isr_in->control_mode = CONTROL_MODE_POS;
			break;
	}
}

void dronecan_handle_param_ExecuteOpcode(CanardInstance* ins,
	CanardRxTransfer* transfer) {
	struct uavcan_protocol_param_ExecuteOpcodeRequest request;
	if(uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &request)) {
		return;
	}

	uint8_t error = 0;
	if(request.opcode == 
			UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
		error = config_save();
	} else if(request.opcode ==
			UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
		config_defaults();
	}

	struct uavcan_protocol_param_ExecuteOpcodeResponse response;
	memset(&response, 0, sizeof(response));
	response.ok = (error) ? false : true;

	uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
	uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(
		&response, buffer);

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


void dronecan_handle_equipment_safety_ArmingStatus(CanardInstance* ins, 
	CanardRxTransfer* transfer) {
	static struct uavcan_equipment_safety_ArmingStatus msg;
	if(uavcan_equipment_safety_ArmingStatus_decode(transfer, &msg)) {
		return;
	}
	//fire arm or disarm events for FSM to receive
	if(msg.status == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED) {
		dronecan_event = EVENT_ARM;
		//gpio_toggle(LED_PORT, LED_R);
	} else {
		dronecan_event = EVENT_DISARM;
	}
}

void dronecan_handle_SPINORExecuteOpcode(CanardInstance* ins, 
	CanardRxTransfer* transfer) {
	struct local_SPINORExecuteOpcodeRequest request;
	if(local_SPINORExecuteOpcodeRequest_decode(transfer, &request)) {
		return;
	}
	//fire homing, calibration events for FSM to receive
	if(request.opcode 
		== LOCAL_SPINOREXECUTEOPCODE_REQUEST_OPCODE_START_HOMING) {
		dronecan_event = EVENT_HOMING;
	} else if(request.opcode
		== LOCAL_SPINOREXECUTEOPCODE_REQUEST_OPCODE_START_ENCODER_CALIBRATION) {
		dronecan_event = EVENT_CALIBRATE_ENCODER;
	}
}

ControllerEventType_t dronecan_event_pop(void) {
	ControllerEventType_t temp = dronecan_event;
	dronecan_event = EVENT_DEFAULT; // clear events
	return temp;
}

void dronecan_publish_SPINORFeedback(controller_out_t* controller_output_port) {
	static uint32_t t_last_fb = 0;
	if(g_uptime_ms <= t_last_fb + 100) return;
	t_last_fb = g_uptime_ms;

	static struct local_SPINORFeedback msg;
	msg.position = (float)controller_output_port->theta_m * THETA_M_LSB;
	msg.velocity = (float)controller_output_port->omega_m * OMEGA_M_LSB;
	msg.torque = (float)controller_output_port->iq * TORQUE_IDQ_LSB;

	uint8_t buffer[LOCAL_SPINORFEEDBACK_MAX_SIZE];    
	static uint8_t transfer_id = 0;
	uint32_t len = local_SPINORFeedback_encode(&msg, buffer);
	canardBroadcast(&g_canard, 
			LOCAL_SPINORFEEDBACK_SIGNATURE,
			LOCAL_SPINORFEEDBACK_ID,
			&transfer_id,
			CANARD_TRANSFER_PRIORITY_MEDIUM,
			buffer, 
			len);             //some indication
}

void dronecan_publish_SPINORStatus(controller_out_t* controller_output_port,
		CSM_t *csm) {
	static uint32_t t_last_status = 0;
	if(g_uptime_ms <= t_last_status + 100) return;
	t_last_status = g_uptime_ms;


	status_msg.T_fet = thermistor_temperature_from_adc(
		controller_output_port->T_fet);
	status_msg.T_mtr = thermistor_temperature_from_adc(
		controller_output_port->T_mtr);
	status_msg.foc_deadline_perc = 
	   100.0*(float)controller_output_port->t_exec_foc / ((float)PWM_DT*1.0e6);
	status_msg.controller_cpu_perc =
	   100.0*(float)controller_output_port->t_exec_controller 
	   / ((float)CONTROL_DT*1.0e6);
	status_msg.v_bus = controller_output_port->v_bus * VBUS_LSB;
	status_msg.error = csm->error;
	status_msg.armed = (csm->state == CONTROLLERSTATE_ARMED) ? true : false;	

	uint8_t buffer[LOCAL_SPINORSTATUS_MAX_SIZE];    
	static uint8_t transfer_id = 0;
	uint32_t len = local_SPINORStatus_encode(&status_msg, buffer);
	canardBroadcast(&g_canard, 
			LOCAL_SPINORSTATUS_SIGNATURE,
			LOCAL_SPINORSTATUS_ID,
			&transfer_id,
			CANARD_TRANSFER_PRIORITY_MEDIUM,
			buffer, 
			len);             //some indication
}
