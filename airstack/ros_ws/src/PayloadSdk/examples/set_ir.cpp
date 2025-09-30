#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <iostream>

#include"payloadSdkInterface.h"

#if (CONTROL_METHOD == CONTROL_UART)
T_ConnInfo s_conn = {
    CONTROL_UART,
    payload_uart_port,
    payload_uart_baud
};
#else
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};
#endif

PayloadSdkInterface* my_payload = nullptr;

void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);

typedef enum{
	idle = 0,
	check_connection,
	set_ir_view,
	ir_view_active
} camera_mode_sequence_t;

camera_mode_sequence_t current_state = idle;

int main(int argc, char *argv[]){
	printf("Starting IR mode setting example...\n");
	signal(SIGINT,quit_handler);

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// check connection
	my_payload->checkPayloadConnection();
	
	printf("Setting camera view source to IR (value: %d)\n", PAYLOAD_CAMERA_VIEW_IR);
	// Set view source to IR only (value )
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
	
	printf("Setting record source to IR (value: %d)\n", PAYLOAD_CAMERA_RECORD_IR);
	// Set record source to IR only (value )
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RECORD_SRC, PAYLOAD_CAMERA_RECORD_IR, PARAM_TYPE_UINT32);
	
	current_state = ir_view_active;
	
	// Main loop - just stay in IR mode
	while(1){
		printf("Camera set to IR mode - maintaining...\n");
		usleep(3000000); // Sleep 3 seconds between messages
	}

    
	return 0;
}

void quit_handler( int sig ){
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    // end program here
    exit(0);
}


void onPayloadStatusChanged(int event, double* param){
	printf("Event received: %d\n", event);
	
	switch(event){
	case PAYLOAD_CAM_SETTINGS:
		// param[0]: mode_id
		// param[1]: zoomLevel
		// param[2]: focusLevel
		printf("Got camera settings: Mode: %.0f, Zoom: %.2f, Focus: %.2f\n", 
			   param[0], param[1], param[2]);
		break;
		
	case PAYLOAD_GB_ATTITUDE:
		// Gimbal attitude events (frequent, so minimizing output)
		printf("Gimbal attitude update\n");
		break;
		
	default:
		printf("Other event type: %d with values: %.2f, %.2f, %.2f, %.2f\n",
			  event, param[0], param[1], param[2], param[3]);
		break;
	}
}