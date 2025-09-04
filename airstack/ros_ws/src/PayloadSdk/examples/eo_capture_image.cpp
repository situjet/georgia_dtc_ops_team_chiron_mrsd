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
	check_storage,
	check_capture_status,
	check_camera_mode,
	change_camera_mode,
	do_capture,
	wait_capture_done,
}capture_sequence_t;

capture_sequence_t my_capture = idle;
uint8_t image_to_capture = 3;

int main(int argc, char *argv[]){
	printf("Starting CaptureImage example...\n");
	signal(SIGINT,quit_handler);

	// Print camera mode constants for debugging
	printf("=== Camera Mode Constants ===\n");
	printf("CAMERA_MODE_VIDEO = %d\n", CAMERA_MODE_VIDEO);
	printf("CAMERA_MODE_IMAGE = %d\n", CAMERA_MODE_IMAGE);
	printf("=== Record Source Constants ===\n");
	printf("PAYLOAD_CAMERA_RECORD_EO = %d\n", PAYLOAD_CAMERA_RECORD_EO);
	printf("PAYLOAD_CAMERA_RECORD_IR = %d\n", PAYLOAD_CAMERA_RECORD_IR);
	// If other modes exist, print them too (uncomment as needed)
	// printf("PAYLOAD_CAMERA_RECORD_EOIR = %d\n", PAYLOAD_CAMERA_RECORD_EOIR);
	// printf("PAYLOAD_CAMERA_RECORD_IREO = %d\n", PAYLOAD_CAMERA_RECORD_IREO);
	printf("=============================\n");

	// create payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();
	printf("Waiting for payload signal! \n");

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

	// check connection
	my_payload->checkPayloadConnection();
	
	// set payload to video mode for testing
	printf("Setting camera mode to VIDEO (value: %d)\n", CAMERA_MODE_VIDEO);
	my_payload->setPayloadCameraMode(CAMERA_MODE_VIDEO);
	
	printf("Setting record source to EO (value: %d)\n", PAYLOAD_CAMERA_RECORD_EO);
	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RECORD_SRC, PAYLOAD_CAMERA_RECORD_EO, PARAM_TYPE_UINT32);

	// Query available camera modes
	printf("Querying available camera modes...\n");
	my_payload->getPayloadCameraMode();
	
	// Query available record sources
	printf("Checking current record source...\n");
	// my_payload->getPayloadCameraParam(PAYLOAD_CAMERA_RECORD_SRC);
	my_payload->getPayloadCameraMode();
	my_capture = check_storage;
	while(1){
		// to caputre image with payload, follow this sequence
		switch(my_capture){
		case idle:{
			// do nothing;
			break;
		}
		case check_storage:{
			my_payload->getPayloadStorage();
			break;
		}
		case check_capture_status:{
			my_payload->getPayloadCaptureStatus();
			break;
		}
		case check_camera_mode:{
			printf("Checking current camera mode...\n");
			my_payload->getPayloadCameraMode();
			break;
		}
		case change_camera_mode:{
			printf("Changing to IMAGE mode (value: %d)...\n", CAMERA_MODE_IMAGE);
			my_payload->setPayloadCameraMode(CAMERA_MODE_IMAGE);
			my_capture = check_camera_mode;
			break;
		}
		case do_capture:{
			my_payload->setPayloadCameraCaptureImage();
			my_capture = wait_capture_done;
			break;
		}
		case wait_capture_done:{
			my_payload->getPayloadCaptureStatus();
			break;
		}
		default: break;
		}

		usleep(300000); // sleep 0.3s
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
	case PAYLOAD_CAM_CAPTURE_STATUS:{
		// param[0]: image_status
		// param[1]: video_status
		// param[2]: image_count
		// param[3]: recording_time_ms

		if(my_capture == check_capture_status){
			printf("Got payload capture status: image_status: %.2f, video_status: %.2f \n", param[0], param[1]);

			// if image status is idle, do capture
			if(param[0] == 0 ){
				my_capture = check_camera_mode;
				printf("   ---> Payload is idle, Check camera mode \n");
			}else{
				printf("   ---> Payload is busy \n");
				my_capture = idle;
			}
		}else if(my_capture == wait_capture_done){
			if(param[0] == 0 ){
				my_capture = check_storage;
				printf("   ---> Payload is completed capture image, Do next sequence %d\n\n", --image_to_capture);
				if(image_to_capture == 0) {
					// close payload interface
					try {
						my_payload->sdkQuit();
					}
					catch (int error){}

					exit(0);
				}
			}else{
				printf("   ---> Payload is busy \n");
			}
		}
		break;
	}
	case PAYLOAD_CAM_STORAGE_INFO:{
		// param[0]: total_capacity
		// param[1]: used_capacity
		// param[2]: available_capacity
		// param[3]: status

		if(my_capture == check_storage){
			printf("Got payload storage info: total: %.2f MB, used: %.2f MB, available: %.2f MB \n", 
				param[0], param[1], param[2]);

			// if payload have enough space, check capture status
			if(param[2] >= 10.0){
				my_capture = check_capture_status;
				printf("   ---> Storage ready, check capture status \n");
			}else{
				printf("   ---> Payload's storage is not ready \n");
				my_capture = idle;
			}
		}
		break;
	}
	case PAYLOAD_CAM_SETTINGS:{
		// param[0]: mode_id
		// param[1]: zoomLevel
		// param[2]: focusLevel
		if(my_capture == check_camera_mode){
			printf("Got camera mode: %.2f (raw value)\n", param[0]);
			printf("Mode details: Mode ID: %.2f, Zoom Level: %.2f, Focus Level: %.2f\n", 
			       param[0], param[1], param[2]);
			
			// Map mode value to name for readability
			if(param[0] == CAMERA_MODE_IMAGE)
				printf("Current mode is: IMAGE MODE\n");
			else if(param[0] == CAMERA_MODE_VIDEO)
				printf("Current mode is: VIDEO MODE\n");
			else
				printf("Current mode is: UNKNOWN MODE (%.2f)\n", param[0]);

			if(param[0] == CAMERA_MODE_IMAGE){
				my_capture = do_capture;
				printf("   ---> Payload in Image mode, do capture image \n");
			}else{
				my_capture = change_camera_mode;
				printf("   ---> Payload in Video mode, change camera mode \n");
			}
		}
		break;
	}
	default: 
		printf("Unhandled event type: %d with values: %.2f, %.2f, %.2f, %.2f\n",
		      event, param[0], param[1], param[2], param[3]);
		break;
	}
}