// #include "stdio.h"
// #include <pthread.h>
// #include <cstdlib>
// #include <string>
// using namespace std;

// #include"payloadSdkInterface.h"

// #if (CONTROL_METHOD == CONTROL_UART)
// T_ConnInfo s_conn = {
//     CONTROL_UART,
//     payload_uart_port,
//     payload_uart_baud
// };
// #else
// T_ConnInfo s_conn = {
//     CONTROL_UDP,
//     udp_ip_target,
//     udp_port_target
// };
// #endif

// PayloadSdkInterface* my_payload = nullptr;
// bool time_to_exit = false;

// pthread_t thrd_recv;
// pthread_t thrd_gstreamer;

// bool gstreamer_start();
// void gstreamer_terminate();
// void *start_loop_thread(void *threadid);


// bool all_threads_init();
// void quit_handler(int sig);


// int main(int argc, char *argv[]){
// 	printf("Starting Set gimbal mode example...\n");
// 	signal(SIGINT,quit_handler);

// 	// create payloadsdk object
// 	my_payload = new PayloadSdkInterface(s_conn);

// 	// init payload
// 	my_payload->sdkInitConnection();
// 	printf("Waiting for payload signal! \n");

// 	my_payload->checkPayloadConnection();

// 	printf("Gimbal set mode FOLLOW \n");
// 	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
// 	usleep(1000000);


// 	printf("Move gimbal to home position, delay in 5secs \n");
// 	my_payload->setGimbalSpeed(-45, 0 , 0, Gimbal_Protocol::INPUT_ANGLE);
// 	usleep(200000);
// 	my_payload->setGimbalSpeed(0, 0 , 0, Gimbal_Protocol::INPUT_ANGLE);
// 	usleep(500000);
// 	// close payload interface
// 	try {
// 		my_payload->sdkQuit();
// 	}
// 	catch (int error){}

// 	return 0;
// }

// void quit_handler( int sig ){
//     printf("\n");
//     printf("TERMINATING AT USER REQUEST \n");
//     printf("\n");

//     time_to_exit = true;

//     // close payload interface
//     try {
//         my_payload->sdkQuit();
//     }
//     catch (int error){}

//     // end program here
//     exit(0);
// }

#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>
#include <math.h>
using namespace std;

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
bool time_to_exit = false;
float current_pitch = 0, current_roll = 0, current_yaw = 0;
bool got_attitude = false;

pthread_t thrd_recv;
pthread_t thrd_gstreamer;

bool gstreamer_start();
void gstreamer_terminate();
void *start_loop_thread(void *threadid);

bool all_threads_init();
void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);

// interpolation function, implement smooth movement
void moveToCenter(float pitch, float roll, float yaw) {
    // determine the sign of each axis (positive or negative)
    int pitch_sign = (pitch > 0) ? 1 : -1;
    int roll_sign = (roll > 0) ? 1 : -1;
    int yaw_sign = (yaw > 0) ? 1 : -1;
    
    printf("Current angles - Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", pitch, roll, yaw);
    printf("Moving gimbal to center with proper direction...\n");
    
    // use the opposite sign to move back to center
    float target_pitch = -pitch_sign * fabs(pitch);
    float target_roll = -roll_sign * fabs(roll);
    float target_yaw = -yaw_sign * fabs(yaw);
    
    // set the movement speed (adjustable as needed)
    my_payload->setGimbalSpeed(target_pitch, target_roll, target_yaw, Gimbal_Protocol::INPUT_ANGLE);
    usleep(1000000); // wait for enough time for the gimbal to move
    
    // stop the movement
    my_payload->setGimbalSpeed(0, 0, 0, Gimbal_Protocol::INPUT_ANGLE);
}

int main(int argc, char *argv[]) {
    printf("Starting Set gimbal mode example...\n");
    signal(SIGINT, quit_handler);

    // create payloadsdk object
    my_payload = new PayloadSdkInterface(s_conn);

    // initialize payload
    my_payload->sdkInitConnection();
    printf("Waiting for payload signal! \n");

    // register status callback
    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);
    
    my_payload->checkPayloadConnection();

    printf("Gimbal set mode FOLLOW \n");
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
    usleep(1000000);

    // wait for current attitude
    int wait_count = 0;
    while (!got_attitude && wait_count < 10) {
        usleep(500000); // check every 500ms
        wait_count++;
    }

    if (got_attitude) {
        printf("Got current attitude, moving to center\n");
        moveToCenter(current_pitch, current_roll, current_yaw);
    } else {
        printf("Could not get current attitude, using default approach\n");
        
        printf("Move gimbal to home position\n");
        my_payload->setGimbalSpeed(-45, 0, 0, Gimbal_Protocol::INPUT_ANGLE);
        usleep(200000);
        my_payload->setGimbalSpeed(0, 0, 0, Gimbal_Protocol::INPUT_ANGLE);
    }
    
    usleep(500000);
    
    // 关闭payload接口
    try {
        my_payload->sdkQuit();
    }
    catch (int error) {}

    return 0;
}

void onPayloadStatusChanged(int event, double* param) {
    if (event == PAYLOAD_GB_ATTITUDE) {
        // param[0]: pitch
        // param[1]: roll
        // param[2]: yaw
        current_pitch = param[0];
        current_roll = param[1];
        current_yaw = param[2];
        got_attitude = true;
    }
}

void quit_handler(int sig) {
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    time_to_exit = true;

    // 关闭payload接口
    try {
        my_payload->sdkQuit();
    }
    catch (int error) {}

    // 结束程序
    exit(0);
}