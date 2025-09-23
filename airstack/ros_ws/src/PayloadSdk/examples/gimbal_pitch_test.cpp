#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>
#include <ctime>
#include <chrono>
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

void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);
std::string getCurrentTimeString();

int main(int argc, char *argv[]){
    printf("[%s] Starting gimbal pitch test - will pitch down 30 degrees then zero...\n", getCurrentTimeString().c_str());
    signal(SIGINT,quit_handler);

    // create payloadsdk object
    my_payload = new PayloadSdkInterface(s_conn);

    // init payload
    printf("[%s] Initializing payload connection...\n", getCurrentTimeString().c_str());
    my_payload->sdkInitConnection();
    printf("[%s] Waiting for payload signal!\n", getCurrentTimeString().c_str());

    // register callback function to monitor gimbal attitude
    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

    // check connection
    my_payload->checkPayloadConnection();
    usleep(500000);

    printf("[%s] Setting gimbal mode to FOLLOW...\n", getCurrentTimeString().c_str());
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
    usleep(1000000);

    // Step 1: Move gimbal pitch down by -30 degrees (negative is down)
    printf("[%s] === STEP 1: Pitching gimbal DOWN by 30 degrees ===\n", getCurrentTimeString().c_str());
    my_payload->setGimbalSpeed(-30, 0, 0, Gimbal_Protocol::INPUT_ANGLE);
    
    // Wait for movement to complete (5 seconds)
    printf("[%s] Waiting 5 seconds for gimbal to reach target position...\n", getCurrentTimeString().c_str());
    usleep(5000000);

    // Step 2: Return gimbal to zero position using reset mode
    printf("[%s] === STEP 2: Zeroing gimbal (returning to home position) ===\n", getCurrentTimeString().c_str());
    my_payload->setGimbalResetMode(Gimbal_Protocol::GIMBAL_RESET_MODE_PITCH_AND_YAW);
    
    // Wait for reset to complete
    printf("[%s] Waiting 3 seconds for gimbal to return to zero position...\n", getCurrentTimeString().c_str());
    usleep(3000000);

    // Alternative method: Manually set to zero degrees
    printf("[%s] === ALTERNATIVE: Manually setting gimbal to zero degrees ===\n", getCurrentTimeString().c_str());
    my_payload->setGimbalSpeed(0, 0, 0, Gimbal_Protocol::INPUT_ANGLE);
    usleep(2000000);

    printf("[%s] === Test completed! ===\n", getCurrentTimeString().c_str());

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    return 0;
}

void quit_handler( int sig ){
    printf("\n");
    printf("[%s] TERMINATING AT USER REQUEST\n", getCurrentTimeString().c_str());
    printf("\n");

    time_to_exit = true;

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    // end program here
    exit(0);
}

void onPayloadStatusChanged(int event, double* param){
    switch(event){
        case PAYLOAD_GB_ATTITUDE:{
            // param[0]: pitch
            // param[1]: roll
            // param[2]: yaw
            printf("[%s] GIMBAL ATTITUDE - Pitch: %.2f° - Roll: %.2f° - Yaw: %.2f°\n", 
                   getCurrentTimeString().c_str(), param[0], param[1], param[2]);
            break;
        }
        default: break;
    }
}

std::string getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", std::localtime(&time_t));
    
    char result[150];
    sprintf(result, "%s.%03d", buffer, (int)ms.count());
    return std::string(result);
}