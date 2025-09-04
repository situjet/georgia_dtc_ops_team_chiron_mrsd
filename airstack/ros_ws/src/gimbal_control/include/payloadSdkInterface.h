#ifndef PAYLOADSDK_INTERFACE_H_
#define PAYLOADSDK_INTERFACE_H_

#include <iostream>
#include "payloadsdk.h"
#include "gimbal_protocol_v2.h"
#include <functional>

enum payload_status_event_t{
	PAYLOAD_CAM_CAPTURE_STATUS = 0,
	PAYLOAD_CAM_STORAGE_INFO,
	PAYLOAD_CAM_SETTINGS,
	PAYLOAD_CAM_PARAM_VALUE,

	PAYLOAD_GB_ATTITUDE,
	PAYLOAD_GB_ACK,

	PAYLOAD_CAM_INFO,
	PAYLOAD_CAM_STREAMINFO,

	PAYLOAD_PARAMS,
};

enum payload_param_t{
	PARAM_EO_ZOOM_LEVEL = 0,
	PARAM_IR_ZOOM_LEVEL,
	PARAM_LRF_RANGE,

	PARAM_TRACK_POS_X,
	PARAM_TRACK_POS_Y,
	PARAM_TRACK_STATUS,

	PARAM_LRF_OFSET_X,
	PARAM_LRF_OFSET_Y,

	PARAM_TARGET_COOR_LON,
	PARAM_TARGET_COOR_LAT,
	PARAM_TARGET_COOR_ALT,

	PARAM_COUNT
};

enum cable_state_e {
	 CABLE_STATE_OFF = 0x00,
	 CABLE_STATE_MIO = 0x02,
	 CABLE_STATE_VIO = 0x03
};

class PayloadSdkInterface
{
public:
	typedef std::function<void(int event, double* param)> payload_status_callback_t;
	typedef std::function<void(int event, char* param_char, double* param_double)> payload_param_callback_t;
	typedef std::function<void(int event, char* param_char, double* param_double)> payload_streamInfo_callback_t;

	PayloadSdkInterface();
	PayloadSdkInterface(T_ConnInfo data);
	~PayloadSdkInterface();

	void regPayloadStatusChanged(payload_status_callback_t func);
	void regPayloadParamChanged(payload_param_callback_t func);
	void regPayloadStreamChanged(payload_streamInfo_callback_t func);
	bool sdkInitConnection();
	void sdkQuit();
	bool all_threads_init();
	void checkPayloadConnection();
	uint8_t getNewMewssage(mavlink_message_t& new_msg);
	void setPayloadCameraParam(char param_id[], uint32_t param_value, uint8_t param_type);
	void getPayloadCameraSettingList();
	void getPayloadStorage();
	void getPayloadCaptureStatus();
	void getPayloadCameraMode();
	void getPayloadCameraInformation();
	void getPayloadCameraStreamingInformation();
	void setPayloadCameraMode(CAMERA_MODE mode);
	void setPayloadCameraCaptureImage(int = 0);
	void setPayloadCameraStopImage();
	void setPayloadCameraRecordVideoStart();
	void setPayloadCameraRecordVideoStop();
	void requestParamValue(uint8_t pIndex);
	void setParamRate(uint8_t pIndex, uint16_t time_ms);
	void initGimbal(Serial_Port* port);
	void setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, Gimbal_Protocol::input_mode_t mode);
	void setGimbalMode(Gimbal_Protocol::control_mode_t mode);
	void setGimbalResetMode(Gimbal_Protocol::gimbal_reset_mode_t reset_mode);
	void setGimbalPowerOn();
	void setGimbalPowerOff();
	void setGimbalICSwitch(cable_state_e state);
	void setCameraZoom(float zoomType,float zoomValue);
	void setCameraFocus(float focusType, float focusValue=0);
	void setPayloadObjectTrackingParams(float cmd, float pos_x, float pos_y);
	void payload_recv_handle();
	void payload_request_handle();
private:
	pthread_t thrd_recv;
	pthread_t thrd_request_params;

	uint8_t payload_ctrl_type = CONTROL_METHOD;
	Generic_Port *port;
	Generic_Port *port_quit = nullptr;

	uint8_t SYS_ID = 1;
	uint8_t COMP_ID = MAV_COMP_ID_ONBOARD_COMPUTER;

	bool time_to_exit = false;

	uint16_t paramRate[PARAM_COUNT];

	uint8_t SYS_ID_USER2 = 1;

public:
	Gimbal_Protocol_V2* myGimbal = nullptr;
	Serial_Port* myGimbalPort = nullptr;
	mavlink_system_t _system_id;
	mavlink_system_t _gimbal_id;

	void initGimbal(Serial_Port* port);
	void setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, Gimbal_Protocol::input_mode_t mode);
	void setGimbalMode(Gimbal_Protocol::control_mode_t mode);
	void setGimbalResetMode(Gimbal_Protocol::gimbal_reset_mode_t reset_mode);
	void setGimbalPowerOn();
	void setGimbalPowerOff();
	void setGimbalICSwitch(cable_state_e state);
	void setCameraZoom(float zoomType,float zoomValue);
	void setCameraFocus(float focusType, float focusValue=0);
	void setPayloadObjectTrackingParams(float cmd, float pos_x, float pos_y);
	void payload_recv_handle();
	void payload_request_handle();
private:
	void _handle_msg_param_ext_value(mavlink_message_t* msg);
	void _handle_msg_command_ack(mavlink_message_t* msg);
	void _handle_msg_storage_information(mavlink_message_t* msg);
	void _handle_msg_camera_capture_status(mavlink_message_t* msg);
	void _handle_msg_camera_settings(mavlink_message_t* msg);
	void _handle_msg_mount_orientation(mavlink_message_t* msg);
	void _handle_msg_param_value(mavlink_message_t* msg);
	void _handle_msg_camera_stream_information(mavlink_message_t* msg);
	void _handle_msg_camera_information(mavlink_message_t* msg);
};
#endif
