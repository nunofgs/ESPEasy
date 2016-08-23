//#######################################################################################################
//#################################### Plugin 036: MPU6050 ##############################################
//#######################################################################################################

#include "Wire.h"
#include "MPU6050.h"

MPU6050 *mpu;

int16_t motion = 0;

#define PLUGIN_036
#define PLUGIN_ID_036         36
#define PLUGIN_NAME_036       "Motion Detection - MPU6050"
#define PLUGIN_VALUENAME1_036 "Motion"

#define MPU6050_ADDRESS 0x68 // I2C address

boolean Plugin_036(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_036;
        Device[deviceCount].Type = DEVICE_TYPE_I2C;
        Device[deviceCount].VType = SENSOR_TYPE_SINGLE;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = false;
        Device[deviceCount].ValueCount = 1;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].GlobalSyncOption = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_036);
        break;
      }

      case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_036));
        break;
      }

      case PLUGIN_WEBFORM_LOAD:
      {
        char tmpString[128];
        sprintf_P(tmpString, PSTR("<TR><TD>Motion threshold:<TD><input type='text' name='plugin_036_motion_threshold' value='%d'>"), ExtraTaskSettings.TaskDevicePluginConfigLong[0]);
        string += tmpString;

        success = true;
        break;
      }

    case PLUGIN_WEBFORM_SAVE:
      {
        int16_t threshold = WebServer.arg("plugin_036_motion_threshold").toInt();
        ExtraTaskSettings.TaskDevicePluginConfigLong[0] = threshold;

        mpu->setMotionDetectionThreshold(threshold);
        mpu->setZeroMotionDetectionThreshold(threshold);

        success = true;
        break;
      }

    case PLUGIN_INIT:
      {
        addLog(LOG_LEVEL_INFO, "MPU-6050 : Init");

        // Load saved threshold settings.
        LoadTaskSettings(event->TaskIndex);

        // Initialize MPU sensor.
        mpu = new MPU6050(MPU6050_ADDRESS);
        mpu->initialize();

        // Enable Zero Motion Detection interrupt.
        mpu->setIntZeroMotionEnabled(false);

        // Set accelerometer power-on delay.
        mpu->setAccelerometerPowerOnDelay(3);

        // Set high-pass filter configuration in mode 1 (5Hz) for improved motion detection.
        mpu->setDHPFMode(1);

        // Set motion detection event acceleration threshold. Motion is detected when the
        // absolute value of any of the accelerometer measurements exceeds this Motion
        // detection threshold. This condition increments the Motion detection duration
        // counter (Register 32). The Motion detection interrupt is triggered when the
        // Motion Detection counter reaches the time count specified in MOT_DUR.
        mpu->setMotionDetectionThreshold(ExtraTaskSettings.TaskDevicePluginConfigLong[0] ?: 2);

        // Set zero motion detection event acceleration threshold. When a zero motion event
        // is detected, a Zero Motion Status will be indicated in the MOT_DETECT_STATUS
        // register (Register 97). When a motion-to-zero-motion condition is detected,
        // the status bit is set to 1. When a zero-motion-to-motion condition is detected,
        // the status bit is set to 0.
        mpu->setZeroMotionDetectionThreshold(ExtraTaskSettings.TaskDevicePluginConfigLong[0] ?: 2);

        // Set motion detection event duration threshold. The Motion detection duration
        // counter increments when the absolute value of any of the accelerometer 
        // measurements exceeds the Motion detection threshold (Register 31). The Motion
        // detection interrupt is triggered when the Motion detection counter reaches
        // the time count specified in this register.
        mpu->setMotionDetectionDuration(40);

        // Set zero motion detection event duration threshold. The Zero Motion duration counter
        // increments while the absolute value of the accelerometer measurements are each
        // less than the detection threshold (Register 33). The Zero Motion interrupt is
        // triggered when the Zero Motion duration counter reaches the time count specified
        // in this register.
        mpu->setZeroMotionDetectionDuration(1);

        success = true;
        break;
      }

    case PLUGIN_ONCE_A_SECOND:
      {
        // If any axis has motion, then increment motion.
        if (mpu->getMotionStatus() > 0) {
          motion++;
        }

        success = true;
        break;
      }

    case PLUGIN_READ:
      {
        // Convert motion to a percentage that represents how much motion there was
        // in the last X seconds, where X is the configured polling delay.
        UserVar[event->BaseVarIndex] = ((float) motion / Settings.TaskDeviceTimer[event->TaskIndex]) * 100;
        
        // Reset motion.
        motion = 0;

        String log = F("MPU6050 : Reporting motion: ");
        log += UserVar[event->BaseVarIndex];
        addLog(LOG_LEVEL_DEBUG, log);

        sendData(event);

        success = true;
        break;
      }
  }

  return success;
}
