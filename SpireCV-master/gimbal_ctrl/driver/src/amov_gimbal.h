/*
 * @Description:    External interface of amov gimbals
 * @Author: L LC @amov
 * @Date: 2022-10-27 18:34:26
 * @LastEditors: L LC @amov
 * @LastEditTime: 2023-08-16 22:21:28
 * @FilePath: /gimbal-sdk-multi-platform/src/amov_gimbal.h
 */

#ifndef AMOV_GIMBAL_H
#define AMOV_GIMBAL_H

#include <stdint.h>
#include <stdbool.h>
#include <iostream>

#include "amov_gimbal_struct.h"

#define MAX_QUEUE_SIZE 100
namespace amovGimbal
{
#define IN
#define OUT
#define SET

    static inline void idleCallback(double &frameAngleRoll, double &frameAnglePitch, double &frameAngleYaw,
                                    double &imuAngleRoll, double &imuAnglePitch, double &imuAngleYaw,
                                    double &fovX, double &fovY)
    {
    }
    static inline void idleMsgCallback(void *)
    {
    }


    // Control data input and output
    class IOStreamBase
    {
    public:
        IOStreamBase() {}
        virtual ~IOStreamBase() {}

        virtual bool open() = 0;
        virtual bool close() = 0;
        virtual bool isOpen() = 0;
        virtual bool isBusy() = 0;
        // These two functions need to be thread-safe
        virtual bool inPutByte(IN uint8_t *byte) = 0;
        virtual uint32_t outPutBytes(IN uint8_t *byte, uint32_t lenght) = 0;
    };

    // Device interface
    class IamovGimbalBase
    {
    public:
        IamovGimbalBase() {}
        virtual ~IamovGimbalBase() {}
        // functions
        virtual void nodeSet(SET uint32_t _self, SET uint32_t _remote);
        virtual uint32_t setGimabalPos(const AMOV_GIMBAL_POS_T &pos);
        virtual uint32_t setGimabalSpeed(const AMOV_GIMBAL_POS_T &speed);
        virtual uint32_t setGimabalFollowSpeed(const AMOV_GIMBAL_POS_T &followSpeed);
        virtual uint32_t setGimabalHome(void);
        virtual uint32_t setGimbalZoom(AMOV_GIMBAL_ZOOM_T zoom, float targetRate = 0);
        virtual uint32_t setGimbalFocus(AMOV_GIMBAL_ZOOM_T zoom, float targetRate = 0);
        virtual uint32_t setGimbalROI(const AMOV_GIMBAL_ROI_T area);
        virtual uint32_t takePic(void);
        virtual uint32_t setVideo(const AMOV_GIMBAL_VIDEO_T newState);
        virtual uint32_t attitudeCorrection(const AMOV_GIMBAL_QUATERNION_T &quaterion, const AMOV_GIMBAL_VELOCITY_T &speed, const AMOV_GIMBAL_VELOCITY_T &acc, void *extenData);
        virtual uint32_t attitudeCorrection(const AMOV_GIMBAL_POS_T &pos, const AMOV_GIMBAL_VELOCITY_T &seppd, const AMOV_GIMBAL_VELOCITY_T &acc, void *extenData);
        virtual uint32_t extensionFuntions(void *cmd);
    };

    class gimbal
    {
    private:
        std::string typeName;
        IOStreamBase *IO;

    public:
        // Instantiated device handle
        IamovGimbalBase *dev;

        // Protocol stack function items
        void startStack(void);
        void parserAuto(pStateInvoke callback = idleCallback);
        void setParserCallback(pStateInvoke callback);
        void setMsgCallback(pMsgInvoke callback);

        AMOV_GIMBAL_STATE_T getGimabalState(void);

        std::string name()
        {
            return typeName;
        }
        gimbal(const std::string &type, IOStreamBase *_IO,
               uint32_t _self = 0x02, uint32_t _remote = 0X80);
        ~gimbal();
    };
}
#endif