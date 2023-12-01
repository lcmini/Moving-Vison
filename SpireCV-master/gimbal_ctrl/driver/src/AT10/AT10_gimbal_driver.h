/*
 * @Description:    Q10f吊舱的驱动文件
 * @Author: L LC @amov
 * @Date: 2022-10-28 12:24:21
 * @LastEditors: L LC @amov
 * @LastEditTime: 2023-08-25 19:28:55
 * @FilePath: /gimbal-sdk-multi-platform/src/AT10/AT10_gimbal_driver.h
 */
#include "../amov_gimbal.h"
#include "../amov_gimbal_private.h"
#include "AT10_gimbal_struct.h"
#include <mutex>
#include <malloc.h>
#include <iostream>

#ifndef __AT10_DRIVER_H
#define __AT10_DRIVER_H

class AT10GimbalDriver : protected amovGimbal::amovGimbalBase
{
private:
    AT10::GIMBAL_SERIAL_STATE_T parserState;
    AT10::GIMBAL_EXTEND_FRAME_T extendRx;

    AT10::GIMBAL_STD_FRAME_T stdRx;
    fifoRing *stdRxQueue;
    fifoRing *stdTxQueue;

    // void send(void);

    void stackStart(void);
    void sendHeart(void);
    void sendStd(void);

    void parserStart(amovGimbal::pStateInvoke callback);
    void parserLoop(void);
    void getExtRxPack(void);
    void getStdRxPack(void);

    // bool getRxPack(OUT void *pack);

    bool parser(IN uint8_t byte);
    void convert(void *buf);
    uint32_t pack(IN uint32_t cmd, uint8_t *pPayload, uint8_t payloadSize);
    uint32_t calPackLen(void *pack);

public:
    // funtions
    uint32_t setGimabalPos(const amovGimbal::AMOV_GIMBAL_POS_T &pos);
    uint32_t setGimabalSpeed(const amovGimbal::AMOV_GIMBAL_POS_T &speed);
    uint32_t setGimabalFollowSpeed(const amovGimbal::AMOV_GIMBAL_POS_T &followSpeed);
    uint32_t setGimabalHome(void);

    uint32_t setGimbalZoom(amovGimbal::AMOV_GIMBAL_ZOOM_T zoom, float targetRate = 0);
    uint32_t setGimbalFocus(amovGimbal::AMOV_GIMBAL_ZOOM_T zoom, float targetRate = 0);

    uint32_t takePic(void);
    uint32_t setVideo(const amovGimbal::AMOV_GIMBAL_VIDEO_T newState);

    uint32_t extensionFuntions(void* cmd);

    // builds
    static amovGimbal::amovGimbalBase *creat(amovGimbal::IOStreamBase *_IO)
    {
        return new AT10GimbalDriver(_IO);
    }

    AT10GimbalDriver(amovGimbal::IOStreamBase *_IO);
    ~AT10GimbalDriver()
    {
        if (stdRxQueue != nullptr)
        {
            delete stdRxQueue;
        }
        if (stdTxQueue != nullptr)
        {
            delete stdTxQueue;
        }
    }
};

#endif
