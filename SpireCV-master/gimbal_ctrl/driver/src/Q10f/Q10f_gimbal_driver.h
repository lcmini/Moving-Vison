/*
 * @Description:    Q10f吊舱的驱动文件
 * @Author: L LC @amov
 * @Date: 2022-10-28 12:24:21
 * @LastEditors: L LC @amov
 * @LastEditTime: 2023-03-28 17:01:00
 * @FilePath: /gimbal-sdk-multi-platform/src/Q10f/Q10f_gimbal_driver.h
 */
#include "../amov_gimbal.h"
#include "../amov_gimbal_private.h"
#include "Q10f_gimbal_struct.h"
#include <mutex>
#include <malloc.h>
#include <iostream>

#ifndef __Q10F_DRIVER_H
#define __Q10F_DRIVER_H

class Q10fGimbalDriver : protected amovGimbal::amovGimbalBase
{
private:
    Q10f::GIMBAL_SERIAL_STATE_T parserState;
    Q10f::GIMBAL_FRAME_T rx;

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

    // builds
    static amovGimbal::amovGimbalBase *creat(amovGimbal::IOStreamBase *_IO)
    {
        return new Q10fGimbalDriver(_IO);
    }

    Q10fGimbalDriver(amovGimbal::IOStreamBase *_IO);
};

#endif
