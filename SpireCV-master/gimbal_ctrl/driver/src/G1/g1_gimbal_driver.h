/*
 * @Description:    G1吊舱的驱动文件
 * @Author: L LC @amov
 * @Date: 2022-10-28 12:24:21
 * @LastEditors: L LC @amov
 * @LastEditTime: 2023-09-07 02:31:19
 * @FilePath: /gimbal-sdk-multi-platform/src/G1/g1_gimbal_driver.h
 */
#include "../amov_gimbal.h"
#include "../amov_gimbal_private.h"
#include "g1_gimbal_struct.h"
#include <mutex>
#include <malloc.h>
#include <iostream>

#ifndef __G1_DRIVER_H
#define __G1_DRIVER_H

class g1GimbalDriver : protected amovGimbal::amovGimbalBase
{
private:
    G1::GIMBAL_CMD_PARSER_STATE_T parserState;
    G1::GIMBAL_FRAME_T rx;

    uint32_t pack(IN uint32_t cmd, uint8_t *pPayload, uint8_t payloadSize);
    bool parser(IN uint8_t byte);
    void convert(void *buf);
    uint32_t calPackLen(void *pack);

public:
    // funtions
    uint32_t setGimabalPos(const amovGimbal::AMOV_GIMBAL_POS_T &pos);
    uint32_t setGimabalSpeed(const amovGimbal::AMOV_GIMBAL_POS_T &speed);
    uint32_t setGimabalFollowSpeed(const amovGimbal::AMOV_GIMBAL_POS_T &followSpeed);
    uint32_t setGimabalHome(void);

    uint32_t takePic(void);
    uint32_t setVideo(const amovGimbal::AMOV_GIMBAL_VIDEO_T newState);

    uint32_t attitudeCorrection(const amovGimbal::AMOV_GIMBAL_QUATERNION_T &quaterion,
                                const amovGimbal::AMOV_GIMBAL_VELOCITY_T &speed,
                                const amovGimbal::AMOV_GIMBAL_VELOCITY_T &acc,
                                void *extenData);

    uint32_t attitudeCorrection(const amovGimbal::AMOV_GIMBAL_POS_T &pos,
                                const amovGimbal::AMOV_GIMBAL_VELOCITY_T &seppd,
                                const amovGimbal::AMOV_GIMBAL_VELOCITY_T &acc,
                                void *extenData);
    uint32_t extensionFuntions(void *cmd);

    // builds
    static amovGimbal::amovGimbalBase *creat(amovGimbal::IOStreamBase *_IO)
    {
        return new g1GimbalDriver(_IO);
    }

    g1GimbalDriver(amovGimbal::IOStreamBase *_IO);
};

#endif
