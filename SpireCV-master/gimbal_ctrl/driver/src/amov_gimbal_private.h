/*
 * @Description    :
 * @Author         : Aiyangsky
 * @Date           : 2023-05-13 10:39:20
 * @LastEditors: L LC @amov
 * @LastEditTime: 2023-08-16 22:30:53
 * @FilePath: /gimbal-sdk-multi-platform/src/amov_gimbal_private.h
 */
#ifndef __AMOV_GIMABL_PRIVATE_H
#define __AMOV_GIMABL_PRIVATE_H

#include <stdint.h>
#include <stdbool.h>
#include <iostream>

#include <thread>
#include <unistd.h>
#include <mutex>

#include "amov_gimbal.h"

#include "Ring_Fifo.h"
#include "amov_tool.h"
namespace amovGimbal
{
    class PamovGimbalBase
    {
    public:
        AMOV_GIMBAL_STATE_T state;
        std::mutex mState;
        IOStreamBase *IO;
        pStateInvoke updateGimbalStateCallback;
        pMsgInvoke msgCustomCallback = idleMsgCallback;

        fifoRing *rxQueue;
        fifoRing *txQueue;

        PamovGimbalBase(SET IOStreamBase *_IO)
        {
            IO = _IO;
        }
        virtual ~PamovGimbalBase()
        {
            if (txQueue != nullptr)
            {
                delete txQueue;
            }
            if (rxQueue != nullptr)
            {
                delete rxQueue;
            }
        }
    };

    class amovGimbalBase : public IamovGimbalBase, public PamovGimbalBase
    {
    public:
        virtual uint32_t pack(IN uint32_t cmd, uint8_t *pPayload, uint8_t payloadSize) = 0;
        virtual bool parser(IN uint8_t byte) = 0;
        virtual void convert(void *buf) = 0;
        virtual uint32_t calPackLen(void *pack) = 0;

        virtual void send(void);
        virtual bool getRxPack(OUT void *pack);

        virtual void parserLoop(void);
        virtual void sendLoop(void);
        virtual void mainLoop(void);

        virtual void stackStart(void);
        virtual void parserStart(amovGimbal::pStateInvoke callback);

    public:
        amovGimbalBase(IOStreamBase *_IO);
        virtual ~amovGimbalBase();
    };
}

#endif