/*
 * @Description:
 * @Author: L LC @amov
 * @Date: 2022-10-28 11:54:11
 * @LastEditors: L LC @amov
 * @LastEditTime: 2023-08-17 11:57:11
 * @FilePath: /gimbal-sdk-multi-platform/src/amov_gimbal.cpp
 */

// #include "amov_gimbal.h"
#include "amov_gimbal_private.h"
#include "g1_gimbal_driver.h"
#include "Q10f_gimbal_driver.h"
#include "AT10_gimbal_driver.h"

#include <iostream>
#include <thread>
#include <map>
#include <iterator>

#define MAX_PACK_SIZE 280
typedef enum
{
    AMOV_GIMBAL_TYPE_NULL,
    AMOV_GIMBAL_TYPE_G1 = 1,
    AMOV_GIMBAL_TYPE_G2,
    AMOV_GIMBAL_TYPE_Q10,
    AMOV_GIMBAL_TYPE_AT10,
} AMOV_GIMBAL_TYPE_T;

namespace amovGimbal
{
    typedef amovGimbal::amovGimbalBase *(*createCallback)(amovGimbal::IOStreamBase *_IO);
    typedef std::map<std::string, createCallback> callbackMap;
    std::map<std::string, AMOV_GIMBAL_TYPE_T> amovGimbalTypeList =
        {
            {"G1", AMOV_GIMBAL_TYPE_G1},
            {"Q10f", AMOV_GIMBAL_TYPE_Q10},
            {"AT10", AMOV_GIMBAL_TYPE_AT10}};

    callbackMap amovGimbals =
        {
            {"G1", g1GimbalDriver::creat},
            {"Q10f", Q10fGimbalDriver::creat},
            {"AT10", AT10GimbalDriver::creat}};
}

/* The amovGimbalCreator class is a factory class that creates an instance of the amovGimbal class */
// Factory used to create the gimbal instance
class amovGimbalCreator
{
public:
    static amovGimbal::amovGimbalBase *createAmovGimbal(const std::string &type, amovGimbal::IOStreamBase *_IO)
    {
        amovGimbal::callbackMap::iterator temp = amovGimbal::amovGimbals.find(type);

        if (temp != amovGimbal::amovGimbals.end())
        {
            return (temp->second)(_IO);
        }
        std::cout << type << " is Unsupported device type!" << std::endl;
        return NULL;
    }

private:
    amovGimbalCreator()
    {
    }
    static amovGimbalCreator *pInstance;
    static amovGimbalCreator *getInstance()
    {
        if (pInstance == NULL)
        {
            pInstance = new amovGimbalCreator();
        }
        return pInstance;
    }

    ~amovGimbalCreator();
};

/**
 * This is a constructor for the amovGimbalBase class that initializes its parent classes with an
 * IOStreamBase object.
 *
 * @param _IO _IO is a pointer to an object of type amovGimbal::IOStreamBase, which is the base class
 * for input/output streams used by the amovGimbal class. This parameter is passed to the constructor
 * of amovGimbalBase, which is a derived class of I
 */
amovGimbal::amovGimbalBase::amovGimbalBase(amovGimbal::IOStreamBase *_IO) : amovGimbal::IamovGimbalBase(), amovGimbal::PamovGimbalBase(_IO)
{
}

/**
 * The function is a destructor that sleeps for 50 milliseconds and closes an IO object.
 */
amovGimbal::amovGimbalBase::~amovGimbalBase()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    IO->close();
}

/**
 * This function retrieves a packet from a ring buffer queue and returns a boolean value indicating
 * whether the operation was successful or not.
 *
 * @param void void is a keyword in C++ that represents the absence of a type. In this function, it is
 * used to indicate that the function does not return any value.
 *
 * @return a boolean value, which indicates whether or not a data packet was successfully retrieved
 * from a ring buffer queue.
 */
bool amovGimbal::amovGimbalBase::getRxPack(OUT void *pack)
{
    bool state = false;
    state = rxQueue->outCell(pack);
    return state;
}

/**
 * This function sends data from a buffer to an output device if it is not busy and open.
 */
void amovGimbal::amovGimbalBase::send(void)
{
    uint8_t tempBuffer[MAX_PACK_SIZE];

    if (!IO->isBusy() && IO->isOpen())
    {
        bool state = false;

        state = txQueue->outCell(&tempBuffer);

        if (state)
        {
            IO->outPutBytes((uint8_t *)&tempBuffer, calPackLen(tempBuffer));
        }
    }
}

/**
 * "If the input byte is available, then parse it."
 *
 * The function is a loop that runs forever. It calls the IO->inPutByte() function to get a byte from
 * the serial port. If the byte is available, then it calls the parser() function to parse the byte
 */
void amovGimbal::amovGimbalBase::parserLoop(void)
{
    uint8_t temp;

    while (1)
    {
        if (IO->inPutByte(&temp))
        {
            parser(temp);
        }
    }
}

void amovGimbal::amovGimbalBase::sendLoop(void)
{
    while (1)
    {
        send();
    }
}

void amovGimbal::amovGimbalBase::mainLoop(void)
{
    uint8_t tempBuffer[MAX_PACK_SIZE];

    while (1)
    {
        if (getRxPack(tempBuffer))
        {
            msgCustomCallback(tempBuffer);
            convert(tempBuffer);
        }
    }
}

void amovGimbal::amovGimbalBase::stackStart(void)
{
    if (!this->IO->isOpen())
    {
        this->IO->open();
    }
    std::thread parserLoop(&amovGimbalBase::parserLoop, this);
    std::thread sendLoop(&amovGimbalBase::sendLoop, this);
    parserLoop.detach();
    sendLoop.detach();
}
/**
 * It starts two threads, one for reading data from the serial port and one for sending data to the
 * serial port
 */
void amovGimbal::gimbal::startStack(void)
{
    ((amovGimbalBase *)(this->dev))->stackStart();
}

void amovGimbal::gimbal::setParserCallback(amovGimbal::pStateInvoke callback)
{
    ((amovGimbalBase *)(this->dev))->updateGimbalStateCallback = callback;
}

void amovGimbal::amovGimbalBase::parserStart(pStateInvoke callback)
{
    this->updateGimbalStateCallback = callback;

    std::thread mainLoop(&amovGimbalBase::mainLoop, this);

    mainLoop.detach();
}
/**
 * The function creates a thread that runs the mainLoop function
 */
void amovGimbal::gimbal::parserAuto(pStateInvoke callback)
{
    ((amovGimbalBase *)(this->dev))->parserStart(callback);
}

void amovGimbal::gimbal::setMsgCallback(pMsgInvoke callback)
{
    ((amovGimbalBase *)(this->dev))->msgCustomCallback = callback;
}

amovGimbal::AMOV_GIMBAL_STATE_T amovGimbal::gimbal::getGimabalState(void)
{
    ((amovGimbalBase *)(this->dev))->mState.lock();
    AMOV_GIMBAL_STATE_T temp = ((amovGimbalBase *)(this->dev))->state;
    ((amovGimbalBase *)(this->dev))->mState.unlock();
    return temp;
}

/**
 * Default implementation of interface functions, not pure virtual functions for ease of extension.
 */
void amovGimbal::IamovGimbalBase::nodeSet(SET uint32_t _self, SET uint32_t _remote)
{
    return;
}

uint32_t amovGimbal::IamovGimbalBase::setGimabalPos(const amovGimbal::AMOV_GIMBAL_POS_T &pos)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::setGimabalSpeed(const amovGimbal::AMOV_GIMBAL_POS_T &speed)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::setGimabalFollowSpeed(const amovGimbal::AMOV_GIMBAL_POS_T &followSpeed)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::setGimabalHome(void)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::setGimbalZoom(amovGimbal::AMOV_GIMBAL_ZOOM_T zoom, float targetRate)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::setGimbalFocus(amovGimbal::AMOV_GIMBAL_ZOOM_T zoom, float targetRate)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::setGimbalROI(const amovGimbal::AMOV_GIMBAL_ROI_T area)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::takePic(void)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::setVideo(const amovGimbal::AMOV_GIMBAL_VIDEO_T newState)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::attitudeCorrection(const AMOV_GIMBAL_QUATERNION_T &quaterion, const AMOV_GIMBAL_VELOCITY_T &speed, const AMOV_GIMBAL_VELOCITY_T &acc, void *extenData)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::attitudeCorrection(const AMOV_GIMBAL_POS_T &pos, const AMOV_GIMBAL_VELOCITY_T &seppd, const AMOV_GIMBAL_VELOCITY_T &acc, void *extenData)
{
    return 0;
}

uint32_t amovGimbal::IamovGimbalBase::extensionFuntions(void *cmd)
{
    return 0;
}

/**
 * The function creates a new gimbal object, which is a pointer to a new amovGimbal object, which is a
 * pointer to a new Gimbal object, which is a pointer to a new IOStreamBase object
 *
 * @param type the type of the device, which is the same as the name of the class
 * @param _IO The IOStreamBase object that is used to communicate with the device.
 * @param _self the node ID of the device
 * @param _remote the node ID of the remote device
 */
amovGimbal::gimbal::gimbal(const std::string &type, IOStreamBase *_IO,
                           uint32_t _self, uint32_t _remote)
{
    typeName = type;
    IO = _IO;

    dev = amovGimbalCreator::createAmovGimbal(typeName, IO);

    dev->nodeSet(_self, _remote);
}

amovGimbal::gimbal::~gimbal()
{
    // 先干掉请求线程
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    delete dev;
}
