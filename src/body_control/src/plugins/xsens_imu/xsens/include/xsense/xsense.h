#ifndef XENSE_h
#define XENSE_h

#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <list>
#include <unistd.h>

#include "xsensdeviceapi.h"
#include "xstime.h"
#include "xsens_mutex.h"


#include "../broccoli/core/Time.hpp"

using namespace std;

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != nullptr);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;
	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};

// added by Max 2024.4.2
struct XsensImuData {
    XsVector acc;
    XsVector gyr;
    XsEuler euler;
	XsQuaternion quaternion;
};

void *xsense_run(void *arg);
// modified by Max 2024.4.2
// bool xsense_init(Eigen::VectorXd *data);
bool xsense_init(std::function<void(XsensImuData)> funcOnData, bool realtime);
bool xsense_start();

#endif
#pragma once