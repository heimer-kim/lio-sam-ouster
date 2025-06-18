
//  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xdacallback.h"

#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsdatapacket.h>

XdaCallback::XdaCallback(size_t maxBufferSize)
	: m_maxBufferSize(maxBufferSize)
{
}

XdaCallback::~XdaCallback() throw()
{
}

// Returns empty packet on timeout
RosXsDataPacket XdaCallback::next(const std::chrono::milliseconds &timeout)
{
	RosXsDataPacket packet;

	std::unique_lock<std::mutex> lock(m_mutex);

	if (m_condition.wait_for(lock, timeout, [&] { return !m_buffer.empty(); }))
	{
		assert(!m_buffer.empty());

		packet = m_buffer.front();
		m_buffer.pop_front();
	}

	return packet;
}

void XdaCallback::onLiveDataAvailable(XsDevice *, const XsDataPacket *packet)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    
    ros::Time timestamp;
    
    // 센서 타임스탬프 사용 (스케일링 수정)
    if (packet->containsSampleTime64())
    {
        uint64_t sampleTime = packet->sampleTime64();

        static uint64_t firstSampleTime = 0;
        static ros::Time firstRosTime;
        static bool initialized = false;

        if (!initialized) {
            firstSampleTime = sampleTime;
            firstRosTime = ros::Time::now();
            initialized = true;

            ROS_INFO("IMU time sync initialized:");
            ROS_INFO("  - firstSampleTime: %lu", firstSampleTime);
            ROS_INFO("  - firstRosTime: %.6f", firstRosTime.toSec());
        }

        uint64_t elapsedTicks = sampleTime - firstSampleTime;
        
        // *** 스케일링 수정 ***
        // 원래: elapsedSecs = elapsedTicks / 100.0 (4Hz 결과)
        // 수정: 실제 400Hz를 위해서는 다른 스케일 필요
        
        // 방법 1: 실제 경과 시간 기반 스케일링
        static double tick_to_sec_ratio = 0.0;
        static ros::Time last_calibration_time;
        static uint64_t last_calibration_tick;
        static int calibration_count = 0;
        
        ros::Time current_ros_time = ros::Time::now();
        
        if (calibration_count < 100) { // 처음 100샘플로 보정
            if (calibration_count == 0) {
                last_calibration_time = current_ros_time;
                last_calibration_tick = sampleTime;
            } else if (calibration_count == 99) {
                double real_elapsed_sec = (current_ros_time - last_calibration_time).toSec();
                uint64_t tick_elapsed = sampleTime - last_calibration_tick;
                
                if (tick_elapsed > 0 && real_elapsed_sec > 0) {
                    tick_to_sec_ratio = real_elapsed_sec / tick_elapsed;
                    ROS_INFO("IMU tick scaling calibrated: %.10f sec/tick", tick_to_sec_ratio);
                    ROS_INFO("This gives approximately %.1f Hz", 1.0/(tick_to_sec_ratio * (tick_elapsed/99.0)));
                }
            }
            calibration_count++;
        }
        
        if (tick_to_sec_ratio > 0) {
            double elapsedSecs = elapsedTicks * tick_to_sec_ratio;
            timestamp = firstRosTime + ros::Duration(elapsedSecs);
        } else {
            // 보정 중에는 시스템 시간 사용
            timestamp = current_ros_time;
        }
    }
    else if (packet->containsSampleTimeFine())
    {
        uint32_t sampleTimeFine = packet->sampleTimeFine();
        
        static uint32_t firstSampleTimeFine = 0;
        static ros::Time firstRosTime;
        static double fine_tick_to_sec_ratio = 0.0;
        static int fine_calibration_count = 0;
        
        if (firstSampleTimeFine == 0) {
            firstSampleTimeFine = sampleTimeFine;
            firstRosTime = ros::Time::now();
            ROS_INFO("Using fine sample timestamp - first sample: %u", firstSampleTimeFine);
        }
        
        // Fine 타임스탬프도 동일한 보정 방식 적용
        ros::Time current_ros_time = ros::Time::now();
        
        if (fine_calibration_count < 100) {
            static ros::Time fine_last_calibration_time;
            static uint32_t fine_last_calibration_tick;
            
            if (fine_calibration_count == 0) {
                fine_last_calibration_time = current_ros_time;
                fine_last_calibration_tick = sampleTimeFine;
            } else if (fine_calibration_count == 99) {
                double real_elapsed_sec = (current_ros_time - fine_last_calibration_time).toSec();
                uint32_t tick_elapsed = sampleTimeFine - fine_last_calibration_tick;
                
                if (tick_elapsed > 0 && real_elapsed_sec > 0) {
                    fine_tick_to_sec_ratio = real_elapsed_sec / tick_elapsed;
                    ROS_INFO("IMU fine tick scaling calibrated: %.10f sec/tick", fine_tick_to_sec_ratio);
                }
            }
            fine_calibration_count++;
        }
        
        if (fine_tick_to_sec_ratio > 0) {
            uint32_t elapsedTicks = sampleTimeFine - firstSampleTimeFine;
            double elapsedSecs = elapsedTicks * fine_tick_to_sec_ratio;
            timestamp = firstRosTime + ros::Duration(elapsedSecs);
        } else {
            timestamp = current_ros_time;
        }
    }
    else if (packet->containsSampleTimeCoarse())
    {
        // Coarse도 동일한 방식으로 처리
        uint16_t sampleTimeCoarse = packet->sampleTimeCoarse();
        
        static uint16_t firstSampleTimeCoarse = 0;
        static ros::Time firstRosTime;
        
        if (firstSampleTimeCoarse == 0) {
            firstSampleTimeCoarse = sampleTimeCoarse;
            firstRosTime = ros::Time::now();
            ROS_INFO("Using coarse sample timestamp - first sample: %u", firstSampleTimeCoarse);
        }
        
        // 간단한 스케일링 (실제 측정값 기반으로 나중에 조정)
        uint16_t elapsedSamples = sampleTimeCoarse - firstSampleTimeCoarse;
        double elapsedSecs = elapsedSamples / 400.0; // 400Hz 가정
        
        timestamp = firstRosTime + ros::Duration(elapsedSecs);
    }
    else
    {
        timestamp = ros::Time::now();
        static bool warned = false;
        if (!warned) {
            ROS_WARN("No sensor timestamp available, using system time");
            warned = true;
        }
    }

    assert(packet != 0);

    if (m_buffer.size() == m_maxBufferSize) {
        m_buffer.pop_front();
    }

    m_buffer.push_back(RosXsDataPacket(timestamp, *packet));

    lock.unlock();
    m_condition.notify_one();
}