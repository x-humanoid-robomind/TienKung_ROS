/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../../core/CycleTimer.hpp"
#include "../../../io/console/Console.hpp"
#include "../../BusDevice.hpp"
#include "../../bus_types/CAN.hpp"
#include "../../variables.hpp"
#include "../general/Faultable.hpp"
#include "../general/Sensor.hpp"
#include <cmath>

#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif

namespace broccoli {
namespace hwl {

    template <typename>
    class IVRU;

    /*!
     * \brief Implements a CAN BusDevice for the iMAR Navigation iVRU-XXX inertial measurement unit
     *
     * \note This implementation currently only supports the "continuous CAN mode" with 16-bit integer values.
     * \warning The LSB sizes for the integer value interpretation must match the setting on the iVRU!
     * \ingroup broccoli_hwl_components
     */
    template <>
    class IVRU<CAN> : public BusDevice<CAN>, public Faultable, public Sensor {
    public:
        using BusType = CAN;
        using BusDevice<CAN>::BusDevice;

        /*!
         * \brief Set the LSB size for accelerations
         * \attention This must match the setting stored in the IMU!
         * \param valueOfLSB The scaling of integer values, i.e. the equivalent of the LSB in m/s
         */
        void setAccelerationLSB(double valueOfLSB) noexcept
        {
            assert(valueOfLSB > 0.0 && "LSB unit size must be greater than zero!");
            m_accelerationLSB = valueOfLSB;
        }

        /*!
         * \brief Set the LSB size for angular velocities
         * \attention This must match the setting stored in the IMU!
         * \param valueOfLSB The scaling of integer values, i.e. the equivalent of the LSB in rad/s
         */
        void setOmegaLSB(double valueOfLSB) noexcept
        {
            assert(valueOfLSB > 0.0 && "LSB unit size must be greater than zero!");
            m_omegaLSB = valueOfLSB;
        }

        /*!
         * \brief Set the LSB size for angles (RPY)
         * \attention This must match the setting stored in the IMU!
         * \param valueOfLSB The scaling of integer values, i.e. the equivalent of the LSB in rad
         */
        void setRPYLSB(double valueOfLSB) noexcept
        {
            assert(valueOfLSB > 0.0 && "LSB unit size must be greater than zero!");
            m_rpyLSB = valueOfLSB;
        }

        //! Returns true if angular rate data is available / valid
        bool omegaAvailable() const { return m_canMode & m_modeOmegaAvailable; }

        //! Returns true if linear acceleration data is available / valid
        bool accelerationsAvailable() const { return m_canMode & m_modeAccelerationAvailable; }

        //! Returns true if roll-pitch-yaw data is available / valid
        bool rpyAvailable() const { return m_canMode & m_modeRPYAvailable; }

        //! Returns the measured linear acceleration [m/s²] in x-direction (IMU-fixed frame), gravity compensated
        double accelerationX() const noexcept { return m_accelerationLSB * m_accX; }

        //! Returns the measured linear acceleration [m/s²] in y-direction (IMU-fixed frame), gravity compensated
        double accelerationY() const noexcept { return m_accelerationLSB * m_accY; }

        //! Returns the measured linear acceleration [m/s²] in z-direction (IMU-fixed frame), gravity compensated
        double accelerationZ() const noexcept { return m_accelerationLSB * m_accZ; }

        //! Returns the earth-rate-compensated angular velocity [rad/s] around the x-axis (IMU-fixed frame)
        double omegaX() const noexcept { return m_omegaLSB * m_omgX; }

        //! Returns the earth-rate-compensated angular velocity [rad/s] around the y-axis (IMU-fixed frame)
        double omegaY() const noexcept { return m_omegaLSB * m_omgY; }

        //! Returns the earth-rate-compensated angular velocity [rad/s] around the z-axis (IMU-fixed frame)
        double omegaZ() const noexcept { return m_omegaLSB * m_omgZ; }

        //! Returns the roll angle [rad] of the Euler transform (world -> IMU: yaw(z), pitch(y), roll(x))
        double roll() const noexcept { return m_rpyLSB * m_rollRaw; }

        //! Returns the pitch angle [rad] of the Euler transform (world -> IMU: yaw(z), pitch(y), roll(x))
        double pitch() const noexcept { return m_rpyLSB * m_pitchRaw; }

        //! Returns the yaw angle [rad] of the Euler transform (world -> IMU: yaw(z), pitch(y), roll(x))
        double yaw() const noexcept { return m_rpyLSB * m_yawRaw; }

        // Eigen interface
#ifdef HAVE_EIGEN3
        /*!
         * \brief Roll-Pitch-Yaw angle vector of the Euler transform (world -> IMU: yaw(z), pitch(y), roll(x))
         * \return 3d rpy vector [rad]
         */
        Eigen::Vector3d rpy() const noexcept
        {
            return m_rpyLSB * Eigen::Vector3d(m_rollRaw, m_pitchRaw, m_yawRaw);
        }

        /*!
         * \brief The measured IMU orientation as quaternion
         * \return Frame-transforming quaternion from IMU-frame to world frame
         */
        Eigen::Quaterniond orientation() const noexcept
        {
            return Eigen::AngleAxisd(yaw(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll(), Eigen::Vector3d::UnitX());
        }

        /*!
         * \brief The angular velocity vector in IMU frame
         * \return Earth-rate compensated angular velocity vector [rad/s]
         */
        Eigen::Vector3d omega() const noexcept
        {
            return m_omegaLSB * Eigen::Vector3d(m_omgX, m_omgY, m_omgZ);
        }

        /*!
         * \brief The linear acceleration vector in IMU frame
         * \return Gravity-compensated linear acceleration vector [m/s²]
         */
        Eigen::Vector3d acceleration() const noexcept
        {
            return m_accelerationLSB * Eigen::Vector3d(m_accX, m_accY, m_accZ);
        }
#endif

        bool onFault() const noexcept override
        {
            return m_onFault;
        }
        void resetFault() noexcept override
        {
            m_onFault = false;
        }

        bool ready() const noexcept override
        {
            uint16_t status = m_status;
            return (status & m_statusRPYInitMask) && (status & m_statusSensorInitMask) && !onFault();
        }

        /*!
         * \brief Set the timeout for messages from the IMU.
         *
         * When no new data arrives in the specified interval,
         * the devices switches to fault state.
         * \param timeoutDurationInMs The timeout duration in milliseconds. Defaults to 11 milliseconds
         */
        void setTimeout(uint8_t timeoutDurationInMs) noexcept
        {
            m_maximumMessageDelay = timeoutDurationInMs;
        }

        void processDevice() override
        {
            if (bus().state() != CAN::StateType::op())
                return;

            m_timer.setDuration(static_cast<std::size_t>(m_maximumMessageDelay) * 1000, bus().cycleTimeInUs());
            if (m_timer.cycle()) {
                if (m_infoSampleCounter == m_lastInfoSampleCounter && !onFault()) {
                    // Note: This implementation requires the info frame to be sent
                    // to detect available features correctly
                    m_onFault = true;
                    io::Console::error().print(prefixName("IVRU: Exceeded timeout limit for INFO data.\n"));
                }

                if (rpyAvailable() && m_rpySampleCounter == m_lastRPYSampleCounter && !onFault()) {
                    m_onFault = true;
                    io::Console::error().print(prefixName("IVRU: Exceeded timeout limit for RPY data.\n"));
                }

                if (accelerationsAvailable() && m_accSampleCounter == m_lastAccSampleCounter && !onFault()) {
                    m_onFault = true;
                    io::Console::error().print(prefixName("IVRU: Exceeded timeout limit for ACCS data.\n"));
                }

                if (omegaAvailable() && m_omgSampleCounter == m_lastOmgSampleCounter && !onFault()) {
                    m_onFault = true;
                    io::Console::error().print(prefixName("IVRU: Exceeded timeout limit for OMGS data.\n"));
                }

                m_lastInfoSampleCounter = m_infoSampleCounter;
                m_lastAccSampleCounter = m_accSampleCounter;
                m_lastOmgSampleCounter = m_omgSampleCounter;
                m_lastRPYSampleCounter = m_rpySampleCounter;
            }

            if ((m_status & m_statusErrorMask) && !onFault()) {
                m_onFault = true;
                io::Console::error().print(prefixName("IVRU: Internal IMU error! One of three error bits set. See user manual!\n"));
            }

            if ((m_status & m_statusTempOoRMask) && !onFault()) {
                m_onFault = true;
                io::Console::error().print(prefixName("IVRU: Temperature readings out of range!\n"));
            }
        }

        void onStateChange() override
        {
            if (bus().state() == CAN::StateType::op() && bus().previousState() < CAN::StateType::op()) {
                m_timer.reset();
            }
        }

        /*!
         * \brief Register the bus variable of this device
         * \param registry The BusVariableRegistry to register to
         */
        template <typename Derived>
        void linkVariables(BusVariableRegistryBase<Derived>& registry)
        {
            const uint16_t INFO = 0;
            const uint16_t ACCS = 2;
            const uint16_t OMGS = 4;
            const uint16_t NRPY = 6;

            registry.registerVariable(m_infoSampleCounter, CAN::InputMessageIdentifierType{ INFO, 0 });
            registry.registerVariable(m_status, CAN::InputMessageIdentifierType{ INFO, 2 });
            registry.registerVariable(m_canMode, CAN::InputMessageIdentifierType{ INFO, 4 });

            registry.registerVariable(m_accSampleCounter, CAN::InputMessageIdentifierType{ ACCS, 0 });
            registry.registerVariable(m_accX, CAN::InputMessageIdentifierType{ ACCS, 2 });
            registry.registerVariable(m_accY, CAN::InputMessageIdentifierType{ ACCS, 4 });
            registry.registerVariable(m_accZ, CAN::InputMessageIdentifierType{ ACCS, 6 });

            registry.registerVariable(m_omgSampleCounter, CAN::InputMessageIdentifierType{ OMGS, 0 });
            registry.registerVariable(m_omgX, CAN::InputMessageIdentifierType{ OMGS, 2 });
            registry.registerVariable(m_omgY, CAN::InputMessageIdentifierType{ OMGS, 4 });
            registry.registerVariable(m_omgZ, CAN::InputMessageIdentifierType{ OMGS, 6 });

            registry.registerVariable(m_rpySampleCounter, CAN::InputMessageIdentifierType{ NRPY, 0 });
            registry.registerVariable(m_rollRaw, CAN::InputMessageIdentifierType{ NRPY, 2 });
            registry.registerVariable(m_pitchRaw, CAN::InputMessageIdentifierType{ NRPY, 4 });
            registry.registerVariable(m_yawRaw, CAN::InputMessageIdentifierType{ NRPY, 6 });
        }

    private:
        //! Acceleration LSB [m/s²]
        double m_accelerationLSB = 0.01;

        //! Omega LSB [rad/s]
        double m_omegaLSB = 0.003 * M_PI / 180.0;

        //! RollPitchYaw LSB [rad]
        double m_rpyLSB = M_PI / 180.0;

        // CAN mode masks
        //! Acceleration data sent (integer mode)
        const uint32_t m_modeAccelerationAvailable = 1 << 1;

        //! Angular velocity data sent (integer mode)
        const uint32_t m_modeOmegaAvailable = 1 << 5;

        //! RollPitchYaw data sent (integer mode)
        const uint32_t m_modeRPYAvailable = 1 << 9;

        //! Cycle timer for timeouts
        core::CycleTimer m_timer;

        //! Fault state
        bool m_onFault = false;

        //! Max. tolerated delay for data messages in ms
        uint8_t m_maximumMessageDelay = 11;

        // Info frame
        InputBusVariable<uint16_t> m_infoSampleCounter;
        uint16_t m_lastInfoSampleCounter = 0;
        InputBusVariable<uint16_t> m_status;
        InputBusVariable<uint32_t> m_canMode;

        //! Mask for errors in the status word
        const uint16_t m_statusErrorMask = (1 << 12) | (1 << 13) | (1 << 14);

        //! Mask for temperature out-of-range bits
        const uint16_t m_statusTempOoRMask = (1 << 10) | (1 << 11);

        //! Mask for sensor initialization bit
        const uint16_t m_statusSensorInitMask = 1;

        //! Mask for RPY initialization bit
        const uint16_t m_statusRPYInitMask = 2;

        // Acceleration frame
        InputBusVariable<uint16_t> m_accSampleCounter;
        uint16_t m_lastAccSampleCounter = 0;
        InputBusVariable<int16_t> m_accX;
        InputBusVariable<int16_t> m_accY;
        InputBusVariable<int16_t> m_accZ;

        // Omega frame
        InputBusVariable<uint16_t> m_omgSampleCounter;
        uint16_t m_lastOmgSampleCounter = 0;
        InputBusVariable<int16_t> m_omgX;
        InputBusVariable<int16_t> m_omgY;
        InputBusVariable<int16_t> m_omgZ;

        // RPY frame
        InputBusVariable<uint16_t> m_rpySampleCounter;
        uint16_t m_lastRPYSampleCounter = 0;
        InputBusVariable<int16_t> m_rollRaw;
        InputBusVariable<int16_t> m_pitchRaw;
        InputBusVariable<int16_t> m_yawRaw;
    };

} // namespace hwl
} // namespace broccoli
