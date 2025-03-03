/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// The joystick module requires SDL2 library
#ifdef HAVE_SDL2

#include "../../memory/CircularBuffer.hpp"
#include "../../parallel/BackgroundWorker.hpp"
#include "JoystickAction.hpp"
#include "JoystickEvent.hpp"
#include "JoystickInformation.hpp"
#include "JoystickSignal.hpp"
#include "JoystickState.hpp"
#include <SDL.h>
#include <array>
#include <sstream>
#include <string>
#include <vector>

namespace broccoli {
namespace io {
    //! Joystick/Gamepad handling class
    /*!
     * \ingroup broccoli_io_joystick
     * Wrapper class for accessing joystick/game controller input. Uses the SDL library (cross plattform)
     * to connect to the device. If available also gives an interface to haptic feedback of the controller.
     *
     * Creates a background thread, which continuously polls events from the input device. The events are stored
     * in a circular buffer which can be read one by one. Further the current state of each button/axis is provided.
     * Haptic feedback actions may be added to another circular buffer.
     */
    class Joystick final : public parallel::BackgroundWorker {
    public:
        //! Constructor (runs in parent thread)
        /*!
         * \param [in] threadPriority Initializes: \copybrief threadPriority()
         * \param [in] desiredDeviceName Initializes: \copybrief desiredDeviceName()
         */
        Joystick(const int& threadPriority, const std::string& desiredDeviceName = "")
            : BackgroundWorker("joystick", true, threadPriority, 0.001 /* <-- max. 1kHz */, false) // Setup background thread for low CPU usage (data is buffered -> joystick input is NOT time-critical)
            , m_device(NULL)
            , m_deviceHaptic(NULL)
            , m_lastActionStarted(0)
            , m_lastActionDuration(0)
            , m_isSDLInitialized(false)
            , m_isConnected(false)
            , m_desiredDeviceName(desiredDeviceName)
            , m_motionThreshold(10000)
            , m_signalBuffer(1000)
            , m_eventBuffer(100)
            , m_actionBuffer(100)
        {
        }

    private:
        //! Copy constructor (internal) (**not** thread-safe -> should only be called by the thread-safe wrapper)
        /*! \param [in] original Reference to original object. */
        Joystick(const Joystick& original, const int& /* <- trick used for locking mutex */)
            : BackgroundWorker(original)
            , m_device(NULL) // m_device = DO NOT COPY FROM ORIGINAL (use own handle)
            , m_deviceHaptic(NULL) // m_deviceHaptic = DO NOT COPY FROM ORIGINAL (use own handle)
            , m_lastActionStarted(original.m_lastActionStarted)
            , m_lastActionDuration(original.m_lastActionDuration)
            , m_isSDLInitialized(original.m_isSDLInitialized)
            , m_isConnected(original.m_isConnected)
            , m_desiredDeviceName(original.m_desiredDeviceName)
            , m_motionThreshold(original.m_motionThreshold)
            , m_availableJoysticks(original.m_availableJoysticks)
            , m_activeJoystick(original.m_activeJoystick)
            , m_initialState(original.m_initialState)
            , m_currentState(original.m_currentState)
            , m_signalBuffer(original.m_signalBuffer)
            , m_eventBuffer(original.m_eventBuffer)
            , m_actionBuffer(original.m_actionBuffer)
        {
        }

    public:
        //! Copy constructor (wrapper) (**thread-safe**)
        /*!
         * Locks the mutex of the original object for reading before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] original Reference to original object.
         */
        Joystick(const Joystick& original)
            : Joystick(original, original.lockForRead() /* <- lock mutex of original object first (for reading since also using "const") */)
        {
            original.unlock(); // Unlock mutex of original object after internal (protected) constructor has been called
        }

        //! Copy assignment operator (**thread-safe**)
        /*!
         * Uses own mutex and mutex of the reference object to guarantee thread-safe copying of members.
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        Joystick& operator=(const Joystick& reference)
        {
            // Avoid self-assignment
            if (this == &reference)
                return *this;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // spinning
                lockForWrite(); // Lock ourselves for writing (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Copy data
            BackgroundWorker::operator=(reference);
            m_device = NULL; // m_device = DO NOT COPY FROM ORIGINAL (use own handle)
            m_deviceHaptic = NULL; // m_deviceHaptic = DO NOT COPY FROM ORIGINAL (use own handle)
            m_lastActionStarted = reference.m_lastActionStarted;
            m_lastActionDuration = reference.m_lastActionDuration;
            m_isSDLInitialized = reference.m_isSDLInitialized;
            m_isConnected = reference.m_isConnected;
            m_desiredDeviceName = reference.m_desiredDeviceName;
            m_motionThreshold = reference.m_motionThreshold;
            m_availableJoysticks = reference.m_availableJoysticks;
            m_activeJoystick = reference.m_activeJoystick;
            m_initialState = reference.m_initialState;
            m_currentState = reference.m_currentState;
            m_signalBuffer = reference.m_signalBuffer;
            m_eventBuffer = reference.m_eventBuffer;
            m_actionBuffer = reference.m_actionBuffer;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return *this;
        }

        //! Destructor
        ~Joystick()
        {
            // De-initialize (if not already triggered)
            deInitialize();
        }

        //! Initialize the \ref Joystick (runs in parent thread)
        /*!
         * Initializes SDL library and triggers start of background thread.
         * \return `true` on success, `false` on failure
         */
        bool initialize()
        {
            // Get initialization status of SDL modules
            uint32_t initializedSubsystems = SDL_WasInit(SDL_INIT_EVERYTHING);

            // Initialize SDL modules
            SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1"); // Prevent SDL from catching SIGINT which is necessary to be able to abort a console application by CTRL-C
            if (initializedSubsystems & SDL_INIT_JOYSTICK) {
                // Joystick module has already been initialized -> do nothing
            } else {
                // Joystick module has not been initialized yet -> initialize
                if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) != 0) {
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::ERROR, JoystickSignal::Message::SDL_INIT_FAILED, SDL_GetError()));
                    return false;
                }
            }
            if (initializedSubsystems & SDL_INIT_HAPTIC) {
                // Haptics module has already been initialized -> do nothing
            } else {
                // Haptics module has not been initialized yet -> initialize
                if (SDL_InitSubSystem(SDL_INIT_HAPTIC) != 0) {
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::ERROR, JoystickSignal::Message::SDL_INIT_FAILED, SDL_GetError()));
                    return false;
                }
            }

            // De-initialize SDL at program exit
            atexit(SDL_Quit);

            // Remember initalization state
            setIsSDLInitialized(true);

            // Start background thread
            return start();
        }

        //! De-initialize the \ref Joystick (runs in parent thread)
        /*!
         * Waits until background thread has finished its work.
         * Closes connection to active device and clears all buffers.
         *
         * \param [in] timeout Timeout for waiting until background thread finished its work.
         * \return `true` if background worker was shutdown properly, `false` otherwise
         */
        bool deInitialize(const core::Time& timeout = core::Time(1))
        {
            // Join background thread and remember result
            bool threadJoinSuccessful = join(timeout);

            // Close connection to active device
            deviceClose();

            // Clear buffers
            m_signalBuffer.clear();
            m_eventBuffer.clear();
            m_actionBuffer.clear();

            // Pass back result
            return threadJoinSuccessful;
        }

    private:
        // Hide base class functionality (use initialize() and deInitialize() instead!)
        using BackgroundWorker::join;
        using BackgroundWorker::start;
        using BackgroundWorker::stop;

    private:
        //! Main execution loop (runs in background thread)
        /*!
         * Tries to establish connection (if not already connected). Further tries to grab
         * new events from driver. Adds new event to buffer and updates state of joystick.
         * Sends action of buffer to the driver one by one.
         */
        void execute()
        {
            // Skip, if SDL has not been initialized properly
            if (isSDLInitialized() == false)
                return;

            // Step 1: Check, if desired device name differs from connected device and disconnect in this case
            // -----------------------------------------------------------------------------------------------
            if (isConnected() == true) {
                // Compare names
                std::string currentDesiredDeviceName = desiredDeviceName();
                JoystickInformation currentActiveJoystick = activeJoystick();
                if (currentDesiredDeviceName != "" && currentDesiredDeviceName != currentActiveJoystick.m_name) {
                    // Names do not match (and name is not arbitrary) -> disconnect
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::INFORMATION, JoystickSignal::Message::DISCONNECT_DESIRED_DEVICE_CHANGED));
                    deviceClose();
                }
            }

            // Step 2: Check, if connection is up and running, otherwise reconnect
            // -------------------------------------------------------------------
            if (isConnected() == false) {
                // Get list of available joysticks
                // -------------------------------
                updateAvailableDevices();
                std::vector<JoystickInformation> newAvailableJoysticks = availableJoysticks(); // Grab new device list

                // Check, if at least one device has been found
                if (newAvailableJoysticks.size() == 0)
                    return;

                // Check, if the desired name is in the list
                // -----------------------------------------
                int desiredDeviceIndex = -1; // Index of desired device in list
                std::string currentDesiredDeviceName = desiredDeviceName();
                if (currentDesiredDeviceName == "")
                    desiredDeviceIndex = 0; // Arbitrary identifier -> use first in line
                else {
                    // Not arbitrary -> Search for identifier
                    for (size_t i = 0; i < newAvailableJoysticks.size(); i++) {
                        if (newAvailableJoysticks[i].m_name == currentDesiredDeviceName) {
                            // Identifier found -> stop searching
                            desiredDeviceIndex = i;
                            break;
                        }
                    }
                }

                // Abort, if name not found
                if (desiredDeviceIndex == -1) {
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::CONNECT_FAILED_DESIRED_DEVICE_NOT_FOUND));
                    return;
                }

                // Swap buffers to allow external access to device information of active joystick
                setActiveJoystick(newAvailableJoysticks[desiredDeviceIndex]);

                // Try to (re-)connect
                // -------------------
                // Disconnect first, if there is already an open connection
                deviceClose();

                // Try to connect to device
                if (deviceOpen(desiredDeviceIndex) == true) {
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::INFORMATION, JoystickSignal::Message::CONNECT_SUCCESS));
                    setIsConnected(true);

                    // Clear action buffer (do not perform actions of "previous" device)
                    m_actionBuffer.clear();

                    // Initialize initial state
                    JoystickState initialState;
                    initialState.m_axisValue.resize(newAvailableJoysticks[desiredDeviceIndex].m_numberOfAxes);
                    initialState.m_buttonPressed.resize(newAvailableJoysticks[desiredDeviceIndex].m_numberOfButtons);
                    initialState.m_hatValue.resize(newAvailableJoysticks[desiredDeviceIndex].m_numberOfHats);

                    // Try to obtain initial axis values
                    for (size_t i = 0; i < initialState.m_axisValue.size(); i++) {
                        // Project axis value to plausible default value (try to compensate error)
                        int16_t defaultValue = SDL_JoystickGetAxis(m_device, i);
                        if (defaultValue > 16384)
                            defaultValue = 32767;
                        else if (defaultValue < -16384)
                            defaultValue = -32767;
                        else
                            defaultValue = 0;
                        initialState.m_axisValue[i] = defaultValue;
                    }

                    // Try to obtain initial button state
                    for (size_t i = 0; i < initialState.m_buttonPressed.size(); i++)
                        initialState.m_buttonPressed[i] = SDL_JoystickGetButton(m_device, i);

                    // Try to obtain initial hat values
                    for (size_t i = 0; i < initialState.m_hatValue.size(); i++)
                        initialState.m_hatValue[i] = SDL_JoystickGetHat(m_device, i);

                    // Swap buffer: set initial state and current state
                    setInitialState(initialState);
                    setCurrentState(initialState);
                } else
                    return;
            }

            // Step 3: Poll event buffer of driver and fill own buffer
            // -------------------------------------------------------
            // Grab initial state (as reference/default state)
            JoystickState defaultState = initialState();

            // Grab current state (to be modified and updated by events)
            JoystickState newState = currentState();

            // Grob configuration
            unsigned int threshold = motionThreshold();

            // Loop through event buffer
            SDL_Event SDLevent;
            while (SDL_PollEvent(&SDLevent)) {
                // Emergency exit point
                if (stopTriggered() == true)
                    return;

                // Initialize new event (to be added to buffer)
                JoystickEvent newEvent;
                newEvent.m_type = static_cast<SDL_EventType>(SDLevent.type);

                // Check event type
                switch (SDLevent.type) {
                // Axis motion
                case SDL_JOYAXISMOTION: {
                    // Check, if axis index is within limits
                    if (SDLevent.jaxis.axis < newState.m_axisValue.size() && SDLevent.jaxis.axis < defaultState.m_axisValue.size()) {
                        // Get time stamp
                        newEvent.m_timestamp = SDLevent.jaxis.timestamp;

                        // Apply threshold to get rid of "drift" in default position
                        int16_t projectedValue = SDLevent.jaxis.value;
                        if (abs(SDLevent.jaxis.value - defaultState.m_axisValue[SDLevent.jaxis.axis]) < (int)threshold)
                            projectedValue = defaultState.m_axisValue[SDLevent.jaxis.axis];

                        // Change current state
                        newState.m_axisValue[SDLevent.jaxis.axis] = projectedValue;
                        setCurrentState(newState);

                        // Setup new event and add it to the buffer
                        newEvent.m_index = SDLevent.jaxis.axis;
                        newEvent.m_value[0] = projectedValue;
                        m_eventBuffer.push(newEvent);
                    }
                    break;
                }
                // Ball motion (relative x and y)
                case SDL_JOYBALLMOTION: {
                    // Get time stamp
                    newEvent.m_timestamp = SDLevent.jball.timestamp;

                    // Setup new event and add it to the buffer
                    newEvent.m_index = SDLevent.jball.ball;
                    newEvent.m_value[0] = SDLevent.jball.xrel;
                    newEvent.m_value[1] = SDLevent.jball.yrel;
                    m_eventBuffer.push(newEvent);
                    break;
                }
                // Hat motion (discrete directions)
                case SDL_JOYHATMOTION: {
                    // Check, if hat index is within limits
                    if (SDLevent.jhat.hat < newState.m_hatValue.size()) {
                        // Get time stamp
                        newEvent.m_timestamp = SDLevent.jhat.timestamp;

                        // Change current state
                        newState.m_hatValue[SDLevent.jhat.hat] = SDLevent.jhat.value;
                        setCurrentState(newState);

                        // Create new event and add it to the buffer
                        newEvent.m_index = SDLevent.jhat.hat;
                        newEvent.m_value[0] = SDLevent.jhat.value;
                        m_eventBuffer.push(newEvent);
                    }
                    break;
                }
                // Button event (press/release)
                case SDL_JOYBUTTONDOWN:
                case SDL_JOYBUTTONUP: {
                    // Check, if button index is within limits
                    if (SDLevent.jbutton.button < newState.m_buttonPressed.size()) {
                        // Get time stamp
                        newEvent.m_timestamp = SDLevent.jbutton.timestamp;

                        // Change current state
                        newState.m_buttonPressed[SDLevent.jbutton.button] = SDLevent.jbutton.state;
                        setCurrentState(newState);

                        // Setup new event and add it to the buffer
                        newEvent.m_index = SDLevent.jbutton.button;
                        newEvent.m_value[0] = SDLevent.jbutton.state;
                        m_eventBuffer.push(newEvent);
                    }
                    break;
                }
                // Adding/Removing devices
                case SDL_JOYDEVICEADDED:
                case SDL_JOYDEVICEREMOVED: {
                    // Either a joystick got added or removed -> check, if we should be connected
                    if (isConnected() == true) {
                        // ...we think that we are connected -> check if connection was lost
                        if (SDL_JoystickGetAttached(m_device) == SDL_FALSE) {
                            // Joystick has been detached -> disconnect
                            m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::DISCONNECT_DEVICE_DETACHED));
                            deviceClose();

                            // Grab new list of available joysticks
                            updateAvailableDevices();
                        }
                        // else: we still are connected -> do not update device list since this would disconnect us
                    } else if (isConnected() == false) {
                        // ... we are not connected anyway -> update device list
                        updateAvailableDevices();
                    }

                    // Pass through event to higher level structures
                    m_eventBuffer.push(newEvent);
                    break;
                }
                case SDL_QUIT: {
                    // User defined quit -> stop background thread
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::INFORMATION, JoystickSignal::Message::SDL_QUIT_SIGNAL));
                    stop(); // Stop background thread
                    return;
                }
                }
            }

            // Step 4: Perform actions in action buffer
            // ----------------------------------------
            // Wait until last action has finished properly
            if (core::Time::currentTime() > m_lastActionStarted + m_lastActionDuration) {
                // Get next action (if there is one)
                JoystickAction nextAction;
                if (m_actionBuffer.pop(nextAction) != 0) {
                    // Skip, if haptic feedback is not available
                    if (m_deviceHaptic == NULL)
                        m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::ACTION_FAILED_NO_HAPTICS));
                    else {
                        // Initialize helpers
                        bool actionTriggered = false; // Has an action been triggered?

                        // Check type of action
                        switch (nextAction.m_type) {
                        case JoystickAction::Type::WAIT: {
                            actionTriggered = true;
                            break;
                        }
                        case JoystickAction::Type::HAPTIC_RUMBLE: {
                            if (SDL_HapticRumblePlay(m_deviceHaptic, nextAction.m_intensity, nextAction.m_duration) != 0)
                                m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::ACTION_FAILED_HAPTIC_RUMBLE, SDL_GetError()));
                            else
                                actionTriggered = true;
                            break;
                        }
                        default: {
                            m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::ERROR, JoystickSignal::Message::ACTION_FAILED_UNKNOWN));
                            break;
                        }
                        }

                        // Remember time when this action has been triggered
                        if (actionTriggered == true) {
                            m_lastActionStarted = core::Time::currentTime();
                            m_lastActionDuration = core::Time(0, nextAction.m_duration * 1000000LL);
                        }
                    }
                }
            }
        }

        //! Grab list of all available joystick
        /*!
         * \warning Disconnects and clears event and action buffer!
         */
        void updateAvailableDevices()
        {
            // Disconnect first, if there is already an open connection
            deviceClose();

            // Remember old device list
            std::vector<JoystickInformation> oldAvailableJoysticks = availableJoysticks();

            // Create empty list of joysticks
            std::vector<JoystickInformation> newAvailableJoysticks;

            // Iterate through all attached joysticks
            SDL_JoystickUpdate(); // Update state of all joysticks
            for (int i = 0; i < SDL_NumJoysticks(); i++) {
                // Try to open connection to device
                if (deviceOpen(i) == true) {
                    // Create new device candidate
                    JoystickInformation newCandidate;

                    // Try to obtain device information
                    if (deviceReadInformation(newCandidate) == true) {
                        // Success -> add candidate to list and close connection
                        newAvailableJoysticks.push_back(newCandidate);
                    }
                }

                // Close connection again
                deviceClose();
            }

            // Swap buffers to allow external access to device list
            setAvailableJoysticks(newAvailableJoysticks);

            // Check, if list of available devices has changed
            bool listHasChanged = false;
            if (oldAvailableJoysticks.size() != newAvailableJoysticks.size())
                listHasChanged = true;
            else {
                // Length is the same, but are the items the same? -> compare all items
                for (size_t i = 0; i < oldAvailableJoysticks.size(); i++) {
                    if (oldAvailableJoysticks[i] != newAvailableJoysticks[i]) {
                        listHasChanged = true;
                        break;
                    }
                }
            }

            // Trigger signal to inform supervisor (only if list of devices changed)
            if (listHasChanged == true)
                m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::INFORMATION, JoystickSignal::Message::UPDATE_DEVICE_LIST));
        }

        // Low level encapsulated device functions
        // ---------------------------------------
        //! Tries to open device (runs in background thread)
        /*!
         * Convenience function to catch low-level exceptions.
         * \param [in] deviceIndex Index of device to try to open the connection to.
         * \return `true` on success, `false` on failure
         */
        bool deviceOpen(const int& deviceIndex)
        {
            // Skip, if SDL has not been initialized properly
            if (isSDLInitialized() == false)
                return false;

            // Otherwise try to open
            bool success = false;
            try {
                m_device = SDL_JoystickOpen(deviceIndex);
                if (m_device == NULL) {
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::ERROR, JoystickSignal::Message::CONNECT_FAILED_SDL_OPEN, SDL_GetError()));
                    success = false;
                } else {
                    // Try to get haptic information
                    int isHaptic = SDL_JoystickIsHaptic(m_device);
                    if (isHaptic < 0) {
                        m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::READ_INFO_FAILED_HAPTIC, SDL_GetError()));

                        // Haptic feedback is optional -> count as success
                        m_deviceHaptic = NULL;
                        success = true;
                    } else {
                        // Check, if device supports haptic feedback...
                        if (isHaptic == SDL_TRUE) {
                            // Try to connect to haptic device
                            m_deviceHaptic = SDL_HapticOpenFromJoystick(m_device);
                            if (m_deviceHaptic == NULL) {
                                m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::CONNECT_FAILED_HAPTICS, SDL_GetError()));

                                // Haptic feedback is optional -> count as success
                                success = true;
                            } else {
                                // Initialize simple rumble
                                if (SDL_HapticRumbleInit(m_deviceHaptic) != 0) {
                                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::CONNECT_FAILED_HAPTICS, SDL_GetError()));

                                    // Haptic feedback is optional -> count as success
                                    SDL_HapticClose(m_deviceHaptic);
                                    m_deviceHaptic = NULL;
                                    success = true;
                                } else
                                    success = true;
                            }
                        } else {
                            // Device has no haptic feedback, but this is an optional feature -> count as success
                            m_deviceHaptic = NULL;
                            success = true;
                        }
                    }
                }
            } catch (...) {
                success = false;
            }
            // Update flag (only in failure - in case of success we only opened device to read information -> this does NOT count as connection)
            if (success == false)
                setIsConnected(false);
            return success;
        }

        //! Tries to close device (runs in own **or** background thread)
        /*!
         * Convenience function to catch low-level exceptions.
         * \return true on success, false on failure
         */
        bool deviceClose()
        {
            // Skip, if SDL has not been initialized properly
            if (isSDLInitialized() == false)
                return false;

            // Otherwise try to close
            bool success = false;
            try {
                // Close connection to haptic interface
                if (m_deviceHaptic != NULL) {
                    SDL_HapticClose(m_deviceHaptic);
                    m_deviceHaptic = NULL;
                }

                // Close connection to joystick
                if (m_device != NULL) {
                    SDL_JoystickClose(m_device);
                    m_device = NULL;
                }

                success = true;
            } catch (...) {
                success = false;
            }
            setIsConnected(false);
            return success;
        }

        //! Tries to read general device information (runs in background thread)
        /*!
         * Convenience function to catch low-level exceptions.
         * Takes information of the **currently opened** device.
         * \param [out] information Joystick information container to store the data into.
         * \return `true on success`, `false` on failure
         */
        bool deviceReadInformation(JoystickInformation& information)
        {
            // Skip, if SDL has not been initialized properly
            if (isSDLInitialized() == false)
                return false;

            // Skip, if device is not ready (in this case reading information makes no sense)
            if (m_device == NULL)
                return false;

            // Otherwise try to read information
            bool success = false;
            try {
                // Try to get name
                if (SDL_JoystickName(m_device) != NULL)
                    information.m_name = std::string(SDL_JoystickName(m_device));
                else
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::READ_INFO_FAILED_NAME, SDL_GetError()));

                // Try to get GUID information
                SDL_JoystickGUID GUID = SDL_JoystickGetGUID(m_device);
                bool nonZeroEntryFound = false;
                for (size_t i = 0; i < sizeof(GUID.data) / sizeof(GUID.data[0]); i++)
                    if (GUID.data[i] != 0)
                        nonZeroEntryFound = true;
                if (nonZeroEntryFound == false)
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::READ_INFO_FAILED_GUID, SDL_GetError()));
                else {
                    information.m_GUID = GUID;

                    // Convert GUID to ASCII string
                    char GUIDAscii[1024];
                    SDL_JoystickGetGUIDString(GUID, GUIDAscii, sizeof(GUIDAscii));
                    information.m_GUIDstring = std::string(GUIDAscii);
                }

                // Try to get number of axes
                int numberOfAxes = SDL_JoystickNumAxes(m_device);
                if (numberOfAxes < 0)
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::READ_INFO_FAILED_AXES, SDL_GetError()));
                else
                    information.m_numberOfAxes = numberOfAxes;

                // Try to get number of balls
                int numberOfBalls = SDL_JoystickNumBalls(m_device);
                if (numberOfBalls < 0)
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::READ_INFO_FAILED_BALLS, SDL_GetError()));
                else
                    information.m_numberOfBalls = numberOfBalls;

                // Try to get number of buttons
                int numberOfButtons = SDL_JoystickNumButtons(m_device);
                if (numberOfButtons < 0)
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::READ_INFO_FAILED_BUTTONS, SDL_GetError()));
                else
                    information.m_numberOfButtons = numberOfButtons;

                // Try to get number of hats
                int numberOfHats = SDL_JoystickNumHats(m_device);
                if (numberOfHats < 0)
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::READ_INFO_FAILED_HATS, SDL_GetError()));
                else
                    information.m_numberOfHats = numberOfHats;

                // Try to get haptic information
                int isHaptic = SDL_JoystickIsHaptic(m_device);
                if (isHaptic < 0)
                    m_signalBuffer.push(JoystickSignal(JoystickSignal::Type::WARNING, JoystickSignal::Message::READ_INFO_FAILED_HAPTIC, SDL_GetError()));
                else {
                    if (isHaptic == SDL_TRUE)
                        information.m_isHaptic = true;
                    else
                        information.m_isHaptic = false;
                }

                // Try to get power level information
                information.m_powerLevel = SDL_JoystickCurrentPowerLevel(m_device);

                // All information has been read -> success
                success = true;
            } catch (...) {
                success = false;
            }
            if (success == false) {
                // Something went wrong -> close connection
                deviceClose();
            }
            return success;
        }

        // Protected data
        // --------------
        // Device objects (no mutex necessary, since these are only accessed by protected functions)
        SDL_Joystick* m_device; //!< Interface for communication with the device
        SDL_Haptic* m_deviceHaptic; //!< Haptic interface of the device
        core::Time m_lastActionStarted; //!< System time when last action was triggered
        core::Time m_lastActionDuration; //!< Duration of last triggered action

        // Start protected area of mutex of background thread
        bool m_isSDLInitialized; //!< \copybrief isSDLInitialized()
        bool m_isConnected; //!< \copybrief isConnected()
        std::string m_desiredDeviceName; //!< \copybrief desiredDeviceName()
        unsigned int m_motionThreshold; //!< \copybrief motionThreshold()
        std::vector<JoystickInformation> m_availableJoysticks; //!< \copybrief availableJoysticks()
        JoystickInformation m_activeJoystick; //!< \copybrief activeJoystick()
        JoystickState m_initialState; //!< \copybrief initialState()
        JoystickState m_currentState; //!< \copybrief currentState()
        // End protected area of mutex of background thread

    public:
        // Buffers (thread safe on their own)
        memory::CircularBuffer<JoystickSignal> m_signalBuffer; //!< Buffer containing signals of the joystick module (**thread-safe**)
        memory::CircularBuffer<JoystickEvent> m_eventBuffer; //!< Buffer containing new received events (**thread-safe**)
        memory::CircularBuffer<JoystickAction> m_actionBuffer; //!< Buffer containing actions to perform (**thread-safe**)

        // Getters (thread-safe)
        // ---------------------
    public:
        // Dynamic members
        //! Flag indicating, if SDL was initialized successfully \details **Thread-safe getter**
        inline bool isSDLInitialized() const { return getProtectedData(m_isSDLInitialized); }
        //! Flag indicating the connection status \details **Thread-safe getter**
        inline bool isConnected() const { return getProtectedData(m_isConnected); }
        //! Desired device name to connect to the right device (if "", the first available device is chosen) \details **Thread-safe getter**
        inline std::string desiredDeviceName() const { return getProtectedData(m_desiredDeviceName); }
        //! Threshold for triggering motion events (to avoid "drift" in default position of axis) \details **Thread-safe getter**
        inline unsigned int motionThreshold() const { return getProtectedData(m_motionThreshold); }
        //! List of available joysticks and their information (only gets updated if disconnected) \details **Thread-safe getter**
        inline std::vector<JoystickInformation> availableJoysticks() const { return getProtectedData(m_availableJoysticks); }
        //! Device information of active joystick \details **Thread-safe getter**
        inline JoystickInformation activeJoystick() const { return getProtectedData(m_activeJoystick); }
        //! Initial state of joystick (axes, buttons and hats) at time of opening the connection (used as "default" state) \details **Thread-safe getter**
        inline JoystickState initialState() const { return getProtectedData(m_initialState); }
        //! Current state of joystick (axes, buttons and hats) \details **Thread-safe getter**
        inline JoystickState currentState() const { return getProtectedData(m_currentState); }

        // Setters (thread-safe)
        // ---------------------
    private:
        // Dynamic members
        //! **Thread-safe setter for:** \copybrief isSDLInitialized()
        inline void setIsSDLInitialized(const bool& newValue) { setProtectedData(m_isSDLInitialized, newValue); }
        //! **Thread-safe setter for:** \copybrief isConnected()
        inline void setIsConnected(const bool& newValue) { setProtectedData(m_isConnected, newValue); }
        //! **Thread-safe setter for:** \copybrief availableJoysticks()
        inline void setAvailableJoysticks(const std::vector<JoystickInformation>& newValue) { setProtectedData(m_availableJoysticks, newValue); }
        //! **Thread-safe setter for:** \copybrief activeJoystick()
        inline void setActiveJoystick(const JoystickInformation& newValue) { setProtectedData(m_activeJoystick, newValue); }
        //! **Thread-safe setter for:** \copybrief initialState()
        inline void setInitialState(const JoystickState& newValue) { setProtectedData(m_initialState, newValue); }
        //! **Thread-safe setter for:** \copybrief currentState()
        inline void setCurrentState(const JoystickState& newValue) { setProtectedData(m_currentState, newValue); }

    public:
        //! **Thread-safe setter for:** \copybrief desiredDeviceName()
        inline void setDesiredDeviceName(const std::string& newValue) { setProtectedData(m_desiredDeviceName, newValue); }
        //! **Thread-safe setter for:** \copybrief motionThreshold()
        inline void setMotionThreshold(const unsigned int& newValue) { setProtectedData(m_motionThreshold, newValue); }
    };
} // namespace io
} // namespace broccoli

#endif // HAVE_SDL2
