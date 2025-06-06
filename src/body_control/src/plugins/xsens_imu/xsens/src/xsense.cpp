#include "xsense.h"
#include <glog/logging.h>

XsDevice* device;
CallbackHandler callback;
Eigen::VectorXd *imuData;
std::function<void(XsensImuData)> gfuncOnData = [](XsensImuData){};
bool realtime;

void *xsense_run(void *arg){
	LOG(INFO) << "Putting device into measurement mode...";
	device->gotoMeasurement();

	LOG(INFO) << "Starting recording...";
	device->startRecording();
	XsDataPacket packet;
    XsVector acc;
    XsVector gyr;
    XsEuler euler;
	XsQuaternion quaternion;

	if (realtime)
    {
		// set cpu-affinity
		int cpus = 0;
		cpu_set_t mask;

		cpus = sysconf(_SC_NPROCESSORS_CONF);
		LOG(INFO) << "cpus: " << cpus;

		CPU_ZERO(&mask);          // init mask
		CPU_SET(cpus - 1, &mask); // add last cup core to cpu set
		// CPU_SET(1, &mask); // add first cup core to cpu set


		if (sched_setaffinity(gettid(), sizeof(mask), &mask) == -1) {
			LOG(INFO) << "Set CPU affinity failue, ERROR:" << strerror(errno);
		}
		usleep(1000);
		LOG(INFO) << "set CPU affinity success";
		//   set cpu-affinity


		//set sched-strategy
		struct sched_param sched;
		int max_priority;

		max_priority = sched_get_priority_max(SCHED_RR);
		sched.sched_priority = max_priority;

		if (sched_setscheduler(gettid(), SCHED_RR, &sched) == -1) {
			LOG(INFO) << "Set Scheduler Param, ERROR:" << strerror(errno);
		}
    }

	LOG(INFO) << "start publisher thread: " << gettid();

	while (1) {
		if (callback.packetAvailable()){
			packet = callback.getNextPacket();
			acc = packet.calibratedAcceleration();
			gyr = packet.calibratedGyroscopeData();
			euler = packet.orientationEuler();
			quaternion = packet.orientationQuaternion();
			// modified by Max 2024.4.2
			// (*imuData)[0] = euler.yaw()/180.0*M_PI;
			// (*imuData)[1] = euler.pitch()/180.0*M_PI;
			// (*imuData)[2] = euler.roll()/180.0*M_PI;
			// (*imuData)[3] = gyr[0];
			// (*imuData)[4] = gyr[1];
			// (*imuData)[5] = gyr[2];
			// (*imuData)[6] = acc[0];
			// (*imuData)[7] = acc[1];
			// (*imuData)[8] = acc[2];
			// (*imuData)[9] = quaternion.w();
			// (*imuData)[10] = quaternion.x();
			// (*imuData)[11] = quaternion.y();
			// (*imuData)[12] = quaternion.z();
			gfuncOnData({acc, gyr, euler, quaternion});
		}
		usleep(1000);
	}
}
// bool xsense_init(Eigen::VectorXd *data){
bool xsense_init(std::function<void(XsensImuData)> funcOnData, bool realtime){
	// imuData = data;
	gfuncOnData = funcOnData;
	LOG(INFO) << "Creating XsControl object...";
	XsControl* control = XsControl::construct();
	assert(control != nullptr);

    // Lambda function for error handling
	auto handleError = [=](string errorString)
	{
		control->destruct();
		LOG(INFO) << errorString;
		// LOG(INFO) << "Press [ENTER] to continue.";
		// cin.get();
		exit(1);
		return -1;
	};

	LOG(INFO) << "Scanning for devices...";
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	LOG(INFO) << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate();

	LOG(INFO) << "Opening port...";
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	device = control->device(mtPort.deviceId());
	assert(device != nullptr);

	LOG(INFO) << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened.";

	// Create and attach callback handler to device
	// CallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	LOG(INFO) << "Putting device into configuration mode...";
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	LOG(INFO) << "Configuring the device...";
	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

	if (device->deviceId().isImu())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}
	else if (device->deviceId().isVru() || device->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 400));
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 400));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 400));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}
	else if (device->deviceId().isGnss())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
		configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
		configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
		configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 100));
	}
	else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");
    return true;
}

bool xsense_start(){
	pthread_t xsense_thread;
	int ret = -1;
	ret = pthread_create(&xsense_thread, NULL, xsense_run, NULL);
	if (ret != 0) {
		LOG(INFO) << "creat xsense_thread fail.";
		return false;
	}
	return true;
}