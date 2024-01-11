//----------------------------------------------------------------------------------------------------------------------------------
// LightWare SF45B ROS driver.
//----------------------------------------------------------------------------------------------------------------------------------
#include "common.h"
#include "lwNx.h"

#include <iostream>
#include <cstdint>

#include <math.h>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

std::map<int, int> updateRateMap = {
		{1, 50},
		{2, 200},
		{3, 200},
		{4, 400},
		{5, 500},
		{6, 625},
		{7, 1000},
		{8, 1250},
		{9, 1538},
		{10, 2000},
		{11, 2500},
		{12, 5000},
};

struct lwSf45Params {
	int32_t updateRate;
	int32_t cycleDelay;
	float lowAngleLimit;
	float highAngleLimit;
};

std::shared_ptr<rclcpp::Node> node;

float degreesToRadians(float degrees) {
	return 2 * M_PI * (degrees / 360);
}

void validateParams(lwSf45Params* Params) {
	if (Params->updateRate < 1) Params->updateRate = 1;
	else if (Params->updateRate > 12) Params->updateRate = 12;

	if (Params->cycleDelay < 5) Params->cycleDelay = 5;
	else if (Params->cycleDelay > 2000) Params->cycleDelay = 2000;

	if (Params->lowAngleLimit < -160) Params->lowAngleLimit = -160;
	else if (Params->lowAngleLimit > -10) Params->lowAngleLimit = -10;

	if (Params->highAngleLimit < 10) Params->highAngleLimit = 10;
	else if (Params->highAngleLimit > 160) Params->highAngleLimit = 160;
}

int driverStart(lwSerialPort** Serial, const char* PortName, int32_t BaudRate) {
	platformInit();

	lwSerialPort* serial = platformCreateSerialPort();
	*Serial = serial;
	if (!serial->connect(PortName, BaudRate)) {
		RCLCPP_ERROR(node->get_logger(), "Could not establish serial connection on %s", PortName);
		return 1;
	};

	// Disable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(serial, 30, 0)) { return 1; }

	// Read the product name. (Command 0: Product name)
	char modelName[16];
	if (!lwnxCmdReadString(serial, 0, modelName)) { return 1; }

	// Read the hardware version. (Command 1: Hardware version)
	uint32_t hardwareVersion;
	if (!lwnxCmdReadUInt32(serial, 1, &hardwareVersion)) { return 1; }

	// Read the firmware version. (Command 2: Firmware version)
	uint32_t firmwareVersion;	
	if (!lwnxCmdReadUInt32(serial, 2, &firmwareVersion)) { return 1; }
	char firmwareVersionStr[16];
	lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);

	// Read the serial number. (Command 3: Serial number)
	char serialNumber[16];
	if (!lwnxCmdReadString(serial, 3, serialNumber)) { return 1; }

	RCLCPP_INFO(node->get_logger(), "Model: %.16s", modelName);
	RCLCPP_INFO(node->get_logger(), "Hardware: %d", hardwareVersion);
	RCLCPP_INFO(node->get_logger(), "Firmware: %.16s (%d)", firmwareVersionStr, firmwareVersion);
	RCLCPP_INFO(node->get_logger(), "Serial: %.16s", serialNumber);

	return 0;
}

int driverScanStart(lwSerialPort* Serial, lwSf45Params* Params) {
	// Configre distance output for first return and angle. (Command 27: Distance output)
	if (!lwnxCmdWriteUInt32(Serial, 27, 0x101)) { return 1; }

	// (Command 66: Update rate)
	if (!lwnxCmdWriteUInt8(Serial, 66, Params->updateRate)) { return 1; }

	// (Command 85: Scan speed)
	if (!lwnxCmdWriteUInt16(Serial, 85, Params->cycleDelay)) { return 1; }

	// (Command 98: Scan low angle)
	if (!lwnxCmdWriteFloat(Serial, 98, Params->lowAngleLimit)) { return 1; }

	// (Command 99: Scan high angle)
	if (!lwnxCmdWriteFloat(Serial, 99, Params->highAngleLimit)) { return 1; }

	// Enable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(Serial, 30, 5)) { return 1; }

	return 0;
}

struct lwDistanceResult {
	float x;
	float y;
	float z;
	float intensity;
	uint16_t ring;
	float time;
};

struct rawDistanceResult
{
	float distance;
	float angle;
};

int driverScan(lwSerialPort *Serial, lwDistanceResult *DistanceResult, rawDistanceResult *RawDistanceResult)
{
	// The incoming point data packet is Command 44: Distance data in cm.
	lwResponsePacket response;

	if (lwnxRecvPacket(Serial, 44, &response, 1000)) {
		int16_t distanceCm = (response.data[5] << 8) | response.data[4];
		int16_t angleHundredths = (response.data[7] << 8) | response.data[6];

		float distance = distanceCm / 100.0f;
		float angle = angleHundredths / 100.0f;
		float faceAngle = (angle - 90) * M_PI / 180.0;

		DistanceResult->x = distance * -cos(faceAngle);
		DistanceResult->y = distance * sin(faceAngle);
		DistanceResult->z = 0;

		DistanceResult->intensity = 1.0f;
		DistanceResult->ring = 0;

		RawDistanceResult->distance = distance;
		RawDistanceResult->angle = degreesToRadians(angle);

		return 1;
	}

	return 0;
}

bool compareRawDistances(const rawDistanceResult & l, const rawDistanceResult & r) {
	return l.angle < r.angle;
}

sensor_msgs::msg::LaserScan getLaserScanMessage(
		int PointCount, std::vector<rawDistanceResult> &rawDistances, lwSf45Params *Params, float ScanTime, rclcpp::Logger Logger)
{
	// maxPointsPerMsg cuts our reading in the middle so the data is not sorted 
	std::sort(rawDistances.begin(), rawDistances.end(), compareRawDistances);
	// build laserscan output
	auto scanMessage = sensor_msgs::msg::LaserScan();
 
	float configuredMinAngle = degreesToRadians(Params->lowAngleLimit);
	float configuredMaxAngle = degreesToRadians(Params->highAngleLimit);

	// We use the min and max angles from the data, as the configured min and max are not valid as the maxPointsPerMsg
	// can cut the reading in the middle and then the configured angle max is not the real angle max
	scanMessage.angle_min = std::max(configuredMinAngle, rawDistances[0].angle);
	scanMessage.angle_max = std::min(configuredMaxAngle, rawDistances.back().angle);
	scanMessage.angle_increment = std::abs(scanMessage.angle_min - scanMessage.angle_max) / (PointCount - 1);
	scanMessage.time_increment = 0.0;
	scanMessage.scan_time = ScanTime;
	scanMessage.range_min = 0.2f;
	scanMessage.range_max = 50.0f;

	scanMessage.ranges.assign(PointCount, std::numeric_limits<double>::infinity());
	
	// Iterate through distances and validate them
	for (int i = 0; i < PointCount; i++)
	{
		rawDistanceResult &current = rawDistances[i];

		if (current.distance > scanMessage.range_max)
		{
			RCLCPP_DEBUG(
					Logger,
					"rejected for range %f above maximum value %f. Point: (%d)",
					current.distance, scanMessage.range_max, i);
			continue;
		}

		if (current.angle < configuredMinAngle || current.angle > configuredMaxAngle)
		{
			RCLCPP_DEBUG(
					Logger,
					"rejected for angle %f not in range (%f, %f)\n",
					current.angle, configuredMinAngle, configuredMaxAngle);
			continue;
		}		
		
		scanMessage.ranges[i] = current.distance;		
	}

	return scanMessage;
}


int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	node = rclcpp::Node::make_shared("sf45b");

	RCLCPP_INFO(node->get_logger(), "Starting SF45B node");
	
	auto pointCloudPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("points", 4);
	auto laserScanPub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 4);
		
	lwSerialPort* serial = 0;

	int32_t baudRate = node->declare_parameter<int32_t>("baudrate", 115200);
	std::string portName = node->declare_parameter<std::string>("port", "/dev/ttyACM0");
	std::string frameId = node->declare_parameter<std::string>("frameId", "laser_sensor_frame");
	bool publishLaserScan = node->declare_parameter<bool>("publishLaserScan", false);

	lwSf45Params params;
	params.updateRate = node->declare_parameter<int32_t>("updateRate", 7); // 1 to 12
	params.cycleDelay = node->declare_parameter<int32_t>("cycleDelay", 10); // 5 to 2000
	params.lowAngleLimit = node->declare_parameter<int32_t>("lowAngleLimit", -160.0f); // -160 to -10
	params.highAngleLimit = node->declare_parameter<int32_t>("highAngleLimit", 160.0f); // 10 to 160
	validateParams(&params);
	
	int32_t maxPointsPerMsg = node->declare_parameter<int32_t>("maxPoints", 1500); // 1 to ...
	if (maxPointsPerMsg < 1) maxPointsPerMsg = 1;

	// This debug statement will give the sizeof the lwDistanceResult struct when the node is run
	// Ensure that the size outputed matches the size variable used for memory allocation and memcpy
	RCLCPP_INFO(node->get_logger(), "Starting SF45B node. Size of lwDistanceResult: %zu bytes", sizeof(lwDistanceResult));
	
    std::cout << "Size of uint16_t: " << sizeof(uint16_t) << " bytes" << std::endl;

	if (driverStart(&serial, portName.c_str(), baudRate) != 0) {
		RCLCPP_ERROR(node->get_logger(), "Failed to start driver");
		return 1;
	}

	if (driverScanStart(serial, &params) != 0) {
		RCLCPP_ERROR(node->get_logger(), "Failed to start scan");
		return 1;
	}

	// This value can be compiler/hardware specific. It's vital to check that this matches
	// the number of bytes for an lwDistanceResult struct as padding (or lack of) can alter the memory size.
	int32_t sizeOflwDistanceResultStruct = 24;

	sensor_msgs::msg::PointCloud2 pointCloudMsg;
	pointCloudMsg.header.frame_id = frameId;
	pointCloudMsg.height = 1;
	pointCloudMsg.width = maxPointsPerMsg;
	
	pointCloudMsg.fields.resize(6);
	pointCloudMsg.fields[0].name = "x";
	pointCloudMsg.fields[0].offset = 0;	
	pointCloudMsg.fields[0].datatype = 7;
	pointCloudMsg.fields[0].count = 1;

	pointCloudMsg.fields[1].name = "y";
	pointCloudMsg.fields[1].offset = 4;
	pointCloudMsg.fields[1].datatype = 7;
	pointCloudMsg.fields[1].count = 1;
	
	pointCloudMsg.fields[2].name = "z";
	pointCloudMsg.fields[2].offset = 8;
	pointCloudMsg.fields[2].datatype = 7;
	pointCloudMsg.fields[2].count = 1;

	// Time, intensity, and ring are added to make the PointCloud2 message compatible with the velodyne
	// laser format used in the LIO-SAM package.
	pointCloudMsg.fields[3].name = "intensity";
	pointCloudMsg.fields[3].offset = 12;
	pointCloudMsg.fields[3].datatype = 7;
	pointCloudMsg.fields[3].count = 1;

	pointCloudMsg.fields[4].name = "ring";
	pointCloudMsg.fields[4].offset = 16;
	pointCloudMsg.fields[4].datatype = 4; // 4: uint16
	pointCloudMsg.fields[4].count = 1;

	pointCloudMsg.fields[5].name = "time";
	pointCloudMsg.fields[5].offset = 18;
	pointCloudMsg.fields[5].datatype = 7;
	pointCloudMsg.fields[5].count = 1;

	pointCloudMsg.is_bigendian = false;
	pointCloudMsg.point_step = sizeOflwDistanceResultStruct;
	pointCloudMsg.row_step = sizeOflwDistanceResultStruct * maxPointsPerMsg;
	pointCloudMsg.is_dense = true;

	pointCloudMsg.data = std::vector<uint8_t>(maxPointsPerMsg * sizeOflwDistanceResultStruct);

	int currentPoint = 0;

	// Used to make the PointCloud2 message compatible with the velodyne lidar format expected by LIO-SAM
	float relativeScanTime = 0.0f;

	std::vector<lwDistanceResult> distanceResults(maxPointsPerMsg);
	std::vector<rawDistanceResult> rawDistances(maxPointsPerMsg);

	while (rclcpp::ok()) {

		while (true) {

			lwDistanceResult distanceResult;
			rawDistanceResult rawDistanceResult;

			int status = driverScan(serial, &distanceResult, &rawDistanceResult);

			if (status == 0) {
				break;
			} else {
				distanceResult.time = relativeScanTime;

				distanceResults[currentPoint] = distanceResult;
				rawDistances[currentPoint] = rawDistanceResult;

				// LIO-SAM requires a relative scan time that ranges between 0 and 0.2 seconds for a 5Hz scan
				// With 5000 pps and (maxPointPerMsg = 5000) we have 0.2 / 5000 => 0.00004 increments
				relativeScanTime += 0.0002f;

				++currentPoint;
			}

			if (currentPoint == maxPointsPerMsg) {
				memcpy(&pointCloudMsg.data[0], &distanceResults[0], maxPointsPerMsg * sizeOflwDistanceResultStruct);
				auto scanTime = node->now().seconds() - pointCloudMsg.header.stamp.sec;

				pointCloudMsg.header.stamp = node->now();
				pointCloudPub->publish(pointCloudMsg);

				if (publishLaserScan)
				{
					auto laserScanMsg = getLaserScanMessage(currentPoint, rawDistances, &params, scanTime, node->get_logger());
					laserScanMsg.header = pointCloudMsg.header;

					laserScanPub->publish(laserScanMsg);
				}

				currentPoint = 0;
				relativeScanTime = 0.0f;
			}
		}

		rclcpp::spin_some(node);
	}

	rclcpp::shutdown();

	return 0;
}
