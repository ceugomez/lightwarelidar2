//----------------------------------------------------------------------------------------------------------------------------------
// LightWare SF45B ROS driver.
//----------------------------------------------------------------------------------------------------------------------------------
#include "common.h"
#include "lwNx.h"

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

struct lwSf45Params
{
	int32_t updateRate;
	int32_t cycleDelay;
	float lowAngleLimit;
	float highAngleLimit;
};

std::shared_ptr<rclcpp::Node> node;

void validateParams(lwSf45Params *Params)
{
	if (Params->updateRate < 1)
		Params->updateRate = 1;
	else if (Params->updateRate > 12)
		Params->updateRate = 12;

	if (Params->cycleDelay < 5)
		Params->cycleDelay = 5;
	else if (Params->cycleDelay > 2000)
		Params->cycleDelay = 2000;

	if (Params->lowAngleLimit < -160)
		Params->lowAngleLimit = -160;
	else if (Params->lowAngleLimit > -10)
		Params->lowAngleLimit = -10;

	if (Params->highAngleLimit < 10)
		Params->highAngleLimit = 10;
	else if (Params->highAngleLimit > 160)
		Params->highAngleLimit = 160;
}

int driverStart(lwSerialPort **Serial, const char *PortName, int32_t BaudRate)
{
	platformInit();

	lwSerialPort *serial = platformCreateSerialPort();
	*Serial = serial;
	if (!serial->connect(PortName, BaudRate))
	{
		RCLCPP_ERROR(node->get_logger(), "Could not establish serial connection on %s", PortName);
		return 1;
	};

	// Disable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(serial, 30, 0))
	{
		return 1;
	}

	// Read the product name. (Command 0: Product name)
	char modelName[16];
	if (!lwnxCmdReadString(serial, 0, modelName))
	{
		return 1;
	}

	// Read the hardware version. (Command 1: Hardware version)
	uint32_t hardwareVersion;
	if (!lwnxCmdReadUInt32(serial, 1, &hardwareVersion))
	{
		return 1;
	}

	// Read the firmware version. (Command 2: Firmware version)
	uint32_t firmwareVersion;
	if (!lwnxCmdReadUInt32(serial, 2, &firmwareVersion))
	{
		return 1;
	}
	char firmwareVersionStr[16];
	lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);

	// Read the serial number. (Command 3: Serial number)
	char serialNumber[16];
	if (!lwnxCmdReadString(serial, 3, serialNumber))
	{
		return 1;
	}

	RCLCPP_INFO(node->get_logger(), "Model: %.16s", modelName);
	RCLCPP_INFO(node->get_logger(), "Hardware: %d", hardwareVersion);
	RCLCPP_INFO(node->get_logger(), "Firmware: %.16s (%d)", firmwareVersionStr, firmwareVersion);
	RCLCPP_INFO(node->get_logger(), "Serial: %.16s", serialNumber);

	return 0;
}

int driverScanStart(lwSerialPort *Serial, lwSf45Params *Params)
{
	// Configre distance output for first return and angle. (Command 27: Distance output)
	if (!lwnxCmdWriteUInt32(Serial, 27, 0x101))
	{
		return 1;
	}

	// (Command 66: Update rate)
	if (!lwnxCmdWriteUInt8(Serial, 66, Params->updateRate))
	{
		return 1;
	}

	// (Command 85: Scan speed)
	if (!lwnxCmdWriteUInt16(Serial, 85, Params->cycleDelay))
	{
		return 1;
	}

	// (Command 98: Scan low angle)
	if (!lwnxCmdWriteFloat(Serial, 98, Params->lowAngleLimit))
	{
		return 1;
	}

	// (Command 99: Scan high angle)
	if (!lwnxCmdWriteFloat(Serial, 99, Params->highAngleLimit))
	{
		return 1;
	}

	// Enable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(Serial, 30, 5))
	{
		return 1;
	}

	return 0;
}

struct lwDistanceResult
{
	float x;
	float y;
	float z;
};

int driverScan(lwSerialPort *Serial, lwDistanceResult *DistanceResult)
{
	// The incoming point data packet is Command 44: Distance data in cm.
	lwResponsePacket response;

	if (lwnxRecvPacket(Serial, 44, &response, 1000))
	{
		int16_t distanceCm = (response.data[5] << 8) | response.data[4];
		int16_t angleHundredths = (response.data[7] << 8) | response.data[6];

		float distance = distanceCm / 100.0f;
		float angle = angleHundredths / 100.0f;
		float faceAngle = (angle - 90) * M_PI / 180.0;

		DistanceResult->x = distance * -cos(faceAngle);
		DistanceResult->y = distance * sin(faceAngle);
		DistanceResult->z = 0;

		return 1;
	}

	return 0;
}

std::unique_ptr<sensor_msgs::msg::LaserScan> pointCloudToLaserScan(
		sensor_msgs::msg::PointCloud2 *CloudMessage, lwSf45Params *Params, float ScanTime, rclcpp::Logger Logger)
{
	// build laserscan output
	auto scanMessage = std::make_unique<sensor_msgs::msg::LaserScan>();

	scanMessage->header = CloudMessage->header;

	scanMessage->angle_min = Params->lowAngleLimit;
	scanMessage->angle_max = Params->highAngleLimit;
	scanMessage->angle_increment = 0.00349066f;
	scanMessage->time_increment = 1.0 / updateRateMap[Params->updateRate];
	scanMessage->scan_time = ScanTime;
	scanMessage->range_min = 0.2f;
	scanMessage->range_max = 50.0f;

	// determine amount of rays to create
	uint32_t ranges_size = std::ceil(
			(scanMessage->angle_max - scanMessage->angle_min) / scanMessage->angle_increment);

	scanMessage->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

	// Iterate through pointcloud
	for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*CloudMessage, "x"),
			 iter_y(*CloudMessage, "y"), iter_z(*CloudMessage, "z");
			 iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
	{
		if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
		{
			RCLCPP_DEBUG(
					Logger,
					"rejected for nan in point(%f, %f, %f)\n",
					*iter_x, *iter_y, *iter_z);
			continue;
		}

		double range = hypot(*iter_x, *iter_y);
		if (range < scanMessage->range_min)
		{
			RCLCPP_DEBUG(
					Logger,
					"rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
					range, scanMessage->range_min, *iter_x, *iter_y, *iter_z);
			continue;
		}
		if (range > scanMessage->range_max)
		{
			RCLCPP_DEBUG(
					Logger,
					"rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
					range, scanMessage->range_max, *iter_x, *iter_y, *iter_z);
			continue;
		}

		double angle = atan2(*iter_y, *iter_x);
		if (angle < scanMessage->angle_min || angle > scanMessage->angle_max)
		{
			RCLCPP_DEBUG(
					Logger,
					"rejected for angle %f not in range (%f, %f)\n",
					angle, scanMessage->angle_min, scanMessage->angle_max);
			continue;
		}

		// overwrite range at laserscan ray if new range is smaller
		int index = (angle - scanMessage->angle_min) / scanMessage->angle_increment;
		if (range < scanMessage->ranges[index])
		{
			scanMessage->ranges[index] = range;
		}
	}

	return scanMessage;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	node = rclcpp::Node::make_shared("sf45b");

	RCLCPP_INFO(node->get_logger(), "Starting SF45B node");

	auto pointCloudPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 4);
	auto laserScanPub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 4);

	lwSerialPort *serial = 0;

	int32_t baudRate = node->declare_parameter<int32_t>("baudrate", 115200);
	std::string portName = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
	std::string frameId = node->declare_parameter<std::string>("frameId", "laser");
	bool publishLaserScan = node->declare_parameter<bool>("publishLaserScan", true);

	lwSf45Params params;
	params.updateRate = node->declare_parameter<int32_t>("updateRate", 6);						 // 1 to 12
	params.cycleDelay = node->declare_parameter<int32_t>("cycleDelay", 5);						 // 5 to 2000
	params.lowAngleLimit = node->declare_parameter<int32_t>("lowAngleLimit", -45.0f);	 // -160 to -10
	params.highAngleLimit = node->declare_parameter<int32_t>("highAngleLimit", 45.0f); // 10 to 160
	validateParams(&params);

	int32_t maxPointsPerMsg = node->declare_parameter<int32_t>("maxPoints", 100); // 1 to ...
	if (maxPointsPerMsg < 1)
		maxPointsPerMsg = 1;

	if (driverStart(&serial, portName.c_str(), baudRate) != 0)
	{
		RCLCPP_ERROR(node->get_logger(), "Failed to start driver");
		return 1;
	}

	if (driverScanStart(serial, &params) != 0)
	{
		RCLCPP_ERROR(node->get_logger(), "Failed to start scan");
		return 1;
	}

	sensor_msgs::msg::PointCloud2 pointCloudMsg;
	pointCloudMsg.header.frame_id = frameId;
	pointCloudMsg.header.stamp = node->now();
	pointCloudMsg.height = 1;
	pointCloudMsg.width = maxPointsPerMsg;

	pointCloudMsg.fields.resize(3);
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

	pointCloudMsg.is_bigendian = false;
	pointCloudMsg.point_step = 12;
	pointCloudMsg.row_step = 12 * maxPointsPerMsg;
	pointCloudMsg.is_dense = true;

	pointCloudMsg.data = std::vector<uint8_t>(maxPointsPerMsg * 12);

	int currentPoint = 0;
	std::vector<lwDistanceResult> distanceResults(maxPointsPerMsg);

	while (rclcpp::ok())
	{

		while (true)
		{
			lwDistanceResult distanceResult;
			int status = driverScan(serial, &distanceResult);

			if (status == 0)
			{
				break;
			}
			else
			{
				distanceResults[currentPoint] = distanceResult;
				++currentPoint;
			}

			if (currentPoint == maxPointsPerMsg)
			{
				memcpy(&pointCloudMsg.data[0], &distanceResults[0], maxPointsPerMsg * 12);
				auto scanTime = pointCloudMsg.header.stamp.sec - node->now().seconds();

				pointCloudMsg.header.stamp = node->now();
				pointCloudPub->publish(pointCloudMsg);

				if (publishLaserScan)
				{
					laserScanPub->publish(pointCloudToLaserScan(&pointCloudMsg, &params, scanTime, node->get_logger()));
				}

				currentPoint = 0;
			}
		}

		rclcpp::spin_some(node);
	}

	rclcpp::shutdown();

	return 0;
}
