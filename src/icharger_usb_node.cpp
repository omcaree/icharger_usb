#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Trigger.h"
#include "icharger_usb/SetCurrent.h"

#include "icharger_usb.h"

#include <sstream>
#include <vector>

icharger_usb_ptr myCharger;


bool chargeCurrentCallback(icharger_usb::SetCurrent::Request &req, icharger_usb::SetCurrent::Response &res) {
	myCharger->set_selected_memory(0);
	memory mem;
	myCharger->get_memory(&mem);
	mem.ChargeCurrent = (u16)(req.current*100);
	myCharger->set_memory(&mem);
	if (myCharger->order(ORDER_WRITE_MEM,CHANNEL_1,RUNOP_CHARGE,0) == 0) {
		ROS_INFO("Charge current set to %fA", req.current);
		res.success = true;
		return true;
	} else {
		ROS_ERROR("Failed to set charge current");
		return false;
	}
}

bool dischargeCurrentCallback(icharger_usb::SetCurrent::Request &req, icharger_usb::SetCurrent::Response &res) {
	myCharger->set_selected_memory(0);
	memory mem;
	myCharger->get_memory(&mem);
	mem.DischargeCurrent = (u16)(req.current*100);
	myCharger->set_memory(&mem);
	if (myCharger->order(ORDER_WRITE_MEM,CHANNEL_1,RUNOP_CHARGE,0) == 0) {
		ROS_INFO("Discharge current set to %fA", req.current);
		res.success = true;
		return true;
	} else {
		ROS_ERROR("Failed to set discharge current");
		return false;
	}
}

bool startChargeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res, int channel) {
	Channel ch = CHANNEL_1;
	if (channel % 2 == 0) {
		ch = CHANNEL_2;
	}
	if (myCharger->order(ORDER_RUN,ch,RUNOP_CHARGE,0) == 0) {
		res.success = true;
		ROS_INFO("Charge started on channel %d", channel);
		return true;
	}
	ROS_ERROR("Failed to start charge on channel %d", channel);
	return false;
}

bool startDischargeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res, int channel) {
	Channel ch = CHANNEL_1;
	if (channel % 2 == 0) {
		ch = CHANNEL_2;
	}
	if (myCharger->order(ORDER_RUN,ch,RUNOP_DISCHARGE,0) == 0) {
		res.success = true;
		ROS_INFO("Discharge started on channel %d", channel);
		return true;
	}
	ROS_ERROR("Failed to start discharge on channel %d", channel);
	return false;
}

bool startStorageCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res, int channel) {
	Channel ch = CHANNEL_1;
	if (channel % 2 == 0) {
		ch = CHANNEL_2;
	}
	if (myCharger->order(ORDER_RUN,ch,RUNOP_STORAGE,0) == 0) {
		res.success = true;
		ROS_INFO("Storage started on channel %d", channel);
		return true;
	}
	ROS_ERROR("Failed to start storage on channel %d", channel);
	return false;
}

int main(int argc, char **argv) {
	/* Init ros */
	ros::init(argc, argv, "icharger_node");
	ros::NodeHandle n;
	ros::ServiceServer chargeCurrentService;
	ros::ServiceServer dischargeCurrentService;
	std::vector<ros::Publisher> packVoltagePublishers;
	std::vector<ros::Publisher> cellVoltagePublishers;
	std::vector<ros::Publisher> chargePublishers;
	std::vector<ros::Publisher> statusPublishers;
	std::vector<ros::ServiceServer> startChargeServices;
	std::vector<ros::ServiceServer> startDischargeServices;
	std::vector<ros::ServiceServer> startStorageServices;
	
	
	/* Init charger */
	libusb_init(NULL);
	charger_list chargers = icharger_usb_dev::all_chargers(NULL,ICHARGER_VENDOR_ID,ICHARGER_PRODUCT_4010_DUO);
	myCharger = chargers.at(0);
	myCharger->acquire();
	
	/* Print info */
	ROS_INFO("Connected to %s (%s)", myCharger->product().c_str(), myCharger->serialNumber().c_str());
	
	/* Charge current service */
	chargeCurrentService = n.advertiseService<icharger_usb::SetCurrent::Request, icharger_usb::SetCurrent::Response>("charge_current", chargeCurrentCallback);

	/* Discharge current service */
	dischargeCurrentService = n.advertiseService<icharger_usb::SetCurrent::Request, icharger_usb::SetCurrent::Response>("discharge_current", dischargeCurrentCallback);
	
	for (int charger_channel = 1; charger_channel <= 2; charger_channel++) {
		char topicName[64];
		
		/* Pack voltage publisher*/
		sprintf(topicName, "channel_%d/pack_voltage",charger_channel);
		packVoltagePublishers.push_back(n.advertise<std_msgs::Float32>(topicName, 1));
		
		/* Cell voltages publisher*/
		sprintf(topicName, "channel_%d/cell_voltages",charger_channel);
		cellVoltagePublishers.push_back(n.advertise<std_msgs::Float32MultiArray>(topicName, 1));
		
		/* Charge publisher*/
		sprintf(topicName, "channel_%d/charge",charger_channel);
		chargePublishers.push_back(n.advertise<std_msgs::Int32>(topicName, 1));
		
		/* Status publisher */
		sprintf(topicName, "channel_%d/status",charger_channel);
		statusPublishers.push_back(n.advertise<std_msgs::Int32>(topicName, 1));
		
		/* Charge service */
		sprintf(topicName, "channel_%d/start_charge",charger_channel);
		startChargeServices.push_back(n.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(topicName,boost::bind(startChargeCallback, _1, _2, charger_channel)));
		
		/* Discharge service */
		sprintf(topicName, "channel_%d/start_discharge",charger_channel);
		startDischargeServices.push_back(n.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(topicName,boost::bind(startDischargeCallback, _1, _2, charger_channel)));
		
		/* Storage service */
		sprintf(topicName, "channel_%d/start_storage",charger_channel);
		startStorageServices.push_back(n.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(topicName,boost::bind(startStorageCallback, _1, _2, charger_channel)));
	}
	ros::Rate r(1);
	while (ros::ok()) {
		for (int ch = 1; ch <= 2; ch++) {
			/* Read channel status */
			channel_status chStatus;
			memset(&chStatus, 0, sizeof(chStatus));
			int retval = myCharger->get_channel_status(ch-1,&chStatus);
			if (retval != 0) {
				ROS_ERROR("Channel %d status failed with code 0x%X", ch, retval);
			}
			
			/* Publish pack voltage */
			std_msgs::Float32 pack_voltage;
			pack_voltage.data = ((float)chStatus.output_voltage.value)/1000.0f;
			packVoltagePublishers.at(ch-1).publish(pack_voltage);
			
			/* Publish cell voltages */
			std_msgs::Float32MultiArray cell_voltages;
			cell_voltages.data.clear();
			for (int j=0; j<10; j++) {
				cell_voltages.data.push_back(((float)chStatus.cell_voltage[j])/1000.0f);
			}
			cellVoltagePublishers.at(ch-1).publish(cell_voltages);
			
			/* Publish charge */
			std_msgs::Int32 charge;
			charge.data = chStatus.output_capacity.svalue;
			chargePublishers.at(ch-1).publish(charge);
			
			/* Publish status */
			std_msgs::Int32 status;
			status.data = chStatus.run_status;
			statusPublishers.at(ch-1).publish(status);
		}
		r.sleep();
		ros::spinOnce();
	}
	libusb_exit(NULL);
  return 0;
}