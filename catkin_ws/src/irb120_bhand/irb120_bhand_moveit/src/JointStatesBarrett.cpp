#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <stdlib.h>
#include <stdint.h>

ros::Publisher BarrettHand_pub;
ros::Publisher BarrettHand_pub2;
float OldBarrettPosition[4];

void JointStatesCallback(const sensor_msgs::JointState & msg)
{
  	float BarrettPosition[4];
  	sensor_msgs::JointState BarrettCommands, BarrettCommandsMoveIt;
  	ros::Rate loop_rate(10);

	BarrettPosition[0] = msg.position[9];
	BarrettPosition[1] = msg.position[12];
	BarrettPosition[2] = msg.position[7];
	BarrettPosition[3] = msg.position[10];

	for (int i = 0; i < 4 ; i ++) {
		if (BarrettPosition[i] < 0) {
			BarrettPosition[i] = -BarrettPosition[i];
		} 	
	
	}

	BarrettCommands.header.seq = 0;
	BarrettCommands.header.frame_id = "";     
	BarrettCommands.name.push_back("bh_j11_joint");
	BarrettCommands.name.push_back("bh_j32_joint");
	BarrettCommands.name.push_back("bh_j12_joint");
	BarrettCommands.name.push_back("bh_j22_joint");
	BarrettCommands.position.push_back(BarrettPosition[0]);
	BarrettCommands.position.push_back(BarrettPosition[1]);
	BarrettCommands.position.push_back(BarrettPosition[2]);
	BarrettCommands.position.push_back(BarrettPosition[3]);
	BarrettCommands.velocity.push_back(0.1);
	BarrettCommands.velocity.push_back(0.1);
	BarrettCommands.velocity.push_back(0.1);
	BarrettCommands.velocity.push_back(0.1);
	BarrettCommands.effort.push_back(0.0);
	BarrettCommands.effort.push_back(0.0);
	BarrettCommands.effort.push_back(0.0);
	BarrettCommands.effort.push_back(0.0);

	BarrettCommandsMoveIt.header.seq = 0;
	BarrettCommandsMoveIt.header.frame_id = "";     
	BarrettCommandsMoveIt.name.push_back("bh_j11_joint");
	BarrettCommandsMoveIt.name.push_back("bh_j32_joint");
	BarrettCommandsMoveIt.name.push_back("bh_j12_joint");
	BarrettCommandsMoveIt.name.push_back("bh_j22_joint");
	BarrettCommandsMoveIt.position.push_back(-BarrettPosition[0]);
	BarrettCommandsMoveIt.position.push_back(-BarrettPosition[1]);
	BarrettCommandsMoveIt.position.push_back(-BarrettPosition[2]);
	BarrettCommandsMoveIt.position.push_back(-BarrettPosition[3]);
	BarrettCommandsMoveIt.velocity.push_back(0.1);
	BarrettCommandsMoveIt.velocity.push_back(0.1);
	BarrettCommandsMoveIt.velocity.push_back(0.1);
	BarrettCommandsMoveIt.velocity.push_back(0.1);
	BarrettCommandsMoveIt.effort.push_back(0.0);
	BarrettCommandsMoveIt.effort.push_back(0.0);
	BarrettCommandsMoveIt.effort.push_back(0.0);
	BarrettCommandsMoveIt.effort.push_back(0.0);

	
	if ((OldBarrettPosition[0] != BarrettPosition[0]) | (OldBarrettPosition[1] != BarrettPosition[1]) | (OldBarrettPosition[2] != BarrettPosition[2]) | 		(OldBarrettPosition[3] != BarrettPosition[3]) ) {

		BarrettHand_pub.publish(BarrettCommands);
		BarrettHand_pub2.publish(BarrettCommandsMoveIt);

	}
		

	for (int i = 0; i < 4 ; i ++) {
		
		OldBarrettPosition[i] = BarrettPosition[i];

	}

	ros::spin();

}

int main(int argc, char **argv)
{
	  ros::init(argc, argv, "JointStatestoBarrett");
	  ros::NodeHandle n;
	  ros::Subscriber sub = n.subscribe("joint_states_barrett", 1000, JointStatesCallback);
	  BarrettHand_pub = n.advertise<sensor_msgs::JointState>("/bhand_node/command", 1000);
	  BarrettHand_pub2 = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);
	  ros::spin();

  return 0;
}


