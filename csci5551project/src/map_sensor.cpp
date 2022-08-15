#include "ros/ros.h" //For general ros use
#include "sensor_msgs/LaserScan.h" //Getting lidar readings
#include "tf2_msgs/TFMessage.h" //Getting robot position and orientation
#include <visualization_msgs/Marker.h> //Using rviz

#include <vector>
#include <math.h>
#include <iostream>

#include "extra_tools.h" //Header file for quadnode

std::vector<float> ranges;
float quatX = 0;
float quatY = 0;
float quatZ = 0;
float quatW = 0;
float scanMax = 0;
float curX = 0;
float curY = 0;
float curYaw = 0;

std::vector<sensor_msgs::LaserScan> scanMsg;
std::vector<tf2_msgs::TFMessage> tfMsg;

void collectLaser(const sensor_msgs::LaserScan::ConstPtr& msg) {
	scanMsg.push_back(*msg);
}

void getTf(const tf2_msgs::TFMessage::ConstPtr& msg) {
	tfMsg.push_back(*msg);
}

int main(int argc, char **argv) {
	quadNode headNode(0,0);
	float width = pow(1.0/2, 6);

	ros::init(argc, argv, "map_sensor");
        ros::NodeHandle n;
	ros::Subscriber scanSub = n.subscribe("scan", 10, collectLaser);
	ros::Subscriber tfSub = n.subscribe("tf", 10, getTf);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	//Set up marker for points
	visualization_msgs::Marker points;
	points.header.frame_id = "my_frame";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	//Point scale
	points.scale.x = 0.05;
	points.scale.y = 0.05;
	//Point color
	points.color.g = 1.0f;
	points.color.a = 1.0;

	//Set up robot center
	visualization_msgs::Marker marker;
	marker.header.frame_id = "my_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "burger_center";
	marker.id = 1;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();

	//Set up robot directional arrow
        visualization_msgs::Marker burgerArrow;
        burgerArrow.header.frame_id = "my_frame";
        burgerArrow.header.stamp = ros::Time::now();
        burgerArrow.ns = "burger_arrow";
        burgerArrow.id = 2;
        burgerArrow.type = visualization_msgs::Marker::ARROW;
        burgerArrow.action = visualization_msgs::Marker::ADD;
        burgerArrow.pose.position.x = 0;
        burgerArrow.pose.position.y = 0;
        burgerArrow.pose.position.z = 0;
        burgerArrow.pose.orientation.x = 0.0;
        burgerArrow.pose.orientation.y = 0.0;
        burgerArrow.pose.orientation.z = 0.0;
        burgerArrow.pose.orientation.w = 1.0;
        burgerArrow.scale.x = 0.5;
        burgerArrow.scale.y = 0.05;
        burgerArrow.scale.z = 0.05;
        burgerArrow.color.r = 0.0f;
        burgerArrow.color.g = 0.0f;
        burgerArrow.color.b = 1.0f;
        burgerArrow.color.a = 1.0;
        burgerArrow.lifetime = ros::Duration();

	ros::Rate r(10);
	// Create the vertices for the points

	while(ros::ok()) {
                int minI = 0;
                int minJ = 0;
                float minDif = 0;
                float curDif = 0;

                if(!scanMsg.empty() && !tfMsg.empty()) {
                        float scanSec = scanMsg[0].header.stamp.sec;
                        float scanNSec = scanMsg[0].header.stamp.nsec;
                        float tfSec = tfMsg[0].transforms[0].header.stamp.sec;
                        float tfNSec = tfMsg[0].transforms[0].header.stamp.nsec;
                        minDif = abs(scanSec + scanNSec*pow(10, -9) - (tfSec + tfNSec*pow(10, -9)));

                        for(int i = 0; i < scanMsg.size(); i++) {
                                for(int j = 0; j < tfMsg.size(); j++) {
                                        scanSec = scanMsg[i].header.stamp.sec;
                                        scanNSec = scanMsg[i].header.stamp.nsec;
                                        tfSec = tfMsg[j].transforms[0].header.stamp.sec;
                                        tfNSec = tfMsg[j].transforms[0].header.stamp.nsec;
                                        curDif = abs(scanSec + scanNSec*pow(10, -9) - (tfSec + tfNSec*pow(10, -9)));
                                        if(curDif < minDif) {
                                                minDif = curDif;
                                                minI = i;
                                                minJ = j;
                                        }
                                }
                        }

			ranges = scanMsg[minI].ranges;
		        scanMax = scanMsg[minI].range_max;
			curX = tfMsg[minJ].transforms[0].transform.translation.x;
		        curY = tfMsg[minJ].transforms[0].transform.translation.y;
		        quatX = tfMsg[minJ].transforms[0].transform.rotation.x;
		        quatY = tfMsg[minJ].transforms[0].transform.rotation.y;
		        quatZ = tfMsg[minJ].transforms[0].transform.rotation.z;
		        quatW = tfMsg[minJ].transforms[0].transform.rotation.w;
		        float siny_cosp = 2 * (quatW * quatZ + quatX * quatY);
		        float cosy_cosp = 1 - 2 * (quatY * quatY + quatZ * quatZ);
		        curYaw = atan2(siny_cosp, cosy_cosp);

			for (int i = 0; i < ranges.size(); i++) {
                        	if(ranges[i] > scanMax) {
                                	continue;
                        	}
				float rawX = ranges[i]*cos(i*M_PI/180.0 + curYaw) + curX;
				float rawY = ranges[i]*sin(i*M_PI/180.0 + curYaw) + curY;
				float roundX = ((int)(rawX/width))*width;
				float roundY = ((int)(rawY/width))*width;
				if(headNode.addNode(roundX, roundY)) {
                        		geometry_msgs::Point p;
                        		p.x = roundX;
                        		p.y = roundY;
                        		p.z = 0;
                        		points.points.push_back(p);
				}
                	}
                	marker.pose.position.x = curX;
                	marker.pose.position.y = curY;
                	burgerArrow.pose.position.x = curX;
                	burgerArrow.pose.position.y = curY;

	                burgerArrow.pose.orientation.x = quatX;
	                burgerArrow.pose.orientation.y = quatY;
	                burgerArrow.pose.orientation.z = quatZ;
	                burgerArrow.pose.orientation.w = quatW;

	                marker_pub.publish(points);
	                marker_pub.publish(marker);
	                marker_pub.publish(burgerArrow);

                        scanMsg.clear();
                        tfMsg.clear();
                }

                ros::spinOnce();
                r.sleep();
        }

	return 0;
}
