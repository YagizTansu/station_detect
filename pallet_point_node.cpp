#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_line_extraction/line_extraction_ros.h"
#include <visualization_msgs/MarkerArray.h>
#include <vector>

visualization_msgs::Marker marker;
visualization_msgs::Marker marker2;
visualization_msgs::MarkerArray markerArray;
double pallet_middle_size = 0.13;
double pallet_middle_size_error = 0.04;

double pallet_side_size = 0.13;
double pallet_side_size_error = 0.03;

double side_to_side = 0.70;
double side_to_side_error = 0.5;

void createMarker(double x, double y)
{
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
}

void createMarker2(double x, double y)
{
    marker2.header.frame_id = "laser";
    marker2.header.stamp = ros::Time();
    marker2.ns = "my_namespace";
    marker2.id = 1;
    marker2.type = visualization_msgs::Marker::SPHERE;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = x;
    marker2.pose.position.y = y;
    marker2.pose.position.z = 0;
    marker2.pose.orientation.x = 0.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 1.0;
    marker2.scale.x = 0.05;
    marker2.scale.y = 0.05;
    marker2.scale.z = 0.05;
    marker2.color.a = 1.0; // Don't forget to set the alpha!
    marker2.color.r = 0.0;
    marker2.color.g = 0.0;
    marker2.color.b = 1.0;
}

double middleOfLine_X(laser_line_extraction::LineSegmentList lines, int val)
{
    double middle = (lines.line_segments[val].start[0] + lines.line_segments[val].end[0]) / 2;
    return middle;
}

double middleOfLine_Y(laser_line_extraction::LineSegmentList lines, int val)
{
    double middle = (lines.line_segments[val].start[1] + lines.line_segments[val].end[1]) / 2;
    return middle;
}

void scanCallback(const laser_line_extraction::LineSegmentList lines)
{
    std::vector<int> points;
    double sumX = 0;
    // ROS_INFO("size= %ld",  lines.line_segments.size());

    for (int i = 0; i < lines.line_segments.size(); i++)
    {
        ROS_INFO("pallet_angle= %f", lines.line_segments[i].radius);
        // ROS_INFO("************************************");
        double line_size = sqrt(pow((lines.line_segments[i].end[1] - lines.line_segments[i].start[1]), 2) + pow((lines.line_segments[i].end[0] - lines.line_segments[i].start[0]), 2));

        if (lines.line_segments[i].radius > 1.0 && lines.line_segments[i].radius < 3.0)
        {
            ROS_INFO("herere");

            points.push_back(i);
        }
    }
    ROS_INFO("************************************");

    ROS_INFO("size= %ld", points.size());

    if (points.size() > 1)
    {
        for (int i = 0; i < points.size() - 1; i++)
        {
            // double middle_x = middleOfLine_X(lines, points[i]);
            double before_middle_x = middleOfLine_X(lines, points[i]);
            double after_middle_x = middleOfLine_X(lines, points[i + 1]);

            // double middle_y = middleOfLine_Y(lines, points[i]);
            double before_middle_y = middleOfLine_Y(lines, points[i]);
            double after_middle_y = middleOfLine_Y(lines, points[i + 1]);

            double left_to_right_dist = sqrt(pow((after_middle_y - before_middle_y), 2) + pow((after_middle_x - before_middle_x), 2));
            double right_to_left_dist = sqrt(pow((before_middle_y - after_middle_y), 2) + pow((before_middle_x - after_middle_x), 2));

            // double middle_line_size = sqrt(pow((lines.line_segments[points[i]].end[1] - lines.line_segments[points[i]].start[1]), 2) + pow((lines.line_segments[points[i]].end[0] - lines.line_segments[points[i]].start[0]), 2));
            double right_side_size = sqrt(pow((lines.line_segments[points[i]].end[1] - lines.line_segments[points[i]].start[1]), 2) + pow((lines.line_segments[points[i]].end[0] - lines.line_segments[points[i]].start[0]), 2));
            double left_side_size = sqrt(pow((lines.line_segments[points[i + 1]].end[1] - lines.line_segments[points[i + 1]].start[1]), 2) + pow((lines.line_segments[points[i + 1]].end[0] - lines.line_segments[points[i + 1]].start[0]), 2));

            //("here= %f", abs(abs(after_middle_x) - abs(middle_x)));
            ROS_INFO("************************************");
            ROS_INFO("size= %ld", points.size());
            ROS_INFO("left_size= %f", left_side_size);
            // ROS_INFO("middle_size= %f", middle_line_size);
            ROS_INFO("right_size= %f", right_side_size);
            ROS_INFO("dis to left= %f", right_to_left_dist);
            ROS_INFO("dist to_right= %f", left_to_right_dist);
            ROS_INFO("************************************");

            if ((left_side_size < (pallet_side_size + pallet_side_size_error)) && (left_side_size > (pallet_side_size - pallet_side_size_error)))
            {
                if ((right_side_size < (pallet_side_size + pallet_side_size_error)) && (right_side_size > (pallet_side_size - pallet_side_size_error)))
                {

                    if ((right_to_left_dist > side_to_side - side_to_side_error) && (right_to_left_dist < side_to_side + side_to_side_error) && (left_to_right_dist < side_to_side + side_to_side_error) && (left_to_right_dist > side_to_side - side_to_side - side_to_side_error))
                    {
                        createMarker((lines.line_segments[points[i]].start[0] + lines.line_segments[points[i]].end[0]) / 2, (lines.line_segments[points[i]].start[1] + lines.line_segments[points[i]].end[1]) / 2);
                        createMarker2((lines.line_segments[points[i + 1]].start[0] + lines.line_segments[points[i + 1]].end[0]) / 2, (lines.line_segments[points[i + 1]].start[1] + lines.line_segments[points[i + 1]].end[1]) / 2);

                        markerArray.markers.push_back(marker);
                        markerArray.markers.push_back(marker2);
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pallet_points_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/line_segments", 1000, scanCallback);
    ros::Publisher left_point = nh.advertise<visualization_msgs::Marker>("left_point", 1000);
    ros::Publisher right_point = nh.advertise<visualization_msgs::Marker>("right_point", 1000);
    ros::Publisher marker_array = nh.advertise<visualization_msgs::MarkerArray>("station_points", 1000);

    ros::Rate r(5.0);
    ros::spinOnce();
    while (ros::ok())
    {
        left_point.publish(marker);
        right_point.publish(marker2);
        marker_array.publish(markerArray);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}