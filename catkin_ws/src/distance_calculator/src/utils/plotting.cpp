#include "utils/plotting.h"


namespace bipedlab
{
namespace plotting 
{

void addMarkerWithPose(visualization_msgs::Marker &marker,
              const uint32_t shape, const std::string t_namespace,
              const double r, const double g, const double b, const double a,
              const pose_6dof_t &pose,
              const int id_count, 
              const double lasting_time,
              const std::string dis_text,
              const double marker_size_x, 
              const double marker_size_y, 
              const double marker_size_z,
              const std::string frame_id)
{

	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();;
	marker.ns = t_namespace;
	marker.id = id_count;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;


    // position
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = pose.z;

    // orientation
    tf2::Quaternion q_tf;
    q_tf.setRPY(pose.rho, pose.phi, pose.theta);
    // std::cout << "x: " << q_tf.getX() << ", y: " << q_tf.getY() << ", z: " << q_tf.getZ() << ", w: " << q_tf.getW() << std::endl;
    // pose.print();
    q_tf.normalize();
    // std::cout << "x: " << q_tf.getX() << ", y: " << q_tf.getY() << ", z: " << q_tf.getZ() << ", w: " << q_tf.getW() << std::endl;

    geometry_msgs::Quaternion q_msg = tf2::toMsg(q_tf);
    //tf2::convert(q_msg , q_tf);
    // std::cout << "x: " << q_msg.x << ", y: " << q_msg.y << ", z: " << q_msg.z << ", w: " << q_msg.w << std::endl;

    marker.pose.orientation.x = q_msg.x;
    marker.pose.orientation.y = q_msg.y;
    marker.pose.orientation.z = q_msg.z;
    marker.pose.orientation.w = q_msg.w;

    // std::cerr << "unsupported point type: " << typeid(T).name();

    // other info
	marker.text = dis_text;
	marker.lifetime = ros::Duration(lasting_time); // should disappear along with updateing rate

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker_size_x;
	marker.scale.y = marker_size_y;
	marker.scale.z = marker_size_z;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;
}

void addMarker(visualization_msgs::Marker &marker,
              const uint32_t shape, const std::string t_namespace,
              const double r, const double g, const double b,const double a,
              const double x, const double y, const double z, 
              const double wx, const double wy, const double wz, const double w,
              const int id_count, const double marker_size, 
              const std::string dis_text,
              const std::string frame_id){
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();;
	marker.ns = t_namespace;
	marker.id = id_count;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.pose.orientation.x = wx;
	marker.pose.orientation.y = wy;
	marker.pose.orientation.z = wz;
	marker.pose.orientation.w = w;
	marker.text = dis_text;
	marker.lifetime = ros::Duration(); // should disappear along with updateing rate

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker_size;
	marker.scale.y = marker_size;
	marker.scale.z = marker_size;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;
}

void addMarkerWithTwoPoints(visualization_msgs::Marker &marker,
              const uint32_t shape, const std::string t_namespace,
              const double r, const double g, const double b,const double a,
              const double x1, const double y1, const double z1,
              const double x2, const double y2, const double z2,
              const int id_count,
              const double lasting_time,
              const std::string dis_text,
              const double marker_size_x,
              const double marker_size_y,
              const double marker_size_z,
              const std::string frame_id)
{
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();;
	marker.ns = t_namespace;
	marker.id = id_count;
	marker.type = shape;
    geometry_msgs::Point point1;
    point1.x = x1;
    point1.y = y1;
    point1.z = z1;
    marker.points.push_back(point1);

    geometry_msgs::Point point2;
    point2.x = x2;
    point2.y = y2;
    point2.z = z2;
    marker.points.push_back(point2);

	marker.text = dis_text;
	marker.lifetime = ros::Duration(lasting_time); // should disappear along with updateing rate


	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker_size_x;
	marker.scale.y = marker_size_y;
	marker.scale.z = marker_size_z;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;
}

bool moveRobot(ros::Publisher marker_pub,
        geometry_msgs::Point next_pose, geometry_msgs::Point goal) {
    static visualization_msgs::Marker rob;
    rob.type = visualization_msgs::Marker::CUBE;


    rob.header.frame_id = "map";
    rob.header.stamp = ros::Time::now();
    rob.ns = "rob";
    rob.id = 0;
    rob.action = visualization_msgs::Marker::ADD;
    rob.lifetime = ros::Duration();

    rob.scale.x = 0.5;
    rob.scale.y = 1;
    rob.scale.z = 0.25;
    rob.pose.orientation.w = 1;
    rob.pose.orientation.x = rob.pose.orientation.y = rob.pose.orientation.z = 0;
    rob.color.r = 1.0f;
    rob.color.g = 0.5f;
    rob.color.b = 0.5f;
    rob.color.a = 1.0;

    //calculate m to change the orientation of the robot
    float m = (next_pose.y - rob.pose.position.y) / (next_pose.x - rob.pose.position.x);

    rob.pose.orientation.z = atan(m) + M_PI / 2;
    rob.pose.position = next_pose;

    marker_pub.publish(rob);

    if ((rob.pose.position.x == goal.x) && (rob.pose.position.y == goal.y)) {
        marker_pub.publish(rob);
        return true;
    }

    return false;
}

// void drawFeasiblePath(
//         const PolynomialTraj_t trajs,
//         const double t_0, const double t_m,
//         const double delta, const int order,
//         ros::Publisher markerArray_pub){
//     visualization_msgs::MarkerArray markerarray_trajs;
//     visualization_msgs::Marker traj;
//     traj.type = visualization_msgs::Marker::ARROW;
//     traj.header.frame_id = "map";
//     traj.header.stamp = ros::Time::now();
//     traj.lifetime = ros::Duration();
//     traj.action = visualization_msgs::Marker::ADD;
//     traj.color.a = 1.0f;
//     traj.color.r = 1.0f;
//     traj.color.g = 0.0f;
//     traj.color.b = 1.0f;
//     traj.scale.x = 1;
//     traj.scale.y = 0.05;
//     traj.scale.z = 0.05;
//     traj.pose.orientation.x = 0.0;
//     traj.pose.orientation.y = 0.0;
//     traj.pose.orientation.w = 1;
//     traj.ns = "feasible path";
// 
//     int k = 0;
//     for (double t=t_0; t<t_m; t+=delta){
//         double x = 0;
//         double y = 0;
//         double theta = 0;
// 
//         for (int i=0; i<order+1; ++i){
//             x = x + trajs.px[i]*std::pow(t, i);
//             y = y + trajs.py[i]*std::pow(t, i);
//             theta = theta + trajs.ptheta[i]*std::pow(t, i);
//         }
//         // if (t==t_0 || t==t_m){
//         //     cout << "(x, y, theta) = " << "(" << x << ", " << y << ", " << theta << ")" << endl;
//         // }
// 
//         // traj.ns = "feasible path" + to_string(k);
//         traj.id = k;
//         traj.pose.orientation.z = theta;
//         traj.pose.position.x = x;
//         traj.pose.position.y = y;
//         traj.pose.position.z = 0.5;
//         markerarray_trajs.markers.push_back(traj);
//         k++;
//     }
//     markerArray_pub.publish(markerarray_trajs);
// }

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub) {
    static visualization_msgs::Marker edge;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "finalPath";
    edge.id = 4;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.04;

    edge.color.g = edge.color.r = 1;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}


} // bipedlab
} // plotting


