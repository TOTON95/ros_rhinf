#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>


double ce = 0;


struct v_object                                                         //Structure that describes the properties of the object
{
        double _posX,_posY,_posZ;                                       //Position of the drone
        double _errorX,_errorY,_errorZ;                                 //Error of the position of the drone
        double _orX,_orY,_orZ,_orW;                                     //Orientation of the drone
        double _roll,_pitch,_yaw,_yawRAD;                               //Roll,Pitch,Yaw (degrees), Yaw (radians)
        double _cmdX,_cmdY,_cmdZ,_cmdYAW;                               //Command values
        double rot_cmd_x,rot_cmd_y;                                     //Position in the rotated matrix
        double _velX,_velY,_velZ,_velYAW;                               //Velocities
        double abs_x,abs_y;                                             //Absolute position in X and Y
        double angle_res,angle_arc;                                     //Angle resultant, angle of the arc
}drone;

void getDronePos(const geometry_msgs::TransformStamped::ConstPtr& pos)  //Function to obtain the position from the vicon system
{
        drone._posX = pos->transform.translation.x;                     //Position in X
        drone._posY = pos->transform.translation.y;                     //Position in Y
        drone._posZ = pos->transform.translation.z;                     //Position in Z
        drone._orX = pos->transform.rotation.x;                         //Rotation in X
        drone._orY = pos->transform.rotation.y;                         //Rotation in Y
        drone._orZ = pos->transform.rotation.z;                         //Rotation in Z
        drone._orW = pos->transform.rotation.w;                         //Rotation in W

        tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(drone._roll,drone._pitch,drone._yawRAD);               //Get the Roll, Pitch, Yaw (Radians)
        drone._yaw = drone._yawRAD*(180/PI);                            //Convert the Yaw (Radians) into Yaw (Degrees)
        heading = drone._yaw;                                           //Set the heading of the drone


        drone.abs_x = drone._posX;                                      //Set the absolute position of the drone in X
        drone.abs_y = drone._posY;                                      //Set the absolute position of the drone in Y
}


void getEffort(const std_msgs::Float64::ConstPtr& msg)
{
        ce = msg->data;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"rhinf_converter"); //Initiates the ROS node
	
}
