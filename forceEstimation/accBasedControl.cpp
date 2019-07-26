/*
 * PROTOTYPE OF A ROS NODE ON WHICH THE ACCELERATIONS FROM TWO MTW SENSORS
 * ARE USED AS INPUT TO CALCULATE A DESIRED OUTPUT ON THE MOTORS IN A JOINT
 * OF THE EXOSUIT DEVELOPED ON REHAB LAB @ EESC-USP.
 * 
 * Leonardo Felipe Lima Santos dos Santos, 2019
 * e-mail leonardo.felipe.santos@usp.br
 * github/bitbucket qleonardolp
 */

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include <math.h>

// This node is a subscriber of two "free_acc_mtwID" topics
// The msg type is Vector3Stamped

geometry_msgs::Vector3 deltaFreeAcc;        //vetor para armazenar a diferença entre os dois sensores
geometry_msgs::Vector3 freeAccLimb;         // Limb acc, [m/s^2]
geometry_msgs::Vector3 freeAccExo;          // Exo acc, [m/s^2]
ros::Rate pub_loop = 100;
float accNorm;


void freeAccCBLimb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    freeAccLimb = msg->vector;
}

void freeAccCBExo(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    freeAccExo = msg->vector;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "acc_based_control");
    ros::NodeHandle node;

    std::string const mtwIDlimb = argv[1];
    std::string const mtwIDexo = argv[2];

    ros::Subscriber sublimb = node.subscribe("free_acc_" + mtwIDlimb, 1000, freeAccCBLimb);
    ros::Subscriber subexo = node.subscribe("free_acc_" + mtwIDexo, 1000, freeAccCBExo);

    deltaFreeAcc.x = freeAccLimb.x - freeAccExo.x;
    deltaFreeAcc.y = freeAccLimb.y - freeAccExo.y;
    deltaFreeAcc.z = freeAccLimb.z - freeAccExo.z;
    
    accNorm = sqrt((deltaFreeAcc.x)*(deltaFreeAcc.x) + (deltaFreeAcc.y)*(deltaFreeAcc.y) + (deltaFreeAcc.z)*(deltaFreeAcc.z));

    ros::Publisher torque = node.advertise<std_msgs::Float32>("desired_torque", 1000);

    std_msgs::Float32 dtorque;
    dtorque.data = 1.0*accNorm;

    torque.publish(dtorque);
    
    /*
    while (ros::ok())
    {
        torque.publish(0.3*accNorm);
        
        ros::spinOnce();
        pub_loop.sleep();
    }
    */
    ros::spin();
    return 0;
}