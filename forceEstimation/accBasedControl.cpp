/*
 * A ROS NODE ON WHICH THE ACCELERATIONS FROM TWO MTw SENSORS
 * ARE USED AS INPUT TO CALCULATE A DESIRED OUTPUT ON THE MOTORS IN A JOINT
 * OF THE EXOSUIT DEVELOPED ON REHAB LAB @ EESC-USP.
 * 
 * Leonardo Felipe Lima Santos dos Santos, 2019
 * e-mail leonardo.felipe.santos@usp.br
 * github/bitbucket qleonardolp
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define QUEUE_SIZE 100

// Global Variables

std_msgs::Float32 desTorque;            // Desired Torque Pub [Nm] 

float inertiaMomentExo = 123*10^(-4);   // [Kg.m^2]
float Kp = 12;                          // [???], maybe Kg.m^2
float Ki = 45;                          // [???], maybe Kg.m^2/s
float mtwLimb_dist = 0.215;             // [m]
float mtwExo_dist = 0.145;              // [m]

//


void fullCallback(const geometry_msgs::Vector3Stamped::ConstPtr& limbAcc, 
                  const geometry_msgs::Vector3Stamped::ConstPtr& exoAcc, 
                  const geometry_msgs::Vector3Stamped::ConstPtr& limbVel, 
                  const geometry_msgs::Vector3Stamped::ConstPtr& exoVel) 
{
    
    desTorque.data = inertiaMomentExo*(1/mtwLimb_dist)*limbAcc->vector.y + 
    Kp*(1/mtwLimb_dist)*(limbAcc->vector.y - exoAcc->vector.y) + Ki*(limbVel->vector.z - exoVel->vector.z);
    
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "acc_based_control"); 
    
    // LEMBRE-SE DE PASSAR OS IDs DOS SENSORES (MTw)
    // EXEMPLO: $ rosrun xsens_mtw_driver acc_based_control 322 324

    std::string const mtwIDlimb = argv[1];
    std::string const mtwIDexo = argv[2];


    std::string const mtwlimb = mtwIDlimb.substr(mtwIDlimb.size()-1, mtwIDlimb.size());
    std::string const mtwexo = mtwIDexo.substr(mtwIDexo.size()-1, mtwIDexo.size());

    ros::NodeHandle nh;
    
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> mtwLimbAcc(nh, "free_acc_0034232" + mtwlimb, QUEUE_SIZE);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> mtwExoAcc(nh, "free_acc_0034232" + mtwexo, QUEUE_SIZE);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> mtwLimbVel(nh, "gyroscope_0034232" + mtwlimb, QUEUE_SIZE);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> mtwExoVel(nh, "gyroscope_0034232" + mtwexo, QUEUE_SIZE);

    ros::Publisher desTorquePub = nh.advertise<std_msgs::Float32>("desired_Torque", QUEUE_SIZE);

    ROS_INFO_STREAM("Subcribed on " << mtwLimbAcc.getTopic() << ", " << mtwLimbVel.getTopic());
    ROS_INFO_STREAM("Subcribed on " << mtwExoAcc.getTopic() << ", " << mtwExoVel.getTopic());
    
    typedef message_filters::sync_policies::ApproximateTime
    <geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> mySyncPolicy;

    message_filters::Synchronizer<mySyncPolicy> sync(mySyncPolicy(4), mtwLimbAcc, mtwExoAcc, mtwLimbVel, mtwExoVel);
    sync.registerCallback(fullCallback);

    ros::Rate pub_rate(120);        // 120 Hz

    while (ros::ok())
    {
        desTorquePub.publish(desTorque);
        ros::spinOnce();
        pub_rate.sleep();
    }

    ros::spin();

    return 0;
}