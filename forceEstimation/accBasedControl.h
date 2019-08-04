#ifndef ACC_BASED_CONTROL_H
#define ACC_BASED_CONTROL_H

/*
 * Class to build a SubcriberPublisher Node,
 * as seen on youtu.be/lR3cK9ZoAF8
 */

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>


class SubTwoMTwPubTorque
{
    public:
        SubTwoMTwPubTorque() {}
        SubTwoMTwPubTorque(std::string const mtwlimb, std::string const mtwexo, int queueSize)
        {
            mtwLimbObject.resize(2);
            mtwExoObject.resize(2);     // Subscribers vectors for Limb and Exo sized in 2, each catching two data types: free_acc and gyroscope_
            
            desTorqueObject = nh.advertise<std_msgs::Float32>("desired_Torque", queueSize);

            mtwLimbObject[0] = nh.subscribe<geometry_msgs::Vector3Stamped>("free_acc_0034232" + mtwlimb, queueSize, &SubTwoMTwPubTorque::mtwLimbAccCB, this);
            mtwExoObject[0] = nh.subscribe<geometry_msgs::Vector3Stamped>("free_acc_0034232" + mtwexo, queueSize, &SubTwoMTwPubTorque::mtwExoAccCB, this);

            mtwLimbObject[1] = nh.subscribe<geometry_msgs::Vector3Stamped>("gyroscope_0034232" + mtwlimb, queueSize, &SubTwoMTwPubTorque::mtwLimbVelCB, this);
            mtwExoObject[1] = nh.subscribe<geometry_msgs::Vector3Stamped>("gyroscope_0034232" + mtwexo, queueSize, &SubTwoMTwPubTorque::mtwExoVelCB, this);
            
            ROS_INFO_STREAM("Subcribed on " << mtwLimbObject[0].getTopic() << ", " << mtwLimbObject[1].getTopic());
            ROS_INFO_STREAM("Subcribed on " << mtwExoObject[0].getTopic() << ", " << mtwExoObject[1].getTopic());
            ROS_INFO_STREAM("Publishing on " << desTorqueObject.getTopic());
        }

        void mtwLimbAccCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
        {
            AccLimb = msg->vector;
        }

        void mtwExoAccCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
        {
            AccExo = msg->vector;
        }

        void mtwLimbVelCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
        {
            VelLimb = msg->vector;
        }

        void mtwExoVelCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
        {
            VelExo = msg->vector;
            desTorque.data = inertiaMomentExo*(1/mtw_dist)*AccLimb.y + Kp*(1/mtw_dist)*(AccLimb.y - AccExo.y) + Ki*(VelLimb.z - VelExo.z);
            
            desTorqueObject.publish(desTorque);
        }

    private:

        geometry_msgs::Vector3 AccLimb;         // Limb acc, [m/s^2]
        geometry_msgs::Vector3 AccExo;          // Exo acc, [m/s^2]

        geometry_msgs::Vector3 VelLimb;         // Limb Velocity, [rad/s]
        geometry_msgs::Vector3 VelExo;          // Exo Velocity, [rad/s]

        std_msgs::Float32 desTorque;            // Desired Torque Pub [Nm] 

        float inertiaMomentExo = 123*10^(-4);   // [Kg.m^2]
        float Kp = 12;                          // [???], maybe Kg.m^2
        float Ki = 45;                          // [???], maybe Kg.m^2/s
        float mtw_dist = 0.215;                  // [m]


        /*
         * -- Conversion from the accNorm [m/s^2] to the dtorque [Nm] --
         *  Maxon Motor Parameter
         *  Type Power 150 [W]
         *  Nominal Voltage 48 [V]
         *  No load speed 7590 [rpm]
         *  Nominal Torque (max) 0.187 [Nm]
         */
        
    protected:
        ros::V_Subscriber mtwLimbObject;
        ros::V_Subscriber mtwExoObject;
        ros::Publisher desTorqueObject;
        ros::NodeHandle nh;
};

#endif