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
            desTorqueObject = nh.advertise<std_msgs::Float32>("desired_Torque", queueSize);
            mtwLimbObject = nh.subscribe<geometry_msgs::Vector3Stamped>("free_acc_0034232" + mtwlimb, queueSize, &SubTwoMTwPubTorque::mtwLimbCallback, this);
            mtwExoObject = nh.subscribe<geometry_msgs::Vector3Stamped>("free_acc_0034232" + mtwexo, queueSize, &SubTwoMTwPubTorque::mtwExoCallback, this);
            
            ROS_INFO_STREAM("Subcribed on " << mtwLimbObject.getTopic() << ", " << mtwExoObject.getTopic());
        }

        void mtwLimbCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
        {
            AccLimb = msg->vector;
        }
        void mtwExoCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
        {
            AccExo = msg->vector;

            deltaAcc.x = AccExo.x - AccLimb.x;
            deltaAcc.y = AccExo.y - AccLimb.y;
            deltaAcc.z = AccExo.z - AccLimb.z;

            deltaAccNorm = sqrt((deltaAcc.x)*(deltaAcc.x) + (deltaAcc.y)*(deltaAcc.y) + (deltaAcc.z)*(deltaAcc.z));
            /*
             * -- Conversion from the accNorm [m/s^2] to the dtorque [Nm] --
             *  Maxon Motor Parameter
             *  Type Power 150 [W]
             *  Nominal Voltage 48 [V]
             *  No load speed 7590 [rpm]
             *  Nominal Torque (max) 0.187 [Nm]
             */
            desTorque.data = 1.0*deltaAccNorm;
            desTorqueObject.publish(desTorque);
        }

    private:
        geometry_msgs::Vector3 deltaAcc;        //vetor para armazenar a diferença entre os dois sensores
        geometry_msgs::Vector3 AccLimb;         // Limb acc, [m/s^2]
        geometry_msgs::Vector3 AccExo;          // Exo acc, [m/s^2]
        std_msgs::Float32 desTorque;            // Desired Torque [Nm] 
        float deltaAccNorm;                     // Norm of the deltaAcc vector

    protected:
        ros::Subscriber mtwLimbObject;
        ros::Subscriber mtwExoObject;
        ros::Publisher desTorqueObject;
        ros::NodeHandle nh;
};

#endif