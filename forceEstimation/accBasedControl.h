#ifndef ACC_BASED_CONTROL_H
#define ACC_BASED_CONTROL_H

/*
 * Class to build a SubcriberPublisher Node,
 * as seen on youtu.be/lR3cK9ZoAF8
 */

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>


class TwoMTwSubForcePub
{
    public:
        TwoMTwSubForcePub() {}
        TwoMTwSubForcePub(std::string const mtwlimb, std::string const mtwexo, int queueSize)
        {
            desTorqueObject = nh.advertise<std_msgs::Float32>("desired_Torque", queueSize);
            mtwLimbObject = nh.subscribe<geometry_msgs::Vector3Stamped>("free_acc_0034232" + mtwlimb, queueSize, &TwoMTwSubForcePub::mtwLimbCallback);
            mtwExoObject = nh.subscribe<geometry_msgs::Vector3Stamped>("free_acc_0034232" + mtwexo, queueSize, &TwoMTwSubForcePub::mtwExoCallback);
        }

        void mtwLimbCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        void mtwExoCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    private:
        geometry_msgs::Vector3 deltaFreeAcc;        //vetor para armazenar a diferença entre os dois sensores
        geometry_msgs::Vector3 freeAccLimb;         // Limb acc, [m/s^2]
        geometry_msgs::Vector3 freeAccExo;          // Exo acc, [m/s^2]

    protected:
        ros::Subscriber mtwLimbObject;
        ros::Subscriber mtwExoObject;
        ros::Publisher desTorqueObject;
        ros::NodeHandle nh;
};

#endif