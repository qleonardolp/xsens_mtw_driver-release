/*
 * Class to build a SubcriberPublisher Node,
 * as seen on youtu.be/lR3cK9ZoAF8
 * 
 * Leonardo Felipe Lima Santos dos Santos, 2019
 * e-mail leonardo.felipe.santos@usp.br
 * github/bitbucket qleonardolp
 */


#ifndef ACC_BASED_CONTROL_H
#define ACC_BASED_CONTROL_H


#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>

#define     GRAVITY           9.810
#define     INERTIA_EXO       1.000             // [Kg.m^2]
#define     MTW_DIST_LIMB     0.255             // [m]
#define     MTW_DIST_EXO      1.000             // [m]
#define     KP                1.000             // [Kg.m^2]
#define     KI                1.000             // [Kg.m^2/s]


class accBasedControl
{
    public:
        accBasedControl() {}
        accBasedControl(std::string const mtwlimb, std::string const mtwexo, int queueSize)
        {   
            nh.setCallbackQueue(&m_CallbackQueue);

	        desTorqueObject.resize(4);
            desTorqueObject[0] = nh.advertise<std_msgs::Float32>("desired_Torque", queueSize);
	        desTorqueObject[1] = nh.advertise<std_msgs::Float32>("Torque_Input", queueSize);
	        desTorqueObject[2] = nh.advertise<std_msgs::Float32>("Torque_Kp", queueSize);
	        desTorqueObject[3] = nh.advertise<std_msgs::Float32>("Torque_Ki", queueSize);

            auxDataObj = nh.advertise<std_msgs::String>("aux_data", queueSize);

            mtwLimbObject = nh.subscribe<sensor_msgs::Imu>("imu_0034232" + mtwlimb, queueSize, &accBasedControl::mtwLimbCB, this);
            mtwExoObject = nh.subscribe<sensor_msgs::Imu>("imu_0034232" + mtwexo, queueSize, &accBasedControl::mtwExoCB, this);
            
            ROS_INFO_STREAM("Subcribed on " << mtwLimbObject.getTopic() << ", " << mtwExoObject.getTopic());
            ROS_INFO_STREAM("Publishing on " << desTorqueObject[0].getTopic());    
        }


        void mtwLimbCB( const sensor_msgs::Imu::ConstPtr& mtw_msg)
        {
            AccLimb.x = (mtw_msg->angular_velocity.z)*(mtw_msg->angular_velocity.z)*MTW_DIST_LIMB;
            float theta_aux = asin((AccLimb.x - mtw_msg->linear_acceleration.x)/GRAVITY);

            if(isnanf(theta_aux) != 1)
                theta_g_limb = theta_aux;

            AccLimb.y = mtw_msg->linear_acceleration.y + GRAVITY * cos(theta_g_limb);

            VelLimb = mtw_msg->angular_velocity;

            ///// auxData is acc_y, acc_T, theta_g, gyro_z /////

            auxData.data = std::to_string(mtw_msg->linear_acceleration.y) + "," + std::to_string(AccLimb.y)
            + "," + std::to_string(180 * theta_g_limb/ M_PI) + "," + std::to_string(mtw_msg->angular_velocity.z); 

            auxDataObj.publish(auxData);

            ////////////////////////////////////////////////////

            //ROS_INFO_STREAM("theta_g_limb: " << 180 * theta_g_limb/ M_PI);

        }

        void mtwExoCB( const sensor_msgs::Imu::ConstPtr& mtw_msg)
        {
            AccExo.x = (mtw_msg->angular_velocity.z)*(mtw_msg->angular_velocity.z)*MTW_DIST_EXO;
            float theta_aux = asin((AccExo.x - mtw_msg->linear_acceleration.x)/GRAVITY);

            if(isnanf(theta_aux) != 1)
                theta_g_exo = theta_aux;

            AccExo.y = mtw_msg->linear_acceleration.y + GRAVITY * cos(theta_g_exo);   

            VelExo = mtw_msg->angular_velocity;

            desTorque.data = INERTIA_EXO*(1/MTW_DIST_LIMB)*AccLimb.y + KP*( (1/MTW_DIST_LIMB)*AccLimb.y - (1/MTW_DIST_EXO)*AccExo.y ) + KI*(VelLimb.z - VelExo.z);
	        desTorqueObject[0].publish(desTorque);

	        desTorque.data = INERTIA_EXO*(1/MTW_DIST_LIMB)*AccLimb.y;
	        desTorqueObject[1].publish(desTorque);

	        desTorque.data = KP*( (1/MTW_DIST_LIMB)*AccLimb.y - (1/MTW_DIST_EXO)*AccExo.y );
       	    desTorqueObject[2].publish(desTorque);

	        desTorque.data = KI*(VelLimb.z - VelExo.z);
	        desTorqueObject[3].publish(desTorque);
        }

        ros::CallbackQueue m_CallbackQueue;

    private:

        geometry_msgs::Vector3 AccLimb;         // Limb acc, [m/s^2]
        geometry_msgs::Vector3 AccExo;          // Exo acc, [m/s^2]

        geometry_msgs::Vector3 VelLimb;         // Limb Velocity, [rad/s]
        geometry_msgs::Vector3 VelExo;          // Exo Velocity, [rad/s]

        std_msgs::Float32 desTorque;            // Desired Torque Pub [Nm] 

        std_msgs::String auxData;

        float theta_g_limb;                     // [rad]
        float theta_g_exo;                      // [rad]


        /*
         * -- Conversion from the accNorm [m/s^2] to the dtorque [Nm] --
         *  Maxon Motor Parameter
         *  Type Power 150 [W]
         *  Nominal Voltage 48 [V]
         *  No load speed 7590 [rpm]
         *  Nominal Torque (max) 0.187 [Nm]
         */
        
    protected:
        ros::Subscriber mtwLimbObject;
        ros::Subscriber mtwExoObject;
        ros::V_Publisher desTorqueObject;
        ros::Publisher auxDataObj;
        ros::NodeHandle nh;
};

#endif
