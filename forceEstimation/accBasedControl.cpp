/*
 * A ROS NODE ON WHICH THE ACCELERATIONS AND ANGULAR VELOCITY FROM TWO MTw SENSORS
 * ARE USED AS INPUT TO CALCULATE A DESIRED OUTPUT ON THE MOTORS IN A JOINT
 * OF THE EXOSUIT DEVELOPED ON REHAB LAB @ EESC-USP.
 * 
 * Leonardo Felipe Lima Santos dos Santos, 2019
 * e-mail leonardo.felipe.santos@usp.br
 * github/bitbucket qleonardolp
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include "accBasedControl.h"


#define QUEUE_SIZE 10


int main (int argc, char **argv)
{
    ros::init(argc, argv, "acc_based_control"); 
    
    // LEMBRE-SE DE PASSAR OS IDs DOS SENSORES (MTw)
    // EXEMPLO: $ rosrun xsens_mtw_driver acc_based_control 322 324

    std::string const mtwIDlimb = argv[1];
    std::string const mtwIDexo = argv[2];

    accBasedControl controlNode(mtwIDlimb.substr(mtwIDlimb.size()-1, mtwIDlimb.size()), 
                                mtwIDexo.substr(mtwIDexo.size()-1, mtwIDexo.size()), QUEUE_SIZE);

    ros::Rate callbacks_rate(300);      // 500 Hz to keep 120 Hz for each topic (4x) with a safety margin
    while (ros::ok())
    {
        controlNode.m_CallbackQueue.callOne();  // Callbacks are called as the msgs arrives, them the order between then is mostly respected with callOne()
        callbacks_rate.sleep();
    }
    
    return 0;
}