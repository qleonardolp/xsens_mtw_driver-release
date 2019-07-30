/*
 * A ROS NODE ON WHICH THE ACCELERATIONS FROM TWO MTw SENSORS
 * ARE USED AS INPUT TO CALCULATE A DESIRED OUTPUT ON THE MOTORS IN A JOINT
 * OF THE EXOSUIT DEVELOPED ON REHAB LAB @ EESC-USP.
 * 
 * Leonardo Felipe Lima Santos dos Santos, 2019
 * e-mail leonardo.felipe.santos@usp.br
 * github/bitbucket qleonardolp
 */

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "accBasedControl.h"


int main (int argc, char **argv)
{
    ros::init(argc, argv, "acc_based_control"); 
    
    // LEMBRE-SE DE PASSAR OS IDs DOS SENSORES (MTw)
    // EXEMPLO: $ rosrun xsens_mtw_driver acc_based_control 322 324

    std::string const mtwIDlimb = argv[1];
    std::string const mtwIDexo = argv[2];

    SubTwoMTwPubTorque theNode(mtwIDlimb.substr(mtwIDlimb.size()-1, mtwIDlimb.size()), 
                               mtwIDexo.substr(mtwIDexo.size()-1, mtwIDexo.size()), 1000);

    ros::spin();
    return 0;
}