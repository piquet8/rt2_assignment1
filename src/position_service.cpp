/**@package rt2_assignment1
*
* @file position_service.cpp
* @brief This node implements a ros service which from a random position
* @author Gianluca Piquet
* @version 0.1
* @date 03/04/2022
*

* @details
*
* Subscribes to: <BR>
* [None]
*
* Publishes to: <BR>
* [None]
*
* Services: <BR>
* /position_server
*
* Description: <BR>
*
*This node provides a randomly chosen position within a possible range of values. The request specifies the x 
*and y range parameters within which the position is to be chosen. In response, the service provides the 
*position obtained by a random function.
*
*/


#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
* @brief Random values generator
* @param M defines the minimum possible value for a random number
* @param N defines the maximum possible value for a random number
* @return boolean True
*
*This function generates two random numbers within a given range via the rand function. It is called when we 
*have a request from the client.
*
*/

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
* @brief main function
*
* @return 0
*
*This function initialises the node: random_position_server 
*then initialises the server: /position_server
*During its execution, it waits for a request from the client to the server to generate a new random 
*position.
*
*/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
