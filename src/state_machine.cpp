#include <memory>
#include <chrono>
#include <cinttypes>
#include <functional>
// add the ROS header
#include "rclcpp/rclcpp.hpp"
// add the headers for
#include "rt2_assignment1/srv/random_position.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using std::placeholders::_1; 
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

// The FSM Node is implemented as a class so that it can be structured as a component. Indeed, it shall communicates with the ros node by means of the ros1_bridge package
class FSM : public rclcpp::Node
{
public:
  FSM(const rclcpp::NodeOptions & options)
  : Node("state_machine", options)
  {
    service_ = this->create_service<rt2_assignment1::srv::Command>("/user_interface", std::bind(&FSM::handle_service, this,_1, _2, _3)); // service for the Command 
    client_1 = this->create_client<rt2_assignment1::srv::Position>("/go_to_point"); // client_1 for the position 
    while (!client_1->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client_1 interrupted while waiting for service to appear.");
      return;
    }
    
    }
    client_2 = this->create_client<rt2_assignment1::srv::RandomPosition>("/position_server"); // client_2 for the RandomPosition 
    while (!client_2->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client_2 interrupted while waiting for service to appear.");
      return;
    }
   
    }
  }

  

private:

  bool  start = false;
// It requires a random position. Then it initialise the request of type position with these random values. By means of them, it allows the robot to reach the expected Pose 
  void state_machine_(){
      if (this->start){
	   	 auto request_2 = std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
		
		 request_2->x_max = 5.0;
		 request_2->x_min = -5.0;
		 request_2->y_max = 5.0;
		 request_2->y_min = -5.0;
		 auto callback = [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future) {
		         auto request_1 = std::make_shared<rt2_assignment1::srv::Position::Request>();
		         this->response_2=future.get();
		         std::cout<<"received random response"<<std::endl;
		         std::cout<<this->response_2->x<<std::endl;
		         request_1->x = this->response_2->x;
		         request_1->y = this->response_2->y;
		         request_1->theta = this->response_2->theta;
		         std::cout << "\nGoing to the position: x= " << request_1->x << " y= " <<request_1->y << " theta = " <<request_1->theta << std::endl;
		         auto response_received_callback2 = [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future) {
		                                (void)future;
		                                std::cout << "Position reached" << std::endl;
		                                this->state_machine_();
            
                };
                
                auto result_1 = client_1->async_send_request(request_1, response_received_callback2);
           };
           auto future_result = client_2->async_send_request(request_2, callback);

	}
	else{std::cout << "else state" << std::endl;
	std::cout << "start in else = " << start << std::endl;
	}
	std::cout << "out of else " << std::endl;
  }
 


// It takes as service request string which is "start" if the user aims at activating the robot's behaviour. If so, state_machine_() function is called.
 
  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rt2_assignment1::srv::Command::Request> request,
  const std::shared_ptr<rt2_assignment1::srv::Command::Response> response)
  {
  (void)request_header;
     if (request->command == "start"){
  
    	this->start = true;
    	std::cout << "start = " << start << std::endl;
    }
    else{
    	this->start = false;
    	std::cout << "start = " << start << std::endl;
    }
    std::cout << "state_machine call" << std::endl;
    response->ok=true;
    this->state_machine_();
   }
    
  rclcpp::Service<rt2_assignment1::srv::Command>::SharedPtr service_;  
  rclcpp::Client<rt2_assignment1::srv::Position>::SharedPtr client_1; 
  rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedPtr client_2; 
  std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response_2;
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::FSM)
