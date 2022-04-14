#include <vector>  
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

#include "imgproc_msgs/msg/imgproc.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

struct Vec2
{
    float x;
    float y;

    Vec2()
        : x(0.0)
        , y(0.0) {}

    Vec2(float _x, float _y)
        : x(_x)
        , y(_y) {}
};

class NavManagerClass : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

  int counter = 0;
  float dist = 1.0;
  std::vector<Vec2> wayPoints;
  mutable std::vector<float> radiusBuffer;
  mutable int max_buffer_size = 10;
  mutable float radius_avg = 0.0;

  bool isAborted = false;
  bool isCanceled = false;
  bool moveFlag = false;
  mutable float ballRadius = 0.0;
  explicit NavManagerClass(): Node("navigation_manager")
  {
    //waypoints
    
    wayPoints.push_back(Vec2(2.0, 2.0));
    wayPoints.push_back(Vec2(2.0, 0.0));
    wayPoints.push_back(Vec2(4.0, 2.5));
    wayPoints.push_back(Vec2(-2.0, 2.0));
    wayPoints.push_back(Vec2(0.0, 0.0));
    

    /*
    wayPoints.push_back(Vec2(0.0, 2.0));
    wayPoints.push_back(Vec2(2.0, 0.0));
    wayPoints.push_back(Vec2(0.0, -2.0));
    wayPoints.push_back(Vec2(-2.0, 0.0));
    */
   
    //create action client
    this->client_ptr_  = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    //timer call back
    timer_ = this->create_wall_timer(2000ms, std::bind(&NavManagerClass::timer_callback, this));
    //subscriber
    subscription_ = this->create_subscription<imgproc_msgs::msg::Imgproc>("img_data", 10, std::bind(&NavManagerClass::topic_callback, this, _1));

  }

  void sendGoal(float x, float y) {
    int counter= 0;
    bool entryFlag = false;

    while (!entryFlag)
    {
      counter ++;
      if(this->client_ptr_->wait_for_action_server()) 
      {
        entryFlag = true;
      }
      else
      {
        if(counter > 100000)
        {
          RCLCPP_INFO(get_logger(), "No connection to server");
          exit(0);
        } 
      }
    }
    
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
    goal_msg.pose.pose.orientation.z = 0.0;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&NavManagerClass::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&NavManagerClass::resultCallback, this, _1);
    
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    
  }
  void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);

    dist = feedback->distance_remaining;
  
  }
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_ERROR(get_logger(), "Success!!!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        isAborted = true;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        isCanceled = true;
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
  }

  private:
    void timer_callback()
    {
      
      RCLCPP_INFO(this->get_logger(), "TIMERCALLBACK");

      if(ballRadius > 0)
      {
        moveFlag = true;
      }
      else
      {
        moveFlag = false;
      }

      if(moveFlag)
      {
        if(dist < 0.35)
        {
            counter ++;
            sendGoal(wayPoints[counter%wayPoints.size()].x, wayPoints[counter%wayPoints.size()].y);  
        }

        if(isAborted || isCanceled)
        {
          sendGoal(wayPoints[counter%wayPoints.size()].x, wayPoints[counter%wayPoints.size()].y);  
          isAborted = false;
          isCanceled = false;
        }
      }
      else
      {
         client_ptr_->async_cancel_all_goals();
      }      
    }

    void topic_callback(const imgproc_msgs::msg::Imgproc::SharedPtr msg) const
    {
      ballRadius = msg->radius;
      RCLCPP_INFO(this->get_logger(), "radius: '%f'", ballRadius);

      
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<imgproc_msgs::msg::Imgproc>::SharedPtr subscription_;
    
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavManagerClass>();
  node->sendGoal(-0.70, 0.0);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
