#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  // Fibonacci被定义为action_tutorials_interfaces::action::Fibonacci的别名
  // action_tutorials_interfaces是一个包含Fibonacci动作定义的包
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

  // 在ROS2中，rclcpp_action是action通信的C++实现
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  /**
   * @brief 构造函数 创建一个处理Fibonacci序列计算的ROS2 action客户端
   * @param options 节点选项
   */
  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options) // 继承Node类并调用其构造函数来初始化为一个ROS2节点，节点名称被设置为"fibonacci_action_client"。
  {
    // 创建一个名为"Fibonacci"的动作客户端
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    // 创建一个定时器，每500毫秒调用一次send_goal函数
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
  }

  /**
   * @brief 发送目标
   */
  void send_goal()
  {
    using namespace std::placeholders;

    // 取消定时器
    this->timer_->cancel();

    // 等待action服务器可用
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "等待后action服务器不可用");
      rclcpp::shutdown();
    }

    // 创建一个名为goal_msg的Fibonacci::Goal类型的消息
    auto goal_msg = Fibonacci::Goal();

    // 设置目标的order为10
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "发送目标请求: %d", goal_msg.order);

    // 创建了一个send_goal_options对象，这个对象用于配置发送动作目标时的选项
    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    // 当目标被接受或拒绝时会调用goal_response_callback回调函数
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);

    // 当接收到反馈时会调用feedback_callback回调函数
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);

    // 当接收到结果时会调用result_callback回调函数
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);

    // 发送目标
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;  // Fibonacci动作客户端指针
  rclcpp::TimerBase::SharedPtr timer_;  // 定时器

  /**
   * @brief 目标响应回调函数
   * @param goal_handle 目标句柄
   */
  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "执行目标被拒绝");
    } else {
      RCLCPP_INFO(this->get_logger(), "执行目标被接受，等待结果...");
    }
  }

  /**
   * @brief 反馈回调函数
   * @param goal_handle 目标句柄
   * @param feedback 反馈
   */
  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "收到的序列中的下一个号码为: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  /**
   * @brief 结果回调函数
   * @param result 结果
   */
  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "执行目标被中止");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "执行目标被取消");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知结果代码");
        return;
    }
    std::stringstream ss;
    ss << "执行结果为: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
