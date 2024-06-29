#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
// 接下来我们创建一个类，它是 rclcpp::Node 的派生类
class FibonacciActionServer : public rclcpp::Node
{
public:
  // Fibonacci被定义为action_tutorials_interfaces::action::Fibonacci的别名
  // action_tutorials_interfaces是一个包含Fibonacci动作定义的包
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

  // 在ROS2中，rclcpp_action是action通信的C++实现
  // ServerGoalHandle是一个模板类，用于管理action服务器上的目标。通过指定模板参数Fibonacci，GoalHandleFibonacci成为了一个专门用于处理Fibonacci动作目标的类型。这使得动作服务器的实现更加类型安全，同时也提高了代码的清晰度和可维护性。
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  /**
   * @brief 构造函数 创建一个处理Fibonacci序列计算的ROS 2动作服务器
   * @param options 节点选项
   */
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options) // 继承Node类并调用其构造函数来初始化为一个ROS2节点，节点名称被设置为"fibonacci_action_server"。
  {
    using namespace std::placeholders;

    /**
     * 创建一个名为"Fibonacci"的动作服务器的示例
     * 动作服务器需要6个参数：
     * 动作类型名称：Fibonacci。
     * 将操作添加到的ROS2节点：this.
     * 动作名称：“fibonacci”。
     * 处理目标的回调函数：handle_goal
     * 用于处理取消的回调函数：handle_cancel。
     * 用于处理目标接受的回调函数：handle_accept。
    */
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_; // "Fibonacci"的动作服务器指针

  /**
   * @brief 处理目标请求
   * @param uuid 目标的唯一标识符
   * @param goal 目标
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "收到目标请求: %d", goal->order);
    (void)uuid;

    // 表示服务器不仅接受了目标请求，而且还将立即开始执行该目标。
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief 处理取消请求
   * @param goal_handle 目标句柄
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消目标的请求");
    (void)goal_handle;

    // 表示服务器接受了取消请求。
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief 处理目标接受
   * @param goal_handle 目标句柄
   */
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // 避免阻塞执行器，因此启动一个新线程
    // std::bind()函数用于将成员函数与类实例绑定在一起，以便在调用时可以访问类的成员函数。其中，_1是占位符，用于传递参数，也就是goal_handle。
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  /**
   * @brief 实际执行目标
   * @param goal_handle 目标句柄
   */
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "开始执行目标");

    // 创建一个循环，每秒发布一个反馈，直到达到目标
    rclcpp::Rate loop_rate(1);
    // 获取目标
    const auto goal = goal_handle->get_goal();
    // 创建一个反馈
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    // 创建一个序列，定义在action_tutorials_interfaces::action::Fibonacci::Feedback中
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    // 创建一个结果
    auto result = std::make_shared<Fibonacci::Result>();

    // 计算Fibonacci序列
    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // 检查是否有取消请求
      if (goal_handle->is_canceling()) {
        // 取消目标
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "目标已取消");
        return;
      }
      // 计算Fibonacci序列
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // 发布反馈
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "发布反馈");

      // 等待1秒
      loop_rate.sleep();
    }

    // 检查是否已经完成目标
    if (rclcpp::ok()) {
      // 完成目标
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "目标已完成");
    }
  }
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

// 使用RCLCPP_COMPONENTS_REGISTER_NODE宏注册FibonacciActionServer类
RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
