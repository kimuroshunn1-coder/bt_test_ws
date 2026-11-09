#include <iostream>
#include <string>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_observer.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <fstream>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <airobot_interfaces/srv/string_command.hpp>

enum Color
{
  RED = 1,
  BLUE = 2,
  GREEN = 3
};

class SaySomething : public BT::StatefulActionNode
{
public:
    SaySomething(
        const std::string& name,
        const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {}
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("message")
        };
    }
    BT::NodeStatus onStart() override
    {
        auto msg = getInput<std::string>("message");
        if(!msg)
        {
            throw BT::RuntimeError("missing port");
        }
        std::cout
            << "Start: "
            << msg.value()
            << std::endl;
        start_time_ =
            std::chrono::steady_clock::now();
        last_printed_sec_ = -1;
        return BT::NodeStatus::RUNNING;
     }

     BT::NodeStatus onRunning() override
     {
         auto now =
             std::chrono::steady_clock::now();
         auto elapsed =
             std::chrono::duration_cast
             <
                 std::chrono::seconds
             >
             (
                 now - start_time_
             );
         int sec = elapsed.count();
         if(sec != last_printed_sec_)
         {
             std::cout
                 << "Elapsed: "
                 << sec
                 << " sec"
                 << std::endl;
             last_printed_sec_ = sec;
         }
         if(elapsed.count() >= 3)
         {
             std::cout
                 << "Completed"
                 << std::endl;
             return BT::NodeStatus::SUCCESS;
         }
         return BT::NodeStatus::RUNNING;
     }

     void onHalted() override
     {
          std::cout
              << "Node Halted"
              << std::endl;
     }
private:
    std::chrono::steady_clock::time_point start_time_;
    int last_printed_sec_ = -1;
};


class BringMeServiceNode :
  public BT::RosServiceNode<
      airobot_interfaces::srv::StringCommand>
{
public:

  using StringCommand =
    airobot_interfaces::srv::StringCommand;
  BringMeServiceNode(
      const std::string& name,
      const BT::NodeConfig& conf,
      const BT::RosNodeParams& params)
    : BT::RosServiceNode<
          StringCommand>(
          name,
          conf,
          params)
  {}
  static BT::PortsList providedPorts()
  {
      return providedBasicPorts(
      {
          BT::InputPort<std::string>(
              "command"),
          BT::OutputPort<std::string>(
              "answer")
      });
  }

  bool setRequest(
    StringCommand::Request::SharedPtr& request) override
  {
    auto command =
        getInput<std::string>("command");

    if(!command)
    {
        throw BT::RuntimeError(
            "missing command");
    }

    request->command = command.value();
    std::cout
        << "Request: "
        << request->command
        << std::endl;
    return true;
  }

  BT::NodeStatus onResponseReceived(
      const StringCommand::Response::SharedPtr&
      response
  ) override
  {
      std::cout
          << "Response: "
          << response->answer
          << std::endl;
      setOutput(
          "answer",
          response->answer);
      return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto ros_node = rclcpp::Node::make_shared(
                      "bt_client");
  BT::RosNodeParams params;
  params.nh = ros_node;
  params.default_port_value =
      "/command";
  params.server_timeout =
      std::chrono::seconds(10);
  params.wait_for_server_timeout =
      std::chrono::seconds(10);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<BringMeServiceNode>("BringMeService", params);
  factory.registerScriptingEnums<Color>();
  factory.registerScriptingEnum("THE_ANSWER", 42);
  std::string xml_file =
      ament_index_cpp::get_package_share_directory(
          "bt_script_test") +
       "/config/tree.xml";
  std::string xml_models =
       BT::writeTreeNodesModelXML(factory);
  std::ofstream file("models.xml");
  file << xml_models;
  file.close();
  auto tree = factory.createTreeFromFile(xml_file);;
  BT::TreeObserver observer(tree);
  BT::Groot2Publisher publisher(tree);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ros_node);
  while(rclcpp::ok())
  {
    executor.spin_some();
    tree.tickWhileRunning();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(10));
  }
  rclcpp::shutdown();
  return 0;
}

