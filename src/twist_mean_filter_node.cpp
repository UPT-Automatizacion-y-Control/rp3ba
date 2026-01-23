#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <filters/mean.hpp>
#include <filters/filter_base.hpp>

class TwistMeanFilterNode : public rclcpp::Node
{
public:
  TwistMeanFilterNode()
  : Node("twist_mean_filter_node")
  {
    const std::string filter_ns = "twist_mean_filter";

    this->declare_parameter("twist_mean_filter.type", "filters/MultiChannelMeanFilter" );
    this->declare_parameter("twist_mean_filter.params.number_of_observations", 5);

    filter_ = std::make_unique<filters::MultiChannelMeanFilter<double>>();

    filter_->configure( 6, "twist_mean_filter.params", "filters/MultiChannelMeanFilter",
          this->get_node_logging_interface(), this->get_node_parameters_interface() ); 

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("twist_out", 10);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("twist_in", 10,
      std::bind(&TwistMeanFilterNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "El nodo TwistMeanFilterNode esta corriendo");
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::vector<double> input(6), output(6);

    input[0] = msg->linear.x;     input[1] = msg->linear.y;    input[2] = msg->linear.z;
    input[3] = msg->angular.x; input[4] = msg->angular.y; input[5] = msg->angular.z;

    if (!filter_->update(input, output))
    {
      RCLCPP_WARN(this->get_logger(), "Fallo al actualizar el filtro");
      return;
    }

    for (double v : output)
      if (!std::isfinite(v))
      {
        RCLCPP_WARN(this->get_logger(), "Salida invalida");
        return;
      }

    geometry_msgs::msg::Twist out;
    out.linear.x  = output[0];   out.linear.y  = output[1];   out.linear.z  = output[2];
    out.angular.x = output[3]; out.angular.y = output[4]; out.angular.z = output[5];

    pub_->publish(out);
  }

  std::unique_ptr<filters::MultiChannelFilterBase<double>> filter_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistMeanFilterNode>());
  rclcpp::shutdown();
  return 0;
}

