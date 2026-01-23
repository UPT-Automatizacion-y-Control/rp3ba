#include "rclcpp/rclcpp.hpp" 
#include <cmath> 
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int16.hpp"
        
class TrajectoryGenerator : public rclcpp::Node
{
public:
    TrajectoryGenerator() : Node("trayectoria")
    {
        this->declare_parameter<double>("delay_time", 0.1);
        this->get_parameter("delay_time", delay_time);

        t0_ = this->now().seconds();
        
        // Posición inicial para la variable definida por el usuario (HOME)
        home.linear.x  = 0.0;   home.linear.y  = 0.0;   home.linear.z  = 0.15; 
        home.angular.x = 0.0; home.angular.y = 0.0; home.angular.z = 0.0;
        
        user_goal= home;

        trayectoria_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("trayectoria", 10);

        sub_modo_ = this->create_subscription<std_msgs::msg::Int16>("modo", 10,
            std::bind(&TrajectoryGenerator::modoCallback, this, std::placeholders::_1) );

        sub_user_goal = this->create_subscription<geometry_msgs::msg::Twist>("user_goal", 10,
            std::bind(&TrajectoryGenerator::userGoalCallback, this, std::placeholders::_1) );

        timer_ = this->create_wall_timer( std::chrono::duration<double>(delay_time),
            std::bind(&TrajectoryGenerator::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "El nodo trayectoria esta corriendo con dt: %.2f", delay_time);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr trayectoria_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_modo_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_user_goal;
    rclcpp::TimerBase::SharedPtr timer_;

    double delay_time, t0_;
    int modo_;
    geometry_msgs::msg::Twist user_goal;
    geometry_msgs::msg::Twist home;

    const double Dx_min = -0.03, Dx_max = 0.03, TDx = 5;
    const double Dy_min = -0.03, Dy_max = 0.03, TDy = 5;
    const double Dz_min =  0.15, Dz_max = 0.20, TDz = 5;
    const double Ax = 0.25, TAx = 5;
    const double Ay = 0.25, TAy = 5;
    const double Az = 0.25, TAz = 5;

    void modoCallback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        modo_ = msg->data;
    }

    void userGoalCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        user_goal= *msg;
    }

    void timerCallback()
    {
        double t = this->now().seconds() - t0_;

        geometry_msgs::msg::Twist msg;
        msg = home; 

        switch(modo_)
        {
            case 1:
                msg.linear.x  = ((Dx_max-Dx_min)/2.0) * sin( (2 * M_PI * t / TDx) - M_PI/2.0) + (Dx_max + Dx_min)/2.0 ;
                break;

            case 2: 
                msg.linear.y  = ((Dy_max-Dy_min)/2.0) * sin( (2 * M_PI * t / TDy) - M_PI/2.0) + (Dy_max + Dy_min)/2.0 ;
                break;

            case 3: 
                msg.linear.z  = ((Dz_max-Dz_min)/2.0) * sin( (2 * M_PI * t / TDz) - M_PI/2.0) + (Dz_max + Dz_min)/2.0 ;
                break;

            case 4: 
                msg.linear.z  = 0.17; 
                msg.angular.x = Ax * sin(2 * M_PI * t / TAx);
                break;

            case 5: 
                msg.linear.z  = 0.17; 
                msg.angular.y = Ay * sin(2 * M_PI * t / TAy);
                break;

            case 6: 
                msg.angular.z = Az * sin(2 * M_PI * t / TAz);
                break;

            case 7: 
                msg.linear.x  = Dx_max * cos(2 * M_PI * t / TDx);
                msg.linear.y  = Dy_max * sin(2 * M_PI * t / TDy);
                break;

            case 8: 
                msg.linear.z  = 0.17; 
                msg.angular.x = Ax * cos(2 * M_PI * t / TAx);
                msg.angular.y = Ay * sin(2 * M_PI * t / TAy);
                break;
                
            case 9:
                msg = user_goal;
                break;
        }
        
        trayectoria_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGenerator>());
    rclcpp::shutdown();
    
    return 0;
}
