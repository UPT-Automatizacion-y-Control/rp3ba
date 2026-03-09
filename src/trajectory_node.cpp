#include "rclcpp/rclcpp.hpp" 
#include <cmath> 
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
        
class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode() : Node("trajectory_node")
    {
        this->declare_parameter<double>("delay_time", 0.01);
        this->get_parameter("delay_time", delay_time);

        t0 = this->now().seconds();

        user_goal = createPoseStamped(h_x, h_y, h_z, h_roll, h_pitch, h_yaw);     

        trayectoria_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("trayectoria", 10);

        sub_modo = this->create_subscription<std_msgs::msg::Int16>("modo", 10,
            std::bind(&TrajectoryNode::modoCallback, this, std::placeholders::_1) );

        sub_user_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>("user_goal", 10,
            std::bind(&TrajectoryNode::userGoalCallback, this, std::placeholders::_1) );

        sub_platfom_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("platform_pose", 10,
            std::bind(&TrajectoryNode::pPoseCallback, this, std::placeholders::_1) );

        timer = this->create_wall_timer( std::chrono::duration<double>(delay_time),
                     std::bind(&TrajectoryNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "El nodo trajectory esta corriendo con dt: %.2f", delay_time);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trayectoria_pub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_modo;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_user_goal;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_platfom_pose;
    rclcpp::TimerBase::SharedPtr timer;

    double delay_time, t0;
    int modo = 0;
    bool transition = true;
    bool ppose_recieved = false;
    geometry_msgs::msg::PoseStamped user_goal;
    geometry_msgs::msg::PoseStamped ppose;
    
    const float transition_time = 2.0;
    const double A_x = 0.02, A_y = 0.02, A_z = 0.02;
    const double T_x = 4.00, T_y = 8.00, T_z = 5.00;
    const double A_roll = 0.25, A_pitch = 0.25, A_yaw = 0.25;
    const double T_roll = 4.00, T_pitch = 8.00, T_yaw = 5.00;   
    const double h_x = 0.0, h_y = 0.0, h_z = 0.17, h_roll = 0.0, h_pitch = 0.0, h_yaw = 0.0 ;

    void modoCallback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        modo = msg->data;
        transition = true;
        ppose_recieved = false;
        t0 = this->now().seconds();
    }

    void userGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        user_goal = *msg;
    }

    void pPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!ppose_recieved)
        {
            ppose = *msg;
            ppose_recieved = true;
        }
    }

    void timerCallback()
    {            
        double t = this->now().seconds() - t0;
        geometry_msgs::msg::PoseStamped msg;
        
        float x = h_x, y = h_y, z = h_z, roll = h_roll, pitch = h_pitch, yaw = h_yaw;
        
        if (transition)
        {
            if (!ppose_recieved)
                  return;
                  
            if (t < transition_time)
            {
                auto p = ppose.pose.position;
                x = p.x + (h_x - p.x)*t/transition_time;   
                y = p.y + (h_y - p.y)*t/transition_time;   
                z = p.z + (h_z - p.z)*t/transition_time;
                                
                auto q = ppose.pose.orientation;
                tf2::Quaternion quat(q.x, q.y, q.z, q.w);
                double p_roll, p_pitch, p_yaw;
                tf2::Matrix3x3(quat).getRPY(p_roll, p_pitch, p_yaw);
    
                roll = p_roll + (h_roll - p_roll)*t/transition_time;   
                pitch = p_pitch + (h_pitch - p_pitch)*t/transition_time;   
                yaw = p_yaw + (h_yaw - p_yaw)*t/transition_time; 
            }
            else
            {
                transition = false;
                t0 = this->now().seconds();
            }
        }
        else
        {
            switch(modo)
            {
                case 1:
                    x = h_x + A_x*sin(2*M_PI*t/T_x); break;
                case 2: 
                    y  = h_y + A_y*sin(2*M_PI*t/T_y); break;
                case 3: 
                    z  = h_z + A_z*sin(2*M_PI*t/T_z); break;
                case 4: 
                    roll  = h_roll + A_roll*sin(2*M_PI*t/T_roll); break;
                case 5: 
                    pitch = h_pitch + A_pitch*sin(2*M_PI*t/T_pitch); break;
                case 6: 
                    yaw = h_yaw + A_yaw*sin(2*M_PI*t/T_yaw); break;
                case 7: 
                    x = h_x + A_x*sin(2*M_PI*t/T_x);   
                    y = h_y + A_y*sin(2*M_PI*t/T_y); break;
                case 8: 
                    roll  = h_roll + A_roll*sin(2*M_PI*t/T_roll);  
                    pitch = h_pitch + A_pitch*sin(2*M_PI*t/T_pitch); break;               
                case 9:
                    msg = user_goal; break;
            }
        }
        
        if (modo != 9)
            msg = createPoseStamped(x, y, z, roll, pitch, yaw);     
        
        trayectoria_pub->publish(msg);
    }
    
    geometry_msgs::msg::PoseStamped createPoseStamped(float x, float y, float z, float roll, float pitch, float yaw)
    {
         geometry_msgs::msg::PoseStamped ps;
         ps.header.stamp = this->now();
         ps.header.frame_id = "World_Link";
         
         ps.pose.position.x = x;
         ps.pose.position.y = y;
         ps.pose.position.z = z;
         
         tf2::Quaternion q;
         q.setRPY(roll, pitch, yaw);
         ps.pose.orientation = tf2::toMsg(q);
        
        return ps;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
