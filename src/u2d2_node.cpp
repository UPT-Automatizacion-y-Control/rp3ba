#include "rclcpp/rclcpp.hpp" 
#include "dynamixel_sdk/dynamixel_sdk.h" 
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath> 

using namespace dynamixel;
// https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/#control-table

#define DEVICE_NAME "/dev/ttyUSB0"
#define PROTOCOL_VERSION 2.0 
#define BAUDRATE 3000000
#define ADDR_DRIVE_MODE 10
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VEL 112
#define ADDR_GOAL_POS 116
#define ADDR_PRESENT_PWM  128  
#define ADDR_PRESENT_LOAD  128  
#define ADDR_PRESENT_VEL  128  
#define ADDR_PRESENT_POS 132 
#define SIZE_PWM  2   
#define SIZE_LOAD 2    
#define SIZE_VEL  4   
#define SIZE_POS 4         
#define TOTAL_DXL_IDS 6
#define ADDR_KP 84
#define ADDR_KI 82
#define ADDR_KD 80

const uint8_t DXL_IDS[] = {11, 12, 13, 21, 22, 23}; // JR
const float DEG2RAD = 3.1416f/180.0f; 
const float POS2RAW = 4095.0f/360.0f; 
const float RAW2VEL = 0.02398f; // rad/s
const uint16_t KP_TBL = 2500, KI_TBL = 300, KD_TBL = 1, DM_TBL = 4;

class U2D2Node : public rclcpp::Node
{
public:
    U2D2Node() : Node("u2d2_node")
    {
        this->declare_parameter<double>("delay_time", 0.01);
        this->get_parameter("delay_time", delay_time);
        
        if (!initializeDynamixels()) {
            RCLCPP_ERROR(this->get_logger(), "Error during Dynamixel initialization. Shutting down.");
            rclcpp::shutdown();
        }        
        
        dynamixel_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joints_state", 10);
        dynamixel_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joints_goal", 10, 
            std::bind(&U2D2Node::goalCallback, this, std::placeholders::_1) );
        
        timer = this->create_wall_timer( std::chrono::duration<double>(delay_time),
                     std::bind(&U2D2Node::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "El nodo u2d2 esta corriendo con dt: %.2f", delay_time);
    }

    ~U2D2Node()
    {
            portHandler->closePort();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr dynamixel_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr dynamixel_sub;
    rclcpp::TimerBase::SharedPtr timer;
    
    sensor_msgs::msg::JointState goal_msg;
    sensor_msgs::msg::JointState msg_out;
    bool goal_msg_recieved = false;
    
    std::unique_ptr<dynamixel::PortHandler> portHandler;
    std::unique_ptr<dynamixel::PacketHandler> packetHandler;
    std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite;
    std::unique_ptr<dynamixel::GroupBulkRead> groupBulkRead;
    
    double delay_time;
    
    bool initializeDynamixels()
    {
        msg_out.name.resize(TOTAL_DXL_IDS);
        msg_out.position.resize(TOTAL_DXL_IDS);
        msg_out.velocity.resize(TOTAL_DXL_IDS);
        
        portHandler.reset(dynamixel::PortHandler::getPortHandler(DEVICE_NAME));
        packetHandler.reset(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
        groupSyncWrite = std::make_unique<dynamixel::GroupSyncWrite>(portHandler.get(), packetHandler.get(), ADDR_GOAL_POS, SIZE_POS);
        groupBulkRead = std::make_unique<dynamixel::GroupBulkRead>(portHandler.get(), packetHandler.get());
        
        if (!portHandler->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
            return false;
        }

        if (!portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
            return false;
        }
        
        uint32_t duration_data = (unsigned int) (delay_time*1000); //ms   
        if(duration_data < 1)  
            duration_data = 1; 
        
        uint8_t dxl_error = 0;
        int comm_result = COMM_TX_FAIL;
        
        uint8_t write_data[SIZE_POS] = {0,0,0,0};

        for(int k = 0; k < TOTAL_DXL_IDS; k++)
        {
            comm_result = groupSyncWrite->addParam(DXL_IDS[k], write_data);
            if (comm_result != true){
                RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncWrite for Dynamixel ID %d", DXL_IDS[k]);
                return false;
            }
           comm_result = groupBulkRead->addParam(DXL_IDS[k], ADDR_PRESENT_VEL, SIZE_VEL + SIZE_POS);
            if (comm_result != true){
                RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncRead for Dynamixel ID %d", DXL_IDS[k]);
                return false;
            }
            comm_result = packetHandler->write2ByteTxRx(portHandler.get(), DXL_IDS[k], ADDR_KP, KP_TBL, &dxl_error);
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set KP gain for Dynamixel ID %d!", DXL_IDS[k]);
                return false;
            } 
            comm_result = packetHandler->write2ByteTxRx(portHandler.get(), DXL_IDS[k], ADDR_KI, KI_TBL, &dxl_error);
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set KI gain for Dynamixel ID %d!", DXL_IDS[k]);
                return false;
            } 
            comm_result = packetHandler->write2ByteTxRx(portHandler.get(), DXL_IDS[k], ADDR_KD, KD_TBL, &dxl_error);
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set KD gain for Dynamixel ID %d!", DXL_IDS[k]);
                return false;
            } 
            comm_result = packetHandler->write4ByteTxRx(portHandler.get(), DXL_IDS[k], ADDR_GOAL_VEL, duration_data, &dxl_error);
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set Profile Velocity for Dynamixel ID %d!", DXL_IDS[k]);
                return false;
            } 
            comm_result = packetHandler->write1ByteTxRx(portHandler.get(), DXL_IDS[k], ADDR_DRIVE_MODE, DM_TBL, &dxl_error);
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to change drive mode for Dynamixel ID %d!", DXL_IDS[k]);
                return false;
            } 
            comm_result = packetHandler->write1ByteTxRx(portHandler.get(), DXL_IDS[k], ADDR_TORQUE_ENABLE, 1, &dxl_error);
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for Dynamixel ID %d!", DXL_IDS[k]);
                return false;
            } 
            
            msg_out.name[k] = "dxl_" + std::to_string(DXL_IDS[k]);
        }
        
        return true;
    }
    
    void timerCallback()
    {   
        if (goal_msg_recieved)
            writeState();
        readState();     
    }
    
    void goalCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
          goal_msg = *msg;
          goal_msg_recieved = true;
    } 
    
    void writeState()
    {
        int dxl_comm_result = false;
        uint32_t position;
        uint8_t write_data[SIZE_POS];
            
        for(int k = 0; k < TOTAL_DXL_IDS; k++)
        {
            position = (unsigned int) (radian2raw(goal_msg.position[k]));            
            write_data[0] = DXL_LOBYTE(DXL_LOWORD(position));
            write_data[1] = DXL_HIBYTE(DXL_LOWORD(position));
            write_data[2] = DXL_LOBYTE(DXL_HIWORD(position));
            write_data[3] = DXL_HIBYTE(DXL_HIWORD(position));
            
            dxl_comm_result = groupSyncWrite->changeParam(DXL_IDS[k], write_data);
            if (dxl_comm_result != true)
                RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncWrite for Dynamixel ID %d", DXL_IDS[k]);
        }
        
        dxl_comm_result = groupSyncWrite->txPacket();
        if (dxl_comm_result != COMM_SUCCESS) 
            RCLCPP_ERROR(this->get_logger(), "Failed to groupSyncWrite position! Result: %d", dxl_comm_result);    
    }
    
    void readState()
    {         
        int32_t vel_raw, pos_raw; 
        
        int dxl_comm_result = groupBulkRead->txRxPacket();
        
        if (dxl_comm_result == COMM_SUCCESS) 
        {      
            for(int k = 0; k < TOTAL_DXL_IDS; k++)
            {
                vel_raw = groupBulkRead->getData(DXL_IDS[k], ADDR_PRESENT_VEL, SIZE_VEL);
                msg_out.velocity[k] = vel_raw*RAW2VEL; 
                pos_raw = groupBulkRead->getData(DXL_IDS[k], ADDR_PRESENT_POS, SIZE_POS);
                msg_out.position[k] =(pos_raw/POS2RAW)*DEG2RAD - M_PI; 
            }
            dynamixel_pub->publish(msg_out);
        }
        else 
            RCLCPP_ERROR(this->get_logger(), "Failed to get velocity and position! Result: %u", dxl_comm_result);    
    }
    
    float radian2raw(float angle)
    {
      float deg = angle*180.0/M_PI + 180.0;
      
       if(deg>360.0)
             deg -= 360.0;
         if(deg<0)
             deg += 360.0;
             
      return(deg*POS2RAW);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<U2D2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
