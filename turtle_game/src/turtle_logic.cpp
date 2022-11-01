#include <rclcpp/rclcpp.hpp>
#include <turtle_game_interfaces/msg/turtle_position.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>

#include <turtle_game/functions.hpp>
#include <turtle_game/topics.hpp>

class TurtleLogicNode : public rclcpp::Node
{
public:
    TurtleLogicNode() : Node("turtle_logic")
    {

        waiting_for_service_reponse_ = false;

        sub_new_turtle_position_ = this->create_subscription<turtle_game_interfaces::msg::TurtlePosition>(TOPICS::NEW_TURTLE_POSITION, 10,
                                                                                                          std::bind(&TurtleLogicNode::SubscribeNewTurtlePosition, this, std::placeholders::_1));
        sub_my_position_ = this->create_subscription<turtlesim::msg::Pose>(TOPICS::MY_TURTLE_POSE, 1,
                                                                           std::bind(&TurtleLogicNode::SubscribeToMyPosition, this, std::placeholders::_1));

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(TOPICS::MY_TURTLE_CMD_VEL, 10);

        cli_kill = this->create_client<turtlesim::srv::Kill>(SERVICES::KILL);

        check_timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&TurtleLogicNode::EstimateNextMovement, this));

        RCLCPP_INFO(this->get_logger(), "[TurtleLogicNode] Node Created");
    }

private:
    void SubscribeToMyPosition(const turtlesim::msg::Pose::SharedPtr msg)
    {
        my_position_ = msg;
    }

    void SubscribeNewTurtlePosition(const turtle_game_interfaces::msg::TurtlePosition::SharedPtr msg)
    {
        auto check_vector = [msg](const turtle_game_interfaces::msg::TurtlePosition::SharedPtr x)
        {
            return (msg->name == x->name);
        };

        if (std::find_if(turtle_positions_.begin(), turtle_positions_.end(), check_vector) == turtle_positions_.end())
        {
            turtle_positions_.push_back(msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[TurtleLogicNode::SubscribeNewTurtlePosition] Already exits");
        }
    }

    int getIndexTurtleClose()
    {
        std::vector<float> v_dist(turtle_positions_.size());
        std::generate(v_dist.begin(), v_dist.end(), [this, counter = 0]() mutable
                      { 
                        float distance =
                            abs(getDistanceBetweenTwoPoints(my_position_->x, turtle_positions_.at(counter)->x,
                                                        my_position_->y, turtle_positions_.at(counter)->y));                       
                        counter++;
                        return distance; });

        auto it = std::min_element(v_dist.begin(), v_dist.end());
        int index = std::distance(v_dist.begin(), it);

        return index;
    }

    void EstimateNextMovement()
    {

        if (turtle_positions_.empty())
            return;
        else
        {
            int index = getIndexTurtleClose();

            float distance = getDistanceBetweenTwoPoints(my_position_->x, turtle_positions_.at(index)->x,
                                                         my_position_->y, turtle_positions_.at(index)->y);
            auto cmd_vel = geometry_msgs::msg::Twist();

            if (distance > 1)
            {
                float e_theta = angleBetweenTwoPoints(my_position_->x, turtle_positions_.at(index)->x,
                                                      my_position_->y, turtle_positions_.at(index)->y) -
                                my_position_->theta;

                cmd_vel.linear.x = 4 * distance;

                if (e_theta > M_PI)
                {
                    e_theta -= 2 * M_PI;
                }
                else if (e_theta < -M_PI)
                {
                    e_theta += 2 * M_PI;
                }
                cmd_vel.angular.z = 6 * e_theta;
            }
            else
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                RemoveCloseTurtle();
            }

            pub_cmd_vel_->publish(std::move(cmd_vel));
        }
    }

    void RemoveCloseTurtle()
    {
        int index_to_remove = -1;
        for (int i = 0; i < turtle_positions_.size(); ++i)
        {
            float distance =
                getDistanceBetweenTwoPoints(my_position_->x, turtle_positions_.at(i)->x,
                                            my_position_->y, turtle_positions_.at(i)->y);
            if (distance < 1)
                index_to_remove = i;
        }

        if (index_to_remove != -1 && !waiting_for_service_reponse_)
        {
            killTurtle(turtle_positions_.at(index_to_remove)->name);
            turtle_positions_.erase(turtle_positions_.begin() + index_to_remove);
        }
    }

    void killTurtle(const std::string &name)
    {
        if (cli_kill->service_is_ready())
        {
            waiting_for_service_reponse_ = true;
            thread_ = std::make_unique<std::thread>(std::bind(&TurtleLogicNode::callCliKill, this, name));
            thread_->detach();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[TurtleSpawnerNode::callSpawnerTurtle] Waiting for the service \"Spawner\"");
        }
    }

    void callCliKill(const std::string &name)
    {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
        auto future = cli_kill->async_send_request(request);

        try
        {
            auto response = future.get();
            waiting_for_service_reponse_ = false;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error service response");
            waiting_for_service_reponse_ = false;
        }
    }

    // Subscribers
    rclcpp::Subscription<turtle_game_interfaces::msg::TurtlePosition>::SharedPtr sub_new_turtle_position_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_my_position_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    // Clients
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr cli_kill;

    std::vector<turtle_game_interfaces::msg::TurtlePosition::SharedPtr> turtle_positions_;

    rclcpp::TimerBase::SharedPtr check_timer_;
    turtlesim::msg::Pose::SharedPtr my_position_;

    bool waiting_for_service_reponse_;
    std::unique_ptr<std::thread> thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleLogicNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
