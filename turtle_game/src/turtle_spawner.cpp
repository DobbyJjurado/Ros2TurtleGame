#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtle_game_interfaces/msg/turtle_position.hpp>

const float MAX_RAND_VALUE = 10.f;

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner"),
                          counter_(0), waiting_for_service_reponse_(false)
    {
        this->declare_parameter("frecuency_spawn", 1);
        frecuency_spawn_ = this->get_parameter("frecuency_spawn").as_int();

        cli_spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        spawn_timer_ = this->create_wall_timer(std::chrono::seconds(frecuency_spawn_), std::bind(&TurtleSpawnerNode::callSpawnerTurtleService, this));
        pub_new_turtle_pos_ = this->create_publisher<turtle_game_interfaces::msg::TurtlePosition>("new_turtle_position", 10);
        RCLCPP_INFO(this->get_logger(), "[TurtleSpawnerNode] Spawner Node Created with a frequency of %d", frecuency_spawn_);
    }

    void callSpawnerTurtleService()
    {
        if (cli_spawner_->service_is_ready())
        {
            if (!waiting_for_service_reponse_)
            {
                waiting_for_service_reponse_ = true;
                thread_ = std::make_unique<std::thread>(std::bind(&TurtleSpawnerNode::SpawnTurtle, this));
                thread_->detach();
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[TurtleSpawnerNode::callSpawnerTurtle] Waiting for the service \"Spawner\"");
        }
    }

    void SpawnTurtle()
    {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = generateFloatNumber();
        request->y = generateFloatNumber();
        request->theta = generateFloatNumber();
        request->name = generateTurtleName();

        auto future = cli_spawner_->async_send_request(request);

        try
        {
            auto response = future.get();
            counter_++;
            publishNewTurtlePosition(request);
            RCLCPP_INFO(this->get_logger(), "[TurtleSpawnerNode::callSpawnerTurtle] Successfully spawn %s in x: %f, y: %f, thetha: %f", request->name.c_str(), request->x, request->y, request->theta);
            waiting_for_service_reponse_ = false;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error service response");
            waiting_for_service_reponse_ = false;
        }
    }

private:
    float generateFloatNumber()
    {
        return static_cast<float>(rand()) / (RAND_MAX / MAX_RAND_VALUE);
    }

    std::string generateTurtleName()
    {
        std::string name = "Turtle_" + std::to_string(counter_);
        return name;
    }

    void publishNewTurtlePosition(const turtlesim::srv::Spawn::Request::SharedPtr &pos)
    {
        auto msg = turtle_game_interfaces::msg::TurtlePosition();

        msg.name = pos->name;
        msg.x = pos->x;
        msg.y = pos->y;

        pub_new_turtle_pos_->publish(std::move(msg));
    }

    int counter_;
    bool waiting_for_service_reponse_;
    int frecuency_spawn_;
    std::unique_ptr<std::thread> thread_;

    rclcpp::TimerBase::SharedPtr spawn_timer_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr cli_spawner_;
    rclcpp::Publisher<turtle_game_interfaces::msg::TurtlePosition>::SharedPtr pub_new_turtle_pos_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
