#ifndef TOPICS_H_
#define TOPICS_H_

class TOPICS
{
public:
    static constexpr auto NEW_TURTLE_POSITION = "new_turtle_position";
    static constexpr auto MY_TURTLE_POSE = "/turtle1/pose";
    static constexpr auto MY_TURTLE_CMD_VEL = "/turtle1/cmd_vel";
};

class SERVICES
{
public:
    static constexpr auto SPAWN = "spawn";
    static constexpr auto KILL = "kill";
};

#endif // TOPICS_H_