#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

const float MAX_RAND_VALUE = 10.f;

inline float generateFloatNumber()
{
    return static_cast<float>(rand()) / (RAND_MAX / MAX_RAND_VALUE);
}

inline std::string generateTurtleName(int counter)
{
    std::string name = "Turtle_" + std::to_string(counter);
    return name;
}

inline float getDistanceBetweenTwoPoints(float x1, float x2, float y1, float y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

inline float angleBetweenTwoPoints(float x1, float x2, float y1, float y2)
{
    return atan2(y2 - y1, x2 - x1);
}

#endif // FUNCTIONS_H_
