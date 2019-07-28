#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry>

struct BoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
    float cube_length{ 0.f };
    float cube_width{ 0.f };
    float cube_height{ 0.f };
};

struct Box
{
	float x_min{ 0.f };
	float y_min{ 0.f };
	float z_min{ 0.f };
	float x_max{ 0.f };
	float y_max{ 0.f };
	float z_max{ 0.f };
};

#endif /*BOX_H*/