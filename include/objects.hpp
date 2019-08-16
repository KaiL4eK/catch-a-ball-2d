#pragma once

#include <string>
#include <chrono>
#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>

namespace sim
{

const double GRAVITY_CONST = 9.81;


struct ObjectDef
{
    cv::Point2d position;
    cv::Point2d speed;
    cv::Scalar  colorBGR;
    double      gravity;
    double      rotationDeg;

    ObjectDef();
};

class Object
{
public:
    Object(ObjectDef &def);
    virtual ~Object() = default;

    virtual void move(double dt);

    cv::Scalar getColor() { return m_colorBGR; }
    cv::Point2d getPosition() { return m_pos; }
    cv::Point2d get_speed() { return m_speed; }
    double getRotationDeg() { return m_angleDeg; }
    double getRotationRad() { return m_angleDeg * M_PI/180; }

private:
    ObjectDef   m_def;

    cv::Point2d m_speed;
    cv::Point2d m_pos;
    cv::Scalar  m_colorBGR;
    double      m_angleDeg;
};

class Ball : public Object
{
public:
    Ball(ObjectDef &def, double radius);

    double getRadius() { return m_radius; }

private:
    const double m_radius;
};

class CatchPlane : public Object
{
public:
    CatchPlane(ObjectDef &def, cv::Size2d size);

    cv::Size2d getSize() { return m_size; } 

    void setRefPosition(double yMeter);

    bool isBallCaught(std::shared_ptr<Ball> p_ball);

private:
    const cv::Size2d m_size;
};

class Cannon : public Object
{
public:
    Cannon(ObjectDef &def, ObjectDef &ballDef, cv::Size2d size);

    cv::Size2d getSize() { return m_size; } 

    void set_angle(double angleDeg);
    std::shared_ptr<Ball> shoot(double ballInitialSpeedMPS, 
                                double ballRadiusMtr);

private:
    ObjectDef   m_ballDef;

    cv::Size2d  m_size;
};


} // namespace sim
