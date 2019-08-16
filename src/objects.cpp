#include "objects.hpp"

using namespace cv;
using namespace std;
using namespace sim;

ObjectDef::ObjectDef() :
    colorBGR(Scalar(0)), 
    position(Point2d(0, 0)),
    speed(Point2d(0, 0)),
    speedLimit(Point2d(0, 0)),
    gravity(0),
    rotationDeg(0)
{
}


Object::Object(ObjectDef &def) : 
    m_pos(def.position), 
    m_speed(def.speed), 
    m_colorBGR(def.colorBGR),
    m_angleDeg(def.rotationDeg),
    m_def(def)
{
}

void Object::move(double dt)
{
    if ( m_def.gravity != 0 )
        m_speed += dt * Point2d(0, -m_def.gravity);
    
    m_pos += dt * m_speed;
}

void Object::setAngle(double angleDeg)
{
    /* Normalize angle to 360 */
    angleDeg = fmod(angleDeg,360);
    if (angleDeg < 0)
        angleDeg += 360;

    m_angleDeg = angleDeg;
}

void Object::setSpeed(cv::Point2d speed)
{
    m_speed = speed;
}

Ball::Ball(ObjectDef &def, double radius) : 
    Object(def), m_radius(radius)
{
}


Cannon::Cannon(ObjectDef &def, ObjectDef &ballDef, cv::Size2d size) :
    Object(def), m_size(size),
    m_ballDef(ballDef)
{
}

void Cannon::setShotAngle(double angleDeg)
{
    angleDeg = angleDeg > 70 ? 70 :
                angleDeg < -70 ? -70 : angleDeg;
    
    setAngle(angleDeg);
}

shared_ptr<Ball> Cannon::shoot(double ballInitialSpeedMPS, 
                               double ballRadiusMtr)
{
    double angleRad = getRotationRad();

    m_ballDef.position = getPosition();
    m_ballDef.speed = Point2d(cos(angleRad), sin(angleRad)) * 
                        ballInitialSpeedMPS;

    shared_ptr<Ball> new_ball = make_shared<Ball>(
        m_ballDef,
        ballRadiusMtr
    );

    return new_ball;
}


CatchPlane::CatchPlane(ObjectDef &def, Size2d size) : 
    Object(def), m_size(size),
    m_refPosY(def.position.y)
{
}

void CatchPlane::move(double dt)
{
    double speedY = (m_refPosY - getPosition().y)/dt;

    double speedLimitY = getSpeedLimit().y;

    speedY = speedY > speedLimitY ? speedLimitY :
                speedY < -speedLimitY ? -speedLimitY : speedY; 

    setSpeed( Point2d(0, speedY) );

    Object::move(dt);
}

void CatchPlane::setRefPosition(double refPosY)
{
    m_refPosY = refPosY;
}

bool CatchPlane::isBallCaught(shared_ptr<Ball> p_ball)
{
    /* Just check intersection */
    Point2d rectPosMeter = getPosition();

    double rectNearestX = max(rectPosMeter.x - m_size.width/2,
                                min(rectPosMeter.x + m_size.width/2, 
                                    p_ball->getPosition().x));

    double rectNearestY = max(rectPosMeter.y - m_size.height/2,
                                min(rectPosMeter.y + m_size.height/2, 
                                    p_ball->getPosition().y));
    
    double deltaX = p_ball->getPosition().x - rectNearestX;
    double deltaY = p_ball->getPosition().y - rectNearestY;

    return (deltaX * deltaX + deltaY * deltaY) < 
            (p_ball->getRadius() * p_ball->getRadius());
}

