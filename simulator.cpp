#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

#include "simulator.hpp"

using namespace cv;
using namespace std;
using namespace sim;

const double GRAVITY_CONST = 9.81;

/* Configurable parameters */

const double BALL_RADIUS_M              = 0.1;
const double BALL_INITIAL_SPEED_MPS     = 20;
const double CATCH_PLANE_WIDTH_M        = 0.04;
const double RANDOM_CANNON_DEG_CHANGE   = 15;
const double M_2_PX                     = 100;
/* BGR objects color */
const Scalar ball_color(0, 0, 255);
const Scalar cannon_color(255, 0, 0);
const Scalar catchPlaneColor(0, 0, 0);

inline int meters_2_px(double m)
{
    return M_2_PX * m;
}

inline Point2i meters_2_px(Point2d m)
{
    return M_2_PX * m;
}

inline Size2i meters_2_px(Size2d m)
{
    return Size2i(M_2_PX * m.width, M_2_PX * m.height);
}

inline double px_2_meters(int px)
{
    return px / M_2_PX;
}

inline Point2d px_2_meters(Point2i px)
{
    return static_cast<Point2d>(px) / M_2_PX;
}

Object::Object(Point2d initial_pos, Point2d initial_speed) : 
    m_pos(initial_pos), m_speed(initial_speed)
{
}

void Object::move(double dt)
{
    m_speed += dt * Point2d(0, -GRAVITY_CONST);
    m_pos += dt * m_speed;
}

Ball::Ball(Point2d initial_pos, Point2d initial_speed, double radius) : 
    Object(initial_pos, initial_speed), m_radius(radius)
{
}

void Ball::render(Mat &canvas)
{
    circle(canvas,
           meters_2_px(get_position()),
           meters_2_px(m_radius),
           ball_color,
           FILLED);
}


Cannon::Cannon(Point2d pos, Size2d size) :
    m_pos(pos), m_size(size), m_angle_deg(0), m_angle_rad(0),
    m_is_ball_flying(false)
{
}

Point2d Cannon::get_ball_position()
{
    return Point2d (m_pos.x + cos(m_angle_rad) * m_size.height,
                    m_pos.y + sin(m_angle_rad) * m_size.height);
}

void Cannon::render(cv::Mat &canvas)
{
    /* Render ball first to be background */

    if (!m_is_ball_flying)
    {
        circle(canvas,
                meters_2_px(get_ball_position()),
                meters_2_px(BALL_RADIUS_M),
                ball_color,
                FILLED);
    }
    
    /* Now render rectangle */

    RotatedRect rotatedRectangle(
        meters_2_px(m_pos), 
        // Increase twice just to make long enough
        // As second part is out of canvas
        meters_2_px(Size2d(m_size.width, m_size.height*2)), 
        m_angle_deg + 90);

    Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    fillConvexPoly(canvas, vertices, 4, cannon_color);
}

void Cannon::set_angle(double angle_deg)
{
    if (angle_deg > 70)
        angle_deg = 70;

    if (angle_deg < -70)
        angle_deg = -70;

    m_angle_deg = angle_deg;
    m_angle_rad = angle_deg * M_PI/180;
}

double Cannon::get_angle()
{
    return m_angle_deg;
}

void Cannon::reset_ball()
{
    m_is_ball_flying = false;
}

shared_ptr<Ball> Cannon::shoot()
{
    if (m_is_ball_flying)
        throw logic_error("No ball in cannon");

    Point2d ball_speed(cos(m_angle_rad) * BALL_INITIAL_SPEED_MPS,
                       sin(m_angle_rad) * BALL_INITIAL_SPEED_MPS);

    shared_ptr<Ball> new_ball = make_shared<Ball>(
        get_ball_position(),
        ball_speed,
        BALL_RADIUS_M
    );

    m_is_ball_flying = true;

    return new_ball;
}

Axes::Axes(Point2d initial_pos) : 
    Object(initial_pos, Point(0, 0))
{
}

void Axes::move(double dt) 
{

}

void Axes::render(Mat &canvas)
{
    arrowedLine(canvas,
                meters_2_px(get_position()),
                meters_2_px(get_position() + Point2d(1, 0)),
                Scalar(255, 0, 0),
                3
    );

    arrowedLine(canvas,
                meters_2_px(get_position()),
                meters_2_px(get_position() + Point2d(0, 1)),
                Scalar(0, 255, 0),
                3
    );
}

CatchPlane::CatchPlane(Point2d initial_pos, Size2d size) : 
    Object(initial_pos, Point(0, 0)), m_size(size)
{
}

void CatchPlane::render(cv::Mat &canvas)
{
    Point pxPos = meters_2_px(get_position());
    Size pxSize = meters_2_px(m_size);

    /* Shift as Object (x,y) is center of object */
    pxPos -= Point(pxSize.width/2, pxSize.height/2);

    Rect desc( pxPos, pxSize );
    rectangle( canvas, desc, catchPlaneColor, FILLED );
}

void CatchPlane::setRefPosition(double yMeter)
{
    setPosition(Point2d(get_position().x, yMeter));
}

bool CatchPlane::isBallCaught(shared_ptr<Ball> p_ball)
{
    /* Just check intersection */
    Point2d rectPosMeter = get_position();

    double rectNearestX = max(rectPosMeter.x - m_size.width/2,
                                min(rectPosMeter.x + m_size.width/2, 
                                    p_ball->get_position().x));

    double rectNearestY = max(rectPosMeter.y - m_size.height/2,
                                min(rectPosMeter.y + m_size.height/2, 
                                    p_ball->get_position().y));
    
    double deltaX = p_ball->get_position().x - rectNearestX;
    double deltaY = p_ball->get_position().y - rectNearestY;

    return (deltaX * deltaX + deltaY * deltaY) < 
            (p_ball->getRadius() * p_ball->getRadius());
}


CatchABallSimulator::CatchABallSimulator(Size size, double sim_dt) : 
    m_canvas(size, CV_8UC3), m_sim_dt(sim_dt),
    m_cannon(Point2d(0, px_2_meters(size.height / 2)), Size2d(0.4, 0.3)),
    m_canvas_rect(Point(0, 0), size),
    m_lock_ticks(20), m_is_end(false),
    m_current_tick(0),
    m_shotCounter(0), m_catchCounter(0), m_catchPlaneRef(0),
    m_control_rect(m_canvas_rect.width/3, 0, m_canvas_rect.width/3, m_canvas_rect.height)
{   
    m_cannon.set_angle(10);

    shared_ptr<Axes> field_axes = make_shared<Axes>(Point2d(0, 0));
    m_objects.push_back(dynamic_pointer_cast<Object>(field_axes));

    int planeWidthPx = meters_2_px(CATCH_PLANE_WIDTH_M);
    int xPlanePosPx = m_canvas_rect.width - planeWidthPx * 2;

    m_catchPlaneRefPx = m_canvas_rect.height/2;

    m_plane = make_shared<CatchPlane>(
        px_2_meters(Point(xPlanePosPx, m_canvas.size().height/2)),
        Size2d(CATCH_PLANE_WIDTH_M, BALL_RADIUS_M*2)
    );

    srand(time(NULL));
}

CatchABallSimulator::~CatchABallSimulator()
{
}

Mat CatchABallSimulator::getScene()
{
    Mat copy = m_canvas.clone();

    return copy;
}

void CatchABallSimulator::resetShot()
{
    m_ball.reset();
    m_cannon.reset_ball();

    double random_degree_change = 
        rand() % static_cast<int>(RANDOM_CANNON_DEG_CHANGE * 2);
    random_degree_change -= RANDOM_CANNON_DEG_CHANGE;

    cout << "Change degree for " << random_degree_change << " degree" << endl;

    m_cannon.set_angle(m_cannon.get_angle() + random_degree_change);    
    
    m_lock_ticks = 20;
}

void CatchABallSimulator::tick()
{
    m_current_tick++;

    for (auto obj : m_objects) {
        obj->move(m_sim_dt);
    }

    if (!m_ball && m_lock_ticks == 0) {
        m_ball = m_cannon.shoot();
        m_shotCounter++;
    } else {
        m_lock_ticks--;
    }
    
    if (m_ball){
        m_ball->move(m_sim_dt);

        /* Exit condition */
        Point ball_px_position = meters_2_px(m_ball->get_position());
        if ( ball_px_position.x > m_canvas_rect.width ||
             ball_px_position.y < 0 ||
             ball_px_position.y > m_canvas_rect.height )
        {
            resetShot();
        }
        /* Catch condition */
        else if ( m_plane->isBallCaught(m_ball) )
        {
            m_catchCounter++;
            resetShot();
        }
    }

    /* After movement render positions */
    m_canvas = Scalar(255, 255, 255);

    for (auto obj : m_objects)
    {
        obj->render(m_canvas);
    }

    m_cannon.render(m_canvas);
    m_plane->render(m_canvas);
    
    if (m_ball)
    {
        m_ball->render(m_canvas);
    }

    flip(m_canvas, m_canvas, 0);
}

Mat CatchABallSimulator::get_control_frame()
{
    return m_canvas(m_control_rect).clone();
}

bool CatchABallSimulator::is_end()
{
    return m_is_end;
}

void CatchABallSimulator::setPlaneControl(int frameYPx)
{
    m_catchPlaneRefPx = frameYPx > m_canvas_rect.height ? m_canvas_rect.height:
                        frameYPx < 0 ? 0 : frameYPx;

    m_plane->setRefPosition( px_2_meters(m_canvas_rect.height-m_catchPlaneRefPx) );
}

int CatchABallSimulator::getPlaneDistancePx()
{
    return meters_2_px(m_plane->get_position().x) - m_control_rect.x;
}

double CatchABallSimulator::get_sim_time()
{
    return m_current_tick * m_sim_dt;
}

int32_t CatchABallSimulator::getShotIdx()
{
    return m_shotCounter;
}
