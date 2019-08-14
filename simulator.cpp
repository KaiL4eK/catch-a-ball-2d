#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

#include "simulator.hpp"

using namespace cv;
using namespace std;
using namespace sim;

const double GRAVITY_CONST = 9.81;

/* Configurable parameters */

const double BALL_RADIUS_M              = 0.2;
const double BALL_INITIAL_SPEED_MPS     = 10;
const double RANDOM_CANNON_DEG_CHANGE   = 15;
const double M_2_PX                     = 100;
const Scalar ball_color(0, 255, 0); // BGR
const Scalar cannon_color(255, 0, 0); // BGR

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
    return px / M_2_PX;
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

const string CatchABallSimulator::CANVAS_NAME = "canvas";

CatchABallSimulator::CatchABallSimulator(Size size, double sim_dt) : 
    m_canvas(size, CV_8UC3), m_sim_dt(sim_dt),
    m_cannon(Point2d(0, px_2_meters(size.height / 2)), Size2d(0.4, 0.3)),
    m_canvas_rect(Point(0, 0), size),
    m_lock_ticks(100), m_is_end(false),
    m_current_tick(0),
    m_control_rect(m_canvas_rect.width/3, 0, m_canvas_rect.width/3, m_canvas_rect.height)
{   
    m_cannon.set_angle(30);

    shared_ptr<Axes> field_axes = make_shared<Axes>(Point2d(0, 0));
    m_objects.push_back(dynamic_pointer_cast<Object>(field_axes));

    
}

CatchABallSimulator::~CatchABallSimulator()
{
}

void CatchABallSimulator::show_scene()
{
    flip(m_canvas, m_canvas, 0);
    imshow(CANVAS_NAME, m_canvas);
    waitKey(1);
}

void CatchABallSimulator::tick()
{
    m_current_tick++;

    for (auto obj : m_objects) {
        obj->move(m_sim_dt);
    }

    if (!m_ball && m_lock_ticks == 0) {
        m_ball = m_cannon.shoot();
    } else {
        m_lock_ticks--;
    }
    
    if (m_ball){
        m_ball->move(m_sim_dt);

        Point ball_px_position = meters_2_px(m_ball->get_position());
        if ( ball_px_position.x > m_canvas_rect.width ||
             ball_px_position.y < 0 )
        {
            m_ball.reset();
            // m_is_end = true;

            m_cannon.reset_ball();

            double random_degree_change = 
                rand() % static_cast<int>(RANDOM_CANNON_DEG_CHANGE * 2);
            random_degree_change -= RANDOM_CANNON_DEG_CHANGE;

            cout << "Change degree for " << random_degree_change << " degree" << endl;


            m_cannon.set_angle(m_cannon.get_angle() + random_degree_change);    
            
            m_lock_ticks = 100;
        }
    }

    // After movement render positions
    m_canvas = Scalar(255, 255, 255);

    for (auto obj : m_objects)
    {
        obj->render(m_canvas);
    }

    m_cannon.render(m_canvas);
    
    if (m_ball)
    {
        m_ball->render(m_canvas);
    }
}

Mat CatchABallSimulator::get_control_frame()
{


    return m_canvas(m_control_rect).clone();
}

bool CatchABallSimulator::is_end()
{
    return m_is_end;
}

double CatchABallSimulator::get_sim_time()
{
    return m_current_tick * m_sim_dt;
}
