#include <string>
#include <chrono>
#include <vector>
#include <memory>
#include <atomic>

#include <opencv2/core/core.hpp>

namespace sim
{

class Object
{
public:
    Object(cv::Point2d initial_pos,
           cv::Point2d initial_speed);
    virtual ~Object() = default;

    virtual void move(double dt);
    virtual void render(cv::Mat &canvas) = 0;

    cv::Point2d get_position() { return m_pos; }
    cv::Point2d get_speed() { return m_speed; }

private:
    cv::Point2d m_speed;
    cv::Point2d m_pos;
};

class Ball : public Object
{
public:
    Ball(cv::Point2d initial_pos,
         cv::Point2d initial_speed,
         double radius);

    void render(cv::Mat &canvas) override;

private:
    const double m_radius;
};

class Cannon
{
public:
    Cannon(cv::Point2d pos, cv::Size2d size);

    void render(cv::Mat &canvas);
    void set_angle(double angle_deg);
    double get_angle();
    std::shared_ptr<Ball> shoot();
    void reset_ball();

private:
    cv::Point2d get_ball_position();

    bool        m_is_ball_flying;
    double      m_angle_deg;
    double      m_angle_rad;
    cv::Point2d m_pos;
    cv::Size2d  m_size;
};

class Axes : public Object
{
public:
    Axes(cv::Point2d initial_pos);

    void move(double dt) override;
    void render(cv::Mat &canvas) override;
};

class CatchABallSimulator
{
public:
    CatchABallSimulator(cv::Size size, double sim_dt = 0.001);
    ~CatchABallSimulator();

    void show_scene();
    void tick();
    bool is_end();

    double get_sim_time();

    cv::Mat get_control_frame();

private:
    cv::Mat m_control_frame;
    double  m_last_frame_tick;

    cv::Mat m_canvas;
    cv::Rect m_canvas_rect;

    cv::Rect m_control_rect;

    std::vector<std::shared_ptr<Object>> m_objects;

    Cannon                  m_cannon;
    std::shared_ptr<Ball>   m_ball;

    int32_t m_lock_ticks;

    const static std::string CANVAS_NAME;

    std::atomic<unsigned long> m_current_tick;
    double  m_sim_dt;

    bool    m_is_end;
};

} // namespace sim
