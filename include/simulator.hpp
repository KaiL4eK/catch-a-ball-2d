#pragma once

#include <string>
#include <chrono>
#include <vector>
#include <memory>
#include <atomic>

#include <opencv2/core/core.hpp>

#include "objects.hpp"

namespace sim
{

class Scene
{
public:
    Scene(cv::Size canvasSzPx, 
            cv::Size2d canvasSzMtr);

    void render(std::shared_ptr<Ball> obj);
    void render(std::shared_ptr<CatchPlane> obj);
    void render(std::shared_ptr<Cannon> obj);

    void cleanCanvas();
    cv::Mat getCanvas();

    cv::Point   m2px(cv::Point2d m);
    cv::Size    m2px(cv::Size2d m);

    cv::Point2d px2m(cv::Point px);
    cv::Size2d  px2m(cv::Size px);

    bool contains(std::shared_ptr<Ball> obj);

private:
    cv::Point2d  m_m2pxScale;

    cv::Mat m_canvas;
    cv::Rect m_canvasRect;
    
    cv::Size m_canvasSzPx;
    cv::Size m_canvasSzMtr;
};

struct CatchABallStatistics
{
    uint32_t ballsCatched;
    uint32_t shotsDone;
    uint32_t flewAway;

    CatchABallStatistics() :
        ballsCatched(0),
        shotsDone(0),
        flewAway(0)
    {}
};

class CatchABallSimulator
{
public:
    CatchABallSimulator(const std::string &configFpath);
    ~CatchABallSimulator();

    cv::Mat     getScene();
    cv::Mat     getControlFrame();

    void        tick();
    bool        isEnd();

    void        setAutoShootingMode(bool enabled);

    void        setPlaneControl(int32_t);
    int32_t     getPlaneDistancePx();

    void        shootBall(double shotAngleDeg, 
                          double ballSpeedMPS);
    void        randomizedShootBall();

    CatchABallStatistics getStatistics();

private:
    cv::Rect m_canvas_rect;
    cv::Rect m_control_rect;

    std::shared_ptr<Scene>   m_scene;

    /* Main objects */
    std::shared_ptr<Cannon>     m_cannon;
    std::shared_ptr<Ball>       m_ball;
    std::shared_ptr<CatchPlane> m_plane;

    double      m_ballRadius;
    double      m_shotInitialAngle;

    /*  [0] - lower limit
        [1] - upper limit */
    std::vector<double> m_speedLimitsMPS;

    CatchABallStatistics m_stats;
    std::atomic<uint32_t> m_current_tick;
    double      m_sim_dt;

    bool        m_isEnd;
    bool        m_isBallFlying;
    bool        m_isAutoShotsEnabled;


    void resetShot();
    void renderStatistics();
};

} // namespace sim
