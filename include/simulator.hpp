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

class CatchABallSimulator
{
public:
    CatchABallSimulator(const std::string &configFpath);
    ~CatchABallSimulator();

    cv::Mat getScene();

    void tick();
    bool is_end();

    double get_sim_time();

    int32_t getShotIdx();
    cv::Mat get_control_frame();

    void setPlaneControl(int32_t);
    int32_t getPlaneDistancePx();

    void shootBall();

private:
    cv::Mat m_control_frame;
    double  m_last_frame_tick;

    cv::Rect m_canvas_rect;

    cv::Rect m_control_rect;

    /* Main objects */
    std::shared_ptr<Cannon>     m_cannon;
    std::shared_ptr<Ball>       m_ball;
    std::shared_ptr<CatchPlane> m_plane;


    int32_t     m_catchPlaneRefPx;
    double      m_catchPlaneRef;

    double      m_ballRadius;

    double      m_initialAngle;

    uint32_t    m_shotCounter;
    uint32_t    m_catchCounter;

    bool m_isRenderEnabled;

    cv::Point2d m_scaleM2Px;

    std::atomic<unsigned long> m_current_tick;
    double  m_sim_dt;

    bool    m_is_end;

    bool    m_isBallFlying;

    std::shared_ptr<Scene>   m_scene;

    void resetShot();
};

} // namespace sim
