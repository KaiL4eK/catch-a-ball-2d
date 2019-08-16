#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

#include "simulator.hpp"

using namespace cv;
using namespace std;
using namespace sim;


/* Configurable parameters */
const double BALL_INITIAL_SPEED_MPS     = 20;
const double RANDOM_CANNON_DEG_CHANGE   = 15;
const double GRAVITY_CONST = 9.81;

Scene::Scene(Size canvasSzPx, 
             Size2d canvasSzMtr) :
    m_canvasSzPx(canvasSzPx), m_canvasSzMtr(canvasSzMtr),
    m_canvas(canvasSzPx, CV_8UC3)
{
    m_canvasRect = Rect(Point(0, 0), canvasSzPx),

    m_m2pxScale = Point2d(canvasSzPx.width/canvasSzMtr.width,
                          canvasSzPx.height/canvasSzMtr.height);
}

Point Scene::m2px(Point2d m)
{
    return Point(m_m2pxScale.x * m.x,
                 m_m2pxScale.y * m.y);
}

Point2d Scene::px2m(Point px)
{
    return Point2d(px.x / m_m2pxScale.x,
                   px.y / m_m2pxScale.x);
}

Size Scene::m2px(Size2d m)
{
    return Size(m_m2pxScale.x * m.width,
                m_m2pxScale.y * m.height);
}

Size2d Scene::px2m(Size px)
{
    return Size2d(px.width / m_m2pxScale.x,
                  px.height / m_m2pxScale.y);
}

void Scene::render(shared_ptr<Ball> obj)
{   
    double ballRadius = obj->getRadius();

    ellipse(m_canvas,
            m2px(obj->getPosition()),
            m2px(Point2d(ballRadius, ballRadius)),
            0, 0, 360,
            obj->getColor(),
            FILLED);
}

void Scene::render(std::shared_ptr<CatchPlane> obj)
{
    Point pxPos = m2px(obj->getPosition());
    Size pxSize = m2px(obj->getSize());

    /* Shift as Object (x,y) is center of object */
    pxPos -= Point(pxSize.width/2, pxSize.height/2);

    Rect desc( pxPos, pxSize );
    rectangle( m_canvas, desc, obj->getColor(), FILLED );
}

void Scene::render(std::shared_ptr<Cannon> obj)
{
    Size2d size = obj->getSize();

    RotatedRect rotatedRectangle(
        m2px(obj->getPosition()), 
        // Increase twice just to make long enough
        // As second part is out of canvas
        m2px(Size2d(size.width, size.height*2)), 
        obj->getRotationDeg() + 90);

    Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    fillConvexPoly(m_canvas, vertices, 4, obj->getColor());
}

Mat Scene::getCanvas()
{
    Mat flipped;
    flip(m_canvas, flipped, 0);

    return flipped;
}

void Scene::cleanCanvas()
{
    m_canvas = Scalar(255, 255, 255);
}

bool Scene::contains(shared_ptr<Ball> obj)
{
    if ( !obj )
        return false;

    Point posPx = m2px(obj->getPosition());
    return m_canvasRect.contains(posPx);
}

double getBallBalisticAngleDeg( Point2d desiredPos )
{
    double S = BALL_INITIAL_SPEED_MPS;
    double S2 = S*S;
    
    double G = GRAVITY_CONST;
    double x = desiredPos.x;
    double y = desiredPos.y;

    cout << S << " / " << G << " / " << x << " / " << y << endl;

    double result = atan2( S2 * sqrt(S2*S2-G*(G*x*x+2*S2*y)), G*x ) * 180/M_PI;
}

CatchABallSimulator::CatchABallSimulator(const string &configFpath) : 
    m_is_end(false), 
    m_current_tick(0), 
    m_isBallFlying(false),
    m_shotCounter(0), 
    m_catchCounter(0),
    /* Defaults */
    m_ballRadius(0.1)
{   
    FileStorage cfgFs(configFpath, FileStorage::READ);
    if ( !cfgFs.isOpened() )
        throw invalid_argument("Failed to open config file");

    std::vector<int> v_canvasSzPx;
    std::vector<int> v_canvasSzMtr;

    cfgFs["canvasSizePixel"] >> v_canvasSzPx;
    cfgFs["canvasSizeMeter"] >> v_canvasSzMtr;

    Size    canvasSzPx (v_canvasSzPx[0], v_canvasSzPx[0]);
    Size2d  canvasSzMtr(v_canvasSzMtr[0], v_canvasSzMtr[0]);

    cfgFs["dt"] >> m_sim_dt;
    cfgFs["debugRender"] >> m_isRenderEnabled;
    cfgFs["ballRadiusMtr"] >> m_ballRadius;

    if ( m_isRenderEnabled )
        cout << "Debug Render enabled" << endl; 

    m_canvas_rect = Rect(Point(0, 0), canvasSzPx),
    m_control_rect = Rect(Point(canvasSzPx.width/3, 0), 
                            Size(canvasSzPx.width/3, canvasSzPx.height)),

    /* Create scene */

    m_scene = make_shared<Scene>(canvasSzPx, canvasSzMtr);

    /* Create Cannon */

    ObjectDef cannonDef;
    cannonDef.position = Point2d(0, canvasSzMtr.height/3);
    cannonDef.colorBGR = Scalar(255, 0, 0);  // Always blue!

    ObjectDef ballDef;
    ballDef.colorBGR = Scalar(0, 0, 255);  // Always red - don`t confuse CS =);
    ballDef.gravity = GRAVITY_CONST;

    Size2d cannonSz(m_ballRadius, m_ballRadius*3);
    m_cannon = make_shared<Cannon>(cannonDef, ballDef, cannonSz);

    /* Create catch plane */

    Size2d catchPlaneSz(m_ballRadius/2, m_ballRadius*2);
    ObjectDef catchPlaneDef;
    // Make a small shift from right border
    catchPlaneDef.position = Point2d(canvasSzMtr.width - catchPlaneSz.width, 
                                     canvasSzMtr.height/2);
    catchPlaneDef.colorBGR = Scalar(0, 0, 0);   // Always black!
    catchPlaneDef.speedLimit = Point2d( 0, 4*BALL_INITIAL_SPEED_MPS );

    m_plane = make_shared<CatchPlane>(catchPlaneDef, catchPlaneSz);

    /* True randomization =) */
    srand(time(NULL));

    /* Calculate initial angle */
    // Start Plane position and desired parameters are same
    double angle = getBallBalisticAngleDeg( catchPlaneDef.position - cannonDef.position );
    cout << angle << endl;

    m_initialAngle = 20;

    m_cannon->setShotAngle(m_initialAngle);
}

CatchABallSimulator::~CatchABallSimulator()
{
}

Mat CatchABallSimulator::getScene()
{
    return m_scene->getCanvas().clone();
}

void CatchABallSimulator::resetShot()
{
    m_ball.reset();
    m_isBallFlying = false;

    double random_degree_change = 
        rand() % static_cast<int>(RANDOM_CANNON_DEG_CHANGE * 2);
    random_degree_change -= RANDOM_CANNON_DEG_CHANGE;

    cout << "Change degree for " << random_degree_change << " degree" << endl;

    m_cannon->setShotAngle(m_initialAngle + random_degree_change);
}

void CatchABallSimulator::shootBall()
{
    if (!m_isBallFlying) {
        m_ball = m_cannon->shoot(BALL_INITIAL_SPEED_MPS, m_ballRadius);
        m_isBallFlying = true;
        m_shotCounter++;
    }
}

void CatchABallSimulator::tick()
{
    m_current_tick++;
    
    if (m_isBallFlying && m_ball){
        m_ball->move(m_sim_dt);

        /* Outline condition */
        if ( !m_scene->contains(m_ball) )
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

    if ( !m_isBallFlying )
    {
        shootBall();
    }

    m_plane->move(m_sim_dt);
    // Cannon movement not required

    /* After movement render positions */
    m_scene->cleanCanvas();

    m_scene->render(m_cannon);
    m_scene->render(m_plane);

    if (m_ball)
        m_scene->render(m_ball);

    // rectangle(m_canvas, m_control_frame, Scalar(0));
}

Mat CatchABallSimulator::get_control_frame()
{
    return m_scene->getCanvas()(m_control_rect).clone();
}

bool CatchABallSimulator::is_end()
{
    return m_is_end;
}

void CatchABallSimulator::setPlaneControl(int frameYPx)
{
    m_catchPlaneRefPx = frameYPx > m_canvas_rect.height ? m_canvas_rect.height:
                        frameYPx < 0 ? 0 : frameYPx;

    Point2d refPos = m_scene->px2m(Point(0, m_canvas_rect.height-m_catchPlaneRefPx));

    m_plane->setRefPosition( refPos.y );
}

int CatchABallSimulator::getPlaneDistancePx()
{
    return m_scene->m2px( m_plane->getPosition() ).x - m_control_rect.x;
}

double CatchABallSimulator::get_sim_time()
{
    return m_current_tick * m_sim_dt;
}

int32_t CatchABallSimulator::getShotIdx()
{
    return m_shotCounter;
}
