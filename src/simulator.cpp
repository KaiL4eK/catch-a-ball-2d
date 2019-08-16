#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

#include "simulator.hpp"

using namespace cv;
using namespace std;
using namespace sim;

/* Configurable parameters */
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
    Point result = Point(m_m2pxScale.x * m.x,
                         m_m2pxScale.y * m.y);

    result.y = m_canvasRect.height - result.y;

    return result;
}

Point2d Scene::px2m(Point px)
{
    px.y = m_canvasRect.height - px.y;

    return Point2d(px.x / m_m2pxScale.x,
                   px.y / m_m2pxScale.y);
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
            m2px(Size2d(ballRadius, ballRadius)),
            0, 0, 360,
            obj->getColor(),
            FILLED);
}

void Scene::render(shared_ptr<CatchPlane> obj)
{
    Point pxPos = m2px(obj->getPosition());
    Size pxSize = m2px(obj->getSize());

    /* Shift as Object (x,y) is center of object */
    pxPos -= Point(pxSize.width/2, pxSize.height/2);

    Rect desc( pxPos, pxSize );
    rectangle( m_canvas, desc, obj->getColor(), FILLED );
}

void Scene::render(shared_ptr<Cannon> obj)
{
    Size2d size = obj->getSize();

    RotatedRect rotatedRectangle(
        m2px(obj->getPosition()), 
        // Increase twice just to make long enough
        // As second part is out of canvas
        m2px(Size2d(size.width, size.height*2)), 
        90-obj->getRotationDeg());

    Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    fillConvexPoly(m_canvas, vertices, 4, obj->getColor());
}

void Scene::renderControlZone(Rect &controlZone)
{
    rectangle(m_canvas, controlZone, Scalar(50, 50, 50));
}

Rect Scene::getRect()
{
    return m_canvasRect;
}

Mat Scene::getCanvas()
{
    return m_canvas;
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

static double getBallBalisticAngleDeg( Point2d desiredPos, double initialSpeed )
{
    double S = initialSpeed;
    double S2 = S*S;
    double S4 = S2*S2;
    
    double G = GRAVITY_CONST;
    double x = desiredPos.x;
    double y = desiredPos.y;

    double result = atan2( S2 - sqrt(S4-G*(G*x*x+2*S2*y)), G*x ) * 180/M_PI;
}

static double getRandomInRange( double min, double max )
{
    return min + (rand() % static_cast<int>(max - min + 1));
}

CatchABallSimulator::CatchABallSimulator(const string &configFpath) : 
    m_current_tick(0), 
    /* States */
    m_isEnd(false), 
    m_isBallFlying(false),
    m_isAutoShotsEnabled(false),
    /* Defaults */
    m_ballRadius(0.1),
    m_shotInitialAngle(16),
    m_sim_dt(0.001)
{   
    /* Parsing config */

    FileStorage cfgFs(configFpath, FileStorage::READ);
    if ( !cfgFs.isOpened() )
        throw invalid_argument("Failed to open config file");

    std::vector<int> v_canvasSzPx;
    std::vector<int> v_canvasSzMtr;

    cfgFs["canvasSizePixel"] >> v_canvasSzPx;
    cfgFs["canvasSizeMeter"] >> v_canvasSzMtr;

    assert(v_canvasSzPx.size() >= 2);
    assert(v_canvasSzMtr.size() >= 2);

    Size    canvasSzPx (v_canvasSzPx[0], v_canvasSzPx[1]);
    Size2d  canvasSzMtr(v_canvasSzMtr[0], v_canvasSzMtr[1]);

    if ( !cfgFs["dt"].empty() )
        cfgFs["dt"] >> m_sim_dt;

    if ( !cfgFs["ballRadiusMtr"].empty() )
        cfgFs["ballRadiusMtr"] >> m_ballRadius;

    if ( !cfgFs["ballShotSpeedLimitsMPS"].empty() )
        cfgFs["ballShotSpeedLimitsMPS"] >> m_speedLimitsMPS;

    assert(m_speedLimitsMPS.size() >= 2);
    assert(m_sim_dt > 0);
    assert(m_ballRadius > 0);

    /* Service variables */

    m_controlRect = Rect(Point(canvasSzPx.width/3, 0), 
                         Size(canvasSzPx.width/3, canvasSzPx.height)),

    /* Create scene */

    m_scene = make_shared<Scene>(canvasSzPx, canvasSzMtr);

    /* Create Cannon */

    ObjectDef cannonDef;
    cannonDef.position = Point2d(0, canvasSzMtr.height/3);
    cannonDef.colorBGR = Scalar(255, 0, 0);  // Always blue!
    cannonDef.rotationDeg = m_shotInitialAngle;

    ObjectDef ballDef;
    ballDef.colorBGR = Scalar(0, 0, 255);  // Always red - don`t confuse CS =);
    ballDef.gravity = GRAVITY_CONST;

    Size2d cannonSz(m_ballRadius, m_ballRadius*3);
    m_cannon = make_shared<Cannon>(cannonDef, ballDef, cannonSz);

    double testSpeed = (m_speedLimitsMPS[0] + m_speedLimitsMPS[1]) / 2;

    /* Calculate initial angle for cannon */
    // Start Plane position and desired parameters are same
    double angle = getBallBalisticAngleDeg(
                    Point2d(canvasSzMtr.width, canvasSzMtr.height/2)-cannonDef.position,
                    testSpeed );
    
    if ( isnan(angle) )
    {
        throw invalid_argument("Too low value of shooting speed. "
                                "Set higher value of 'ballShotSpeedLimitsMPS'");
    }

    cout << "Base angle: " << angle << " [deg] for speed " << testSpeed << " [m/s]" << endl;

    m_shotInitialAngle = angle;
    m_cannon->setShotAngle(m_shotInitialAngle);

    /* Create catch plane */

    Size2d catchPlaneSz(m_ballRadius/2, m_ballRadius*2);
    ObjectDef catchPlaneDef;
    // Make a small shift from right border
    catchPlaneDef.position = Point2d(canvasSzMtr.width - catchPlaneSz.width * 3, 
                                     canvasSzMtr.height/2);
    catchPlaneDef.colorBGR = Scalar(0, 0, 0);   // Always black!
    catchPlaneDef.speedLimit = Point2d( 0, 4*m_speedLimitsMPS[1] );

    m_plane = make_shared<CatchPlane>(catchPlaneDef, catchPlaneSz);

    /* True randomization =) */
    srand(time(NULL));
}

CatchABallSimulator::~CatchABallSimulator()
{
}


Mat CatchABallSimulator::getScene()
{
    Mat scene = m_scene->getCanvas().clone();
    return scene;
}

Mat CatchABallSimulator::getControlFrame()
{
    return m_scene->getCanvas()(m_controlRect).clone();
}

void CatchABallSimulator::resetShot()
{
    m_ball.reset();
    m_isBallFlying = false;
}

void CatchABallSimulator::shootBall(double shotAngleDeg,
                                    double ballSpeedMPS)
{
    if (m_isBallFlying)
        return;

    m_cannon->setShotAngle(shotAngleDeg);

    m_ball = m_cannon->shoot(ballSpeedMPS, m_ballRadius);
    m_isBallFlying = true;
    m_stats.shotsDone++;
}

void CatchABallSimulator::randomizedShootBall()
{
    if (m_isBallFlying)
        return;

    double rndDegChange = getRandomInRange( -RANDOM_CANNON_DEG_CHANGE, RANDOM_CANNON_DEG_CHANGE );
    double rndSpeedMPS = getRandomInRange( m_speedLimitsMPS[0], m_speedLimitsMPS[1] );

    double newShotAngle = m_shotInitialAngle + rndDegChange;
    double newShotSpeed = rndSpeedMPS;

    cout << "Random shot with " << newShotAngle << " [deg] / " << newShotSpeed << " [m/s]" << endl;

    shootBall(newShotAngle, newShotSpeed);
}

CatchABallStatistics CatchABallSimulator::getStatistics()
{
    return m_stats;
}

void CatchABallSimulator::tick()
{
    m_current_tick++;
    
    if (m_isBallFlying && m_ball){
        m_ball->move(m_sim_dt);

        /* Outline condition */
        if ( !m_scene->contains(m_ball) )
        {
            /* If ball flew away no through right wall */
            if ( m_scene->m2px(m_ball->getPosition()).x < m_scene->getRect().width )
                m_stats.flewAway++;

            resetShot();
        }

        /* Catch condition */
        else if ( m_plane->isBallCaught(m_ball) )
        {
            m_stats.ballsCatched++;
            resetShot();
        }
    }

    if ( !m_isBallFlying && m_isAutoShotsEnabled )
    {
        randomizedShootBall();
    }

    m_plane->move(m_sim_dt);
    // Cannon movement not required

    /* After movement render positions */
    m_scene->cleanCanvas();

    m_scene->render(m_cannon);
    m_scene->render(m_plane);

    if (m_ball)
        m_scene->render(m_ball);
    
    m_scene->renderControlZone(m_controlRect);
}

void CatchABallSimulator::setAutoShootingMode(bool enabled)
{
    m_isAutoShotsEnabled = enabled;
}

bool CatchABallSimulator::isEnd()
{
    return m_isEnd;
}

void CatchABallSimulator::setPlaneControl(int frameYPx)
{
    frameYPx = frameYPx > m_scene->getRect().height ? m_scene->getRect().height:
                frameYPx < 0 ? 0 : frameYPx;

    Point2d refPos = m_scene->px2m(Point(0, frameYPx));

    m_plane->setRefPosition( refPos.y );
}

int CatchABallSimulator::getPlaneDistancePx()
{
    return m_scene->m2px( m_plane->getPosition() ).x - m_controlRect.x;
}
