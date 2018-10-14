/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef THESIS_H
#define THESIS_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/space/space.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/* math library*/
#include <math.h>
#include <stack>
#include <iostream>
#include <sstream>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotThesis : public CCI_Controller {

public:
    
    Real Rover_Goal_X[5] = {1, -1.5, 0, 0, -1};
    Real Rover_Goal_Y[5] = {1, -1, 0, 0, 1};
    
    
   /* Class constructor. */
   CFootBotThesis();

   /* Class destructor. */
   virtual ~CFootBotThesis() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}
    
    
public:
    struct RobotData{
        CVector3 TargetPosition;
        CVector3 StartPosition;
        CVector3 TargetWaypoint;
        CVector3 StartWaypoint;
        UInt16 id_robot;
        UInt16 Priority;
        UInt16 WaypointCounter;
        Real fBaseAngularWheelSpeed;
        Real fLinearWheelSpeed;
        bool GoingToNest;
        CRadians InitialOrientation;
        Real TotalDistanceToTarget;
        UInt16 Intial_TurningWaitTime;
        UInt16 StopTurningTime;
        Real dist;
        bool Checked;
        bool Waypoint_Added;
        bool WaypointReached;
        std::stack<CVector3>WaypointStack;
    };
    
    struct IntersectionData{
        UInt16 Robot_ID_Intersectingwith;
        bool Intersection_flag;
        CVector3 IntersectionPoint;
    };
    
    /*
     * Returns the robot data
     */
    inline RobotData& GetRobotData() {
        return stRobotData;
    }
    
    /*
     * Returns the robot data
     */
    inline void SetRobotLinearVelocity(Real vel) {
        stRobotData.fLinearWheelSpeed = vel;
    }
    
    inline void SetRobotStopTurningTime(UInt16 time)
    {
        stRobotData.StopTurningTime = time;
    }
    
    /*
     * Returns the intersection data of the robot
     */
    inline IntersectionData& GetIntersectionData() {
        return st_IntersectionData;
    }
    
public:
    RobotData stRobotData;
    
    IntersectionData st_IntersectionData;
private:
    
    void SetWheelSpeeds(CRadians cAngle);
    
    UInt16 GetTicksToWait(Real length, Real Speed);
    
    Real CalculateArcLength(Real AngleToTurn);
    
    Real CalculateTargetDistance(CVector3 cPosition, CVector3 TargetPosition);
    
    UInt16 GetInitial_TurningWaitTime(CFootBotThesis::RobotData stRobotData);
    
    CRadians GetHeadingAngle();
    
    CVector3 GetPosition();
    
    bool IsAtTarget();
    
    void SetNextMovement();
    
    void SetLeftTurn(Real newAngleToTurnInDegrees);
    
    void SetRightTurn(Real newAngleToTurnInDegrees);
    
    void SetMoveForward(Real newTargetDistance);
    
    void SetMoveBack(Real newTargetDistance);
    
    void Stop();
    
    void PushMovement(UInt8 moveType, Real moveSize);
    
    void PopMovement();
    
    bool CollisionDetection();
    
    CVector2 GetCollisionVector();
    
    void Move();
    
    std::string  extractID(std::string str);
    
    
private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot position sensor */
   CCI_PositioningSensor* m_pcPosSens;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real Delta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> GoStraightAngleRange;

   /*
   * Angular thresholds to change turning state.
   */
   CRadians HardTurnOnAngleThreshold;
   CRadians SoftTurnOnAngleThreshold;
   CRadians NoTurnAngleThreshold;
    
   /* Maximum wheel speed */
   Real MaxSpeed;
   CVector2 collisionVector;
   Real collisionAngle;
    
   const Real Kp                               = 5;
   const Real FOOTBOT_RADIUS                   = 0.085036758f;
   const Real FOOTBOT_INTERWHEEL_DISTANCE      = 0.14f;
//   const Real fBaseAngularWheelSpeed           = 8.0f;
//   const Real fLinearWheelSpeed                = 10.0f;
   const Real TargetDistanceTolerance          = 0.01;
   const CRadians TargetAngleTolerance         = CRadians(0.04);
   const Real SearchStepSize                   = 0.16;
   const Real PI                               = 3.141592653589793238463;
   UInt16 TicksToWaitforSafedistance;
   UInt16 collision_counter;
   CRange<Real> GoStraightAngleRangeInDegrees;
   ticpp::Document m_tConfiguration;
   TConfigurationNode m_tConfRoot;
   UInt16 TicksToWait;
   Real TicksPerSec;
   CVector3 m_cPosition;

   CVector3 cCurrentPos;
   CVector3 CurrentWayPoint;

   Real Distance_To_Goal;
   bool Start_Motion;
    
    
   // controller state variables
   enum MovementState {
        STOP    = 0,
        LEFT    = 1,
        RIGHT   = 2,
        FORWARD = 3,
        BACK    = 4
    } CurrentMovementState;

    /* movement definition variables */
    struct Movement {
        UInt8 type;
        Real magnitude;
    };

    Movement previous_movement;
    CVector2 previous_pattern_position;
    std::stack<Movement> MovementStack;
    

};
#endif

