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
    
    Real Rover_Goal_X[5] = {2, 2, 2, 5, 7};
    Real Rover_Goal_Y[5] = {1, 5, 1, 5, 1};
    
    
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

    
private:
    
    void SetHeadingAngle(CRadians cheadangle);
    
    void TransitToGoal();
    
    void Stop();
    
    void SetWheelSpeeds(CRadians cAngle);
    
    CVector2 DiffusionVector(bool& b_collision);
    
    CRadians CalculateHeadingAngle(CVector3 cPosition);
    
private:
    enum EState
            {
                STATE_INITIAL = 0,
                STATE_TRANSISITION,
                STATE_GOAL
            };
    
    enum ETurningMechanism
        {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        };
    
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
   CRadians cAngle1;
   CDegrees cAngle;
   CVector2 cAccumulator;
   EState m_estate;
   ETurningMechanism TurningMechanism;
   /*
   * Angular thresholds to change turning state.
   */
   CRadians HardTurnOnAngleThreshold;
   CRadians SoftTurnOnAngleThreshold;
   CRadians NoTurnAngleThreshold;
    CRadians cHeading_angle;
   /* Maximum wheel speed */
   Real MaxSpeed;
   bool bCollision;
   bool Stop_x_motion;
   bool Stop_y_motion;
   CRadians cHeadingAngleOffset;
   CVector3 m_cPosition;
   bool StateInitialflag;
   bool TransitFlag;
   CVector3 cCurrentPos;
};
#endif

