/* Include the controller definition */
#include "thesis.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/****************************************/
/****************************************/

CFootBotThesis::CFootBotThesis() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_fWheelVelocity(2.5f),
   m_cAlpha(10.0f),
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

/****************************************/
/****************************************/

void CFootBotThesis::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"          );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
    CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
    GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
    GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                             ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
    GetNodeAttribute(t_node, "delta", Delta);
    GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
    HardTurnOnAngleThreshold = ToRadians(cAngle);
    GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
    SoftTurnOnAngleThreshold = ToRadians(cAngle);
    GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
    NoTurnAngleThreshold = ToRadians(cAngle);
    GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    
    cHeadingAngleOffset = ToRadians(CDegrees(1.0f));
    m_estate = STATE_INITIAL;
    TurningMechanism = NO_TURN;
    m_cPosition = m_pcPosSens->GetReading().Position;

}

/****************************************/
/****************************************/

void CFootBotThesis::ControlStep() {
    
    CRadians angle;

    switch(m_estate){
            
        /* State to get the heading direction */
        case STATE_INITIAL:

            angle = CalculateHeadingAngle(m_cPosition);
            cHeading_angle = angle;
            
            /* change the state of the robot */
            m_estate = STATE_ADJUST_DIRECTION;
            break;
            
        /* State to set the robot in motion */
        case STATE_TRANSISITION:
            /* function to set the robot in motion */
            TransitToGoal();
            break;
            
        case STATE_ADJUST_DIRECTION:
            /* Turn the robot as per heading angle */
            SetHeadingAngle(cHeading_angle);
            
        /* State to handle robot if reched goal */
        case STATE_GOAL:
            Stop();
            break;

        default:
            LOGERR << "[BUG] Unknown robot state: " << m_estate << std::endl;
        
    }
}

/****************************************/
/* Function to control the robot motion towards the goal */
/****************************************/
void CFootBotThesis::TransitToGoal(){
    
    Real X, Y, Z;
    CRadians angle;
    
    bool bCollision;
    /* Get the diffusion vector to perform obstacle avoidance */
    CVector2 cDiffusion = DiffusionVector(bCollision);
    
    /* Get the position of the footbot using position sensor */
    cCurrentPos = m_pcPosSens->GetReading().Position;
    X = cCurrentPos.GetX();
    Y = cCurrentPos.GetY();
    
    LOG << "Position X: " << X << std::endl;
    LOG << "Position Y: " << Y << std::endl;
    LOG << "transition_state: " << transition_state << std::endl;
    LOG << "m_estate: " << m_estate << std::endl;
    LOG << "Turning Mechanism " << TurningMechanism << std::endl;
    LOG << "Adjustment_counter" << Adjustment_counter << std::endl;
    LOG << "------------------------------------" << std::endl;
    
    switch(transition_state){
        /* state to check if the robot coordinates are equal the goal */
        case GOAL_NOT_REACHED:
            
            /* Both coordinates are equal to goal coordinates */
            if((X <= (Rover_Goal_X[0] + 0.05) and X >= (Rover_Goal_X[0] - 0.05)) and
              (Y <= (Rover_Goal_Y[0] + 0.05) and Y >= (Rover_Goal_Y[0] - 0.05)))
               {
                   transition_state = GOAL_REACHED;
               }
            /* Only one of the coordinate is equal to one of the goal coordinate */
            else if((X <= (Rover_Goal_X[0] + 0.05) and X >= (Rover_Goal_X[0] - 0.05)) or
                   (Y <= (Rover_Goal_Y[0] + 0.05) and Y >= (Rover_Goal_Y[0] - 0.05)))
            {
                      
               transition_state = ONE_COORDINATE_REACHED;
            }
            /* None of the coordinate is equal to goal coordinate */
            else
            {
                cDiffusion = MaxSpeed * cDiffusion;
                /* Use the diffusion vector only */
                SetWheelSpeeds(cDiffusion.Angle());
            }
            break;
        /* Only one of the coordinate is equal to one of the goal coordinate */
        case ONE_COORDINATE_REACHED:
            
                angle = CalculateHeadingAngle(cCurrentPos);
                cHeading_angle = angle;
                SetHeadingAngle(cHeading_angle);
            
            break;
            
        /* Both coordinates are equal to goal coordinates */
        case GOAL_REACHED:
                m_estate = STATE_GOAL;
            break;
            
    }

}
/****************************************/
/* Function to calculate the angle in which rover should head towards the goal */
/****************************************/
CRadians CFootBotThesis::CalculateHeadingAngle(CVector3 cPosition){
    Real dist_y, dist_x;
    CRadians headingangle;
    
    dist_y = cPosition.GetY() - Rover_Goal_Y[0];
    dist_x = cPosition.GetX() - Rover_Goal_X[0];
    
    /* Get the heading angle towards the goal */
    headingangle = CRadians(atan2(dist_y, dist_x)) ;
    return headingangle;
}


/****************************************/
/* Function to set the angle in which rover should head towards the goal */
/****************************************/
void CFootBotThesis::SetHeadingAngle(CRadians cheadangle){
    
    CRadians angle;

    // Every angle is for sure in the range [-PI,PI]
    CRadians cZAngle, cYAngle, cXAngle;

    cHeading_angle = cheadangle;

    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    /* If the robot is in the X<0, Y>0 quadrant, go straight, otherwise rotate on the spot */
    
    /* check if the angle rotated is in the range (angle+1) and (angle-1) */
    if(cZAngle >= (cHeading_angle - 2*cHeadingAngleOffset) and
       cZAngle <= (cHeading_angle + 2*cHeadingAngleOffset)) {
        
        TurningMechanism = NO_TURN;
        transition_state = GOAL_NOT_REACHED;
        m_estate = STATE_TRANSISITION;
    }
    /* continue to rotate at specified angle */
    else {
        
        /* Get the diffusion vector to perform obstacle avoidance */
        bool bCheckCollision;
        CVector2 cDiffusion = DiffusionVector(bCheckCollision);
        
        /* if there is collision while adjusting the direction, set the direction to diffusion angle */
        if(bCheckCollision):
        {
            LOG<< "Collision Detected"<< std::endl;
            /* increment the counter, everytime there is collision and adjustment */
            Adjustment_counter++;
            
            /* Calculate the wheel speeds to turn towards heading angle */
            SetWheelSpeeds(cDiffusion.Angle());
        }
        
        else{
            /* Fine tune the turning when the turn is hard turn and need to turn softly*/
            if((cZAngle > (cHeading_angle + 10*cHeadingAngleOffset)) or
               (cZAngle < (cHeading_angle - 10*cHeadingAngleOffset)))
            {
                cHeading_angle = (cHeading_angle + cHeadingAngleOffset) - cHeading_angle;
                
                /* turn the hard turn state to soft turn state */
                if (cHeading_angle < NoTurnAngleThreshold) {
                    cHeading_angle = 2*(NoTurnAngleThreshold - cHeading_angle) + cHeading_angle;
                }
            }
            /* Calculate the wheel speeds to turn towards heading angle */
            SetWheelSpeeds(cHeading_angle);
            
        }
       
        /* Change the state to STATE_ADJUST if need to adjust the angles */
        if(Adjustment < 20){
            m_estate = STATE_ADJUST_DIRECTION;
        }
        else{
            LOG<< "Stopped the robot as number of adjustments exceeded"<< std::endl;
            m_estate = STATE_GOAL;
        }
    }
}

/****************************************/
/* Function to stop the robot */
/****************************************/
void CFootBotThesis::Stop(){
    
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    
}
/****************************************/
/* Function to set the wheel speed the robot */
/****************************************/
void CFootBotThesis::SetWheelSpeeds(CRadians cAngle) {

    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = MaxSpeed;
    
    /* State transition logic */
    if(TurningMechanism == HARD_TURN) {
        if(Abs(cAngle) <= SoftTurnOnAngleThreshold) {
            TurningMechanism = SOFT_TURN;
        }
    }
    if(TurningMechanism == SOFT_TURN) {
        if(Abs(cAngle) > HardTurnOnAngleThreshold) {
            TurningMechanism = HARD_TURN;
        }
        else if(Abs(cAngle) <= NoTurnAngleThreshold) {
            TurningMechanism = NO_TURN;
        }
    }
    if(TurningMechanism == NO_TURN) {
        if(Abs(cAngle) > HardTurnOnAngleThreshold) {
            TurningMechanism = HARD_TURN;
        }
        else if(Abs(cAngle) > NoTurnAngleThreshold) {
            TurningMechanism = SOFT_TURN;
        }
    }
    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(TurningMechanism) {
        case NO_TURN: {
            /* Just go straight */
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            break;
        }
        case SOFT_TURN: {
            /* Both wheels go straight, but one is faster than the other */
            Real fSpeedFactor = (HardTurnOnAngleThreshold - Abs(cAngle)) / HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            break;
        }
        case HARD_TURN: {
            /* Opposite wheel speeds */
            fSpeed1 = -MaxSpeed;
            fSpeed2 =  MaxSpeed;
            break;
        }
    }
    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if(cAngle > CRadians::ZERO) {
        /* Turn Left */
        fLeftWheelSpeed  = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else {
        /* Turn Right */
        fLeftWheelSpeed  = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/* Function to get the obstacle distance*/
/****************************************/
CVector2 CFootBotThesis::DiffusionVector(bool& b_collision) {

    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    /* Sum them together */
    for(size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
     is far enough, ignore the vector and go straight, otherwise return
     it */
    if(GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAccumulator.Angle()) &&
       cAccumulator.Length() < Delta) {
            b_collision = false;
            return CVector2::X;
    }
    else {
        
        b_collision = true;
        cAccumulator.Normalize();
        return -cAccumulator;
    }
}


/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotThesis, "footbot_diffusion_controller")
