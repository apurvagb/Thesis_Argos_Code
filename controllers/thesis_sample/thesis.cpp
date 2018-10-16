/* Include the controller definition */
#include "thesis.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

static UInt16 collision_counter;
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
//    m_tConfiguration.LoadFile(str_conf_fname);
//    m_tConfRoot = *m_tConfiguration.FirstChildElement();
//    TConfigurationNode& tFramework = GetNode(m_tConfRoot, "framework");
//    TConfigurationNode& tExperiment = GetNode(tFramework, "experiment");
//    GetNodeAttribute(tExperiment, "ticks_per_second", TicksPerSec);
    
    CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
    GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
    GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                             ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
    GetNodeAttribute(t_node, "delta", Delta);

    GetNodeAttribute(t_node, "max_speed", MaxSpeed);

    
    std::string id = GetId();

    id = GetId();
    std::string extracted_str = extractID(id);
    stRobotData.id_robot = std::stoi(extracted_str);
    stRobotData.TargetPosition.Set(Rover_Goal_X[stRobotData.id_robot],Rover_Goal_Y[stRobotData.id_robot],0);
    stRobotData.WaypointStack.push(stRobotData.TargetPosition);
    
    CurrentMovementState = STOP;
    GoStraightAngleRangeInDegrees.Set(-37.5, 37.5);
    TicksPerSec = 32;

    m_cPosition = m_pcPosSens->GetReading().Position;
    
    stRobotData.StartPosition = m_cPosition;
    stRobotData.StartWaypoint = stRobotData.StartPosition;
    
//    stRobotData.TotalDistanceToTarget = CalculateTargetDistance(m_cPosition, stRobotData.TargetPosition);
    stRobotData.fLinearWheelSpeed = 10.0f;
    stRobotData.fBaseAngularWheelSpeed = 8.0f;
    
   
    stRobotData.Priority = stRobotData.id_robot;
    
//    stRobotData.InitialOrientation = GetHeadingAngle();
    
    stRobotData.Intial_TurningWaitTime = GetInitial_TurningWaitTime(stRobotData);
    
//    CurrentWayPoint = stRobotData.WaypointStack.top();
    stRobotData.TargetWaypoint = stRobotData.TargetPosition;
 
//    stRobotData.WaypointStack.pop();
    stRobotData.WaypointCounter = 0;
    stRobotData.Checked = 0;
    collision_counter = 0;
}

/************************************************/
/* Function to get the number in the string */
/************************************************/
std::string CFootBotThesis::extractID(std::string str)
//void CFootBotThesis::extractID(std::string str)
{
    UInt8 counter;
    std::string output_str;
    for (counter=0; counter < str.length(); counter++){
        if (isdigit(str[counter])){
            output_str+= str[counter];
        }
    }
    return output_str;
}

/****************************************/
/* Control Step */
/****************************************/
void CFootBotThesis::ControlStep() {

    if(stRobotData.id_robot == 0 or stRobotData.id_robot == 1)
    {
        LOG<<"RobotID: "<<stRobotData.id_robot<<std::endl;
        LOG<<"Collision Counter: "<<collision_counter<<std::endl;
        
        LOG<<"Waypoint Start: "<<stRobotData.StartWaypoint<<std::endl;
        LOG<<"Waypoint End: "<<stRobotData.TargetWaypoint<<std::endl;
    //        LOG<<"Waypoint added flag: "<<stRobotData.WaypointReached<<std::endl;
    //        LOG<<"Checked flag: "<<stRobotData.Checked<<std::endl;
        LOG<<"Heading Angle: "<<stRobotData.InitialOrientation<<std::endl;
    //        LOG<<"Collision length: "<<collisionVector.Length()<<std::endl;
    //        LOG<<"Collision length: "<<collisionAngle<<std::endl;
//        LOG<<"Stop Time: "<<stRobotData.StopTurningTime<<std::endl;
    //        LOG<<"Waypoint Stack Size: "<<stRobotData.WaypointStack.size()<<std::endl;
        LOG<<"Collinearity: "<<stRobotData.dist<<std::endl;
    //
    //        LOG<<"Intersection robot: "<<st_IntersectionData.Robot_ID_Intersectingwith<<std::endl;
        LOG<<"Intersection point: "<<st_IntersectionData.IntersectionPoint<<std::endl;
        LOG<<"Current Movement State: "<<CurrentMovementState<<std::endl;
        LOG<<"Collinear flag "<<stRobotData.CollinearFlag<<std::endl;
    //        LOG<<"Initial Turning Wait time: "<<stRobotData.Intial_TurningWaitTime<<std::endl;
        LOG<<"Veocity: "<<stRobotData.fLinearWheelSpeed<<std::endl;
        LOG<<"----------------------------------------------------"<<std::endl;

    //        LOG<<"Stack: "<<stRobotData.WaypointStack.top()<<std::endl;
    //        LOG<<"Initial Turning Time: "<<stRobotData.Intial_TurningWaitTime<<std::endl;
    //        LOG<<"Priority: "<<stRobotData.Priority<<std::endl;
    //        LOG<<"Stop Turning Time: "<<stRobotData.StopTurningTime<<std::endl;
    //        LOG<<"Distance between robot: "<<stRobotData.dist<<std::endl;
    //        LOG<<"velocity: "<<stRobotData.fLinearWheelSpeed<<std::endl;
    }
    stRobotData.CurrentPos = GetPosition();
    if(stRobotData.Checked == 1)
    {
        Move();
    }
    
}

/****************************************/
/* Function to get the waiting ticks */
/****************************************/

UInt16 CFootBotThesis::GetTicksToWait(Real length, Real Speed){

    UInt16 wait_ticks = std::ceil((abs(length) * TicksPerSec) / Speed);

    return wait_ticks;
}

/****************************************/
/* Function to calculate Arc length */
/****************************************/
Real CFootBotThesis::CalculateArcLength(Real AngleToTurn){

    /* arc length = robot radius * angle to turn */
    Real arc_length = (2*PI*(FOOTBOT_RADIUS + 0.2) * AngleToTurn)/360;

    return arc_length;
}

/********************************************************************************************/
/* Function to calculate distance to Target*/
/********************************************************************************************/
Real CFootBotThesis::CalculateTargetDistance(CVector3 cPosition, CVector3 Targetposition){
    Real dist_y, dist_x, distance;

    dist_x = Targetposition.GetX() - cPosition.GetX();
    dist_y = Targetposition.GetY() - cPosition.GetY();

    distance = sqrt((dist_y * dist_y)+(dist_x * dist_x));

    return distance;
}

/***************************************************************************************************/
/* Function to calculate the angle in which robot should head towards the goal */
/***************************************************************************************************/
UInt16 CFootBotThesis::GetInitial_TurningWaitTime(CFootBotThesis::RobotData stRobotData){

    UInt16 TicksToWaitToTurn;
    CRadians orientation;
    Real newAngleToTurnInDegrees, s;

    /* get the heading angle towards goal */
    
//    CRadians headingToTarget = (stRobotData.TargetPosition - stRobotData.StartPosition).GetZAngle();
    CRadians headingToTarget = (stRobotData.TargetWaypoint - stRobotData.StartWaypoint).GetZAngle();
    
    orientation = GetHeadingAngle();
    /* get the current heading angle of the robot */
    CRadians headingToTargetError = (orientation - headingToTarget).SignedNormalize();
    
    /* turn left */
    if(headingToTargetError > TargetAngleTolerance)
    {
       newAngleToTurnInDegrees = -ToDegrees(headingToTargetError).GetValue();
       s = CalculateArcLength(newAngleToTurnInDegrees);
       TicksToWaitToTurn = GetTicksToWait(s, stRobotData.fBaseAngularWheelSpeed);
    }
    /* turn right */
    else if(headingToTargetError < -TargetAngleTolerance)
    {
        newAngleToTurnInDegrees = ToDegrees(headingToTargetError).GetValue();
        s = CalculateArcLength(newAngleToTurnInDegrees);
        TicksToWaitToTurn = GetTicksToWait(s, stRobotData.fBaseAngularWheelSpeed);
    }
    /* Move Forward */
    else
    {
        /* no time required to turn */
        TicksToWaitToTurn = 0;
    }
    
    return TicksToWaitToTurn;
//    stRobotData.Intial_TurningWaitTime = TicksToWaitToTurn;
//    LOG<<"Initial Turning Time individual: "<<stRobotData.Intial_TurningWaitTime<<std::endl;
}

/**********************************************************************/
/* Function to get the angle in which robot is heading */
/**********************************************************************/
CRadians CFootBotThesis::GetHeadingAngle(){

    // Every angle is for sure in the range [-PI,PI]
    CRadians cZAngle, cYAngle, cXAngle;

    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    return cZAngle;
}

/****************************************/
/* Function to get the curret position of the robot */
/****************************************/
CVector3 CFootBotThesis::GetPosition(){

    return m_pcPosSens->GetReading().Position;

}

/***********************************************************/
/* Function to check if the robot has reached the target */
/***********************************************************/
bool CFootBotThesis::IsAtTarget()
{
    CRadians AngleTol;
    float DistTol;


    DistTol = TargetDistanceTolerance;
    AngleTol = TargetAngleTolerance;

//    Real distanceToTarget = (stRobotData.TargetPosition - GetPosition()).Length();
    
    Real distanceToTarget = (stRobotData.TargetWaypoint - GetPosition()).Length();

    return (distanceToTarget < DistTol) ? (true) : (false);
}

/***************************************************/
/* Function to set the next movement of the robot */
/***************************************************/
void CFootBotThesis::SetNextMovement()
{
    CRadians AngleTol;
    float DistTol;

    DistTol = TargetDistanceTolerance;
    AngleTol = TargetAngleTolerance;


    /* if the movement stack is empty and movement state is STOP */
    if(MovementStack.size() == 0 && CurrentMovementState == STOP) {
        
        stRobotData.WaypointReached = false;
        /* get the distance between current point and goal point */
//        Real distanceToTarget = (stRobotData.TargetPosition - GetPosition()).Length();
        Real distanceToTarget = (stRobotData.TargetWaypoint - GetPosition()).Length();
        
        /* get the heading angle towards goal */
//        CRadians headingToTarget = (stRobotData.TargetPosition - GetPosition()).GetZAngle();
        CRadians headingToTarget = (stRobotData.TargetWaypoint - GetPosition()).GetZAngle();
        stRobotData.InitialOrientation = headingToTarget;
        /* get the current heading angle of the robot */
        CRadians headingToTargetError = (GetHeadingAngle() - headingToTarget).SignedNormalize();

        /* check if robot did not reach the traget */
        if(!IsAtTarget()) {

            /* turn left */
            if(headingToTargetError > AngleTol)
            {
                PushMovement(LEFT, -ToDegrees(headingToTargetError).GetValue());
            }
            /* turn right */
            else if(headingToTargetError < -AngleTol)
            {
                PushMovement(RIGHT, ToDegrees(headingToTargetError).GetValue());
            }
            /* Move Forward */
            else
            {
                PushMovement(FORWARD, distanceToTarget);
            }
        }
        /* robot has reached the target */
        else {
            
//            PushMovement(STOP, 0.0);

            
            /* get the next waypoint unless the stack is empty */
            if(!stRobotData.WaypointStack.empty())
            {
                stRobotData.WaypointReached = true;
                stRobotData.Checked = 0;
                stRobotData.Intial_TurningWaitTime = 0;
                stRobotData.StopTurningTime = 0;
                stRobotData.StartWaypoint = stRobotData.TargetWaypoint;
                
                stRobotData.TargetWaypoint = stRobotData.WaypointStack.top();
                stRobotData.WaypointStack.pop();
                

                stRobotData.Intial_TurningWaitTime = GetInitial_TurningWaitTime(stRobotData);
                
                CurrentMovementState = STOP;
                
            }
            /* if stack is empty, robot has reached the target */
            else{
                
                PushMovement(STOP, 0.0);
                stRobotData.WaypointReached = false;
            }
        }
       
    }
    /* continue the robot motion */
    else {
        PopMovement();}
}

/*************************************************************************/
/* Function to set the current movement state of the robot to left turn */
/*************************************************************************/
void CFootBotThesis::SetLeftTurn(Real newAngleToTurnInDegrees) {

    if(newAngleToTurnInDegrees > 0.0) {
        Real s = CalculateArcLength(newAngleToTurnInDegrees);

        TicksToWait = GetTicksToWait(s, stRobotData.fBaseAngularWheelSpeed);

        CurrentMovementState = LEFT;

    } else if(newAngleToTurnInDegrees < 0.0) {
        Real s = CalculateArcLength(newAngleToTurnInDegrees);

        TicksToWait = GetTicksToWait(s, stRobotData.fBaseAngularWheelSpeed);

        CurrentMovementState = RIGHT;

    } else {
        Stop();
    }
}

/*************************************************************************/
/* Function to set the current movement state of the robot to left turn */
/*************************************************************************/
void CFootBotThesis::SetRightTurn(Real newAngleToTurnInDegrees) {

    if(newAngleToTurnInDegrees > 0.0) {
        Real s = CalculateArcLength(newAngleToTurnInDegrees);
        TicksToWait = GetTicksToWait(s, stRobotData.fBaseAngularWheelSpeed);
        CurrentMovementState = RIGHT;
//        LOG << "Arc length " <<s<<std::endl;
//        LOG << "Ticks_To_Wait: " <<TicksToWait<<std::endl;
//        LOG << "CurrentMovementState: " <<CurrentMovementState<<std::endl;

    } else if(newAngleToTurnInDegrees < 0.0) {
        Real s = CalculateArcLength(newAngleToTurnInDegrees);
        TicksToWait = GetTicksToWait(s, stRobotData.fBaseAngularWheelSpeed);
        CurrentMovementState = LEFT;
//        LOG << "Arc length " <<s<<std::endl;
//        LOG << "Ticks_To_Wait: " <<TicksToWait<<std::endl;
//        LOG << "CurrentMovementState: " <<CurrentMovementState<<std::endl;

    } else {
        Stop();
    }
}

//void CFootBotThesis::Reset() {
//
//    stRobotData.TargetPosition.Set(0,0,0);
//    stRobotData.StartPosition.Set(0,0,0);
//    CurrentMovementState = STOP;
//    GoStraightAngleRangeInDegrees.Set(-37.5, 37.5);
//    TicksPerSec = 32;
//    TicksToWaitforSafedistance = ((2*(2*FOOTBOT_RADIUS)*TicksPerSec)/stRobotData.fLinearWheelSpeed);
//
//    stRobotData.TotalDistanceToTarget = 0;
//    stRobotData.fLinearWheelSpeed = 10.0f;
//    stRobotData.fBaseAngularWheelSpeed = 8.0f;
//
//
//    stRobotData.Priority = 0;
//
//    stRobotData.InitialOrientation = ToRadians(CDegrees(0));
//
//    stRobotData.Intial_TurningWaitTime = 0;
//    stRobotData.StopTurningTime = 0;
//}


/****************************************************************************/
/* Function to set the current movement state of the robot to move forward */
/****************************************************************************/
void CFootBotThesis::SetMoveForward(Real newTargetDistance) {

    if(newTargetDistance > 0.0) {
        TicksToWait = GetTicksToWait(newTargetDistance, stRobotData.fLinearWheelSpeed);
        CurrentMovementState = FORWARD;

    } else if(newTargetDistance < 0.0) {
        TicksToWait = GetTicksToWait(newTargetDistance, stRobotData.fLinearWheelSpeed);
        CurrentMovementState = BACK;
    }
    else {
        Stop();
    }
}

/****************************************************************************/
/* Function to set the current movement state of the robot to move backward */
/****************************************************************************/
void CFootBotThesis::SetMoveBack(Real newTargetDistance) {

    if(newTargetDistance > 0.0) {
        TicksToWait = GetTicksToWait(newTargetDistance, stRobotData.fLinearWheelSpeed);
        CurrentMovementState = BACK;

    } else if(newTargetDistance < 0.0) {
        TicksToWait = GetTicksToWait(newTargetDistance, stRobotData.fLinearWheelSpeed);
        CurrentMovementState = FORWARD;

    } else {
        Stop();
    }
}

/****************************************************************************/
/* Function to set the current movement state of the robot to stop */
/****************************************************************************/
void CFootBotThesis::Stop() {
    TicksToWait = 0.0f;
    CurrentMovementState = STOP;
}

/******************************************************************/
/* Function to push the next movement of the robot onto the stack */
/******************************************************************/
void CFootBotThesis::PushMovement(UInt8 moveType, Real moveSize) {
    Movement newMove = { moveType, moveSize };
    MovementStack.push(newMove);
}

/********************************************************************/
/* Function to pop the next movement of the robot from the stack */
/********************************************************************/
void CFootBotThesis::PopMovement() {
    Movement nextMove = MovementStack.top();

    previous_movement = nextMove;

    MovementStack.pop();

    switch(nextMove.type) {

        case STOP: {
            Stop();
            break;
        }

        case LEFT: {
            SetLeftTurn(nextMove.magnitude);
            break;
        }

        case RIGHT: {
            SetRightTurn(nextMove.magnitude);
            break;
        }

        case FORWARD: {
            SetMoveForward(nextMove.magnitude);
            break;
        }

        case BACK: {
            SetMoveBack(nextMove.magnitude);
            break;
        }

    }

}

/********************************************************************/
/* Function to detect collision */
/********************************************************************/
bool CFootBotThesis::CollisionDetection() {


    CVector3 PositionCurr, NewTargetWayPoint;
    Real x, y, diff;
    CRadians angle, angle_error, AngleTol;
    AngleTol = TargetAngleTolerance;
//    argos::CVector2 collisionVector = GetCollisionVector();
    collisionVector = GetCollisionVector();
//    argos::Real collisionAngle = ToDegrees(collisionVector.Angle()).GetValue();
    collisionAngle = ToDegrees(collisionVector.Angle()).GetValue();
    bool isCollisionDetected = false;

    if(GoStraightAngleRangeInDegrees.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
       && collisionVector.Length() > 0.0)
    {
            
        Stop();
        
        isCollisionDetected = true;
        collision_counter++;
        
        while(MovementStack.size() > 0) MovementStack.pop();
        
        
        // if collinear
        if(stRobotData.CollinearFlag == 1)
        {
        
            PushMovement(BACK, 0.1);
            
            PositionCurr = GetPosition();
            
            if(collisionAngle <= 0.0)
            {
                x = PositionCurr.GetX() - 0.5;
                y = PositionCurr.GetY() ;
                
                if(x < (Arena_Min + 0.4))
                {
                    diff = abs((Arena_Min + 0.4) - PositionCurr.GetX());
                    x = PositionCurr.GetX() - diff;
                }
                else if(x > (Arena_Max - 0.4))
                {
                    
                    diff = abs(PositionCurr.GetX() - (Arena_Max - 0.4));
                    x = PositionCurr.GetX() + diff;
                }
              
            }
            else
            {
                x = PositionCurr.GetX() + 0.5;
                y = PositionCurr.GetY();
                
                if(x < (Arena_Min + 0.4))
                {
                    diff = abs((Arena_Min + 0.4) - PositionCurr.GetX());
                    x = PositionCurr.GetX() - diff;
                }
                else if(x > (Arena_Max - 0.4))
                {
                    
                    diff = abs(PositionCurr.GetX() - (Arena_Max - 0.4));
                    x = PositionCurr.GetX() + diff;
                }
                
            }
            
            
            if(stRobotData.WaypointStack.empty())
            {
               stRobotData.WaypointStack.push(stRobotData.TargetWaypoint);
               NewTargetWayPoint.Set(x, y, 0);
                x =0;
                y=0;
                diff =0;
            }
            stRobotData.TargetWaypoint = NewTargetWayPoint;
            
            // Waypoint added
            stRobotData.Waypoint_Added = 1;
        }
        else{
            
//            if(!stRobotData.WaypointStack.empty())
//            {
//                stRobotData.TargetWaypoint = stRobotData.WaypointStack.top();
//                stRobotData.WaypointStack.pop();
//                collision_counter = 0;
//
//            }
//            else{
            PushMovement(FORWARD, SearchStepSize);
            if(collisionAngle <= 0.0)
            {
                SetLeftTurn((37.5 - collisionAngle));
            }
            else
            {
                SetRightTurn((37.5 + collisionAngle));
            }
            
        }

    }
    
    return isCollisionDetected;
}

/********************************************************************/
/* Function to get the collision readings */
/********************************************************************/
CVector2 CFootBotThesis::GetCollisionVector() {
    /* Get readings from proximity sensor */
    const argos::CCI_FootBotProximitySensor::TReadings& proximityReadings = m_pcProximity->GetReadings();

    /* Sum them together */
    argos::CVector2 collisionVector;

    for(size_t i = 0; i < proximityReadings.size(); ++i) {
        collisionVector += argos::CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
    }

    collisionVector /= proximityReadings.size();

    return collisionVector;
}

/******************************************************************/
/* Function to pop the next movement of the robot from the stack */
/******************************************************************/
void CFootBotThesis::Move() {
    
//    if(Wait() == true) return;

    CollisionDetection();

    /* move based on the movement state flag */
    switch(CurrentMovementState) {

            /* stop movement */
        case STOP: {
            if((stRobotData.StopTurningTime > 0.0))
           {
               Stop();
               stRobotData.StopTurningTime--;
           }
            else{
                m_pcWheels->SetLinearVelocity(0.0, 0.0);
                SetNextMovement();
            }
            
            break;
        }

            /* turn left until the robot is facing an angle inside of the TargetAngleTolerance */
        case LEFT: {
//            LOG << "Ticks_To_Wait: " <<TicksToWait<<std::endl;
            if((TicksToWait--) <= 0.0) {
                Stop();
            } else {
                m_pcWheels->SetLinearVelocity(-stRobotData.fBaseAngularWheelSpeed, stRobotData.fBaseAngularWheelSpeed);
            }
            break;
        }

            /* turn right until the robot is facing an angle inside of the TargetAngleTolerance */
        case RIGHT: {
            if((TicksToWait--) <= 0.0) {
                Stop();
            } else {
                m_pcWheels->SetLinearVelocity(stRobotData.fBaseAngularWheelSpeed, -stRobotData.fBaseAngularWheelSpeed);
            }
            break;
        }

            /* move forward until the robot has traveled the specified distance */
        case FORWARD: {
            if((TicksToWait--) <= 0.0) {
                Stop();
            } else {
                m_pcWheels->SetLinearVelocity(stRobotData.fLinearWheelSpeed, stRobotData.fLinearWheelSpeed);
            }
            break;
        }

            /* move backward until the robot has traveled the specified distance */
        case BACK: {
            if((TicksToWait--) <= 0.0) {
                Stop();
            } else {
                m_pcWheels->SetLinearVelocity(-stRobotData.fLinearWheelSpeed, -stRobotData.fLinearWheelSpeed);
            }
            break;
        }
    }
}

/*****************************************************/
/* Function to set the wheel speed the robot */
/*****************************************************/
void CFootBotThesis::SetWheelSpeeds(CRadians chead_angle) {

    CRadians cZAngle, cYAngle, cXAngle;
    Real error;

    /* get the orientation of the robot */
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    /* calculate the error in the desired angle and acual angle */
    error = (Abs(chead_angle) - Abs(cZAngle))/Abs(chead_angle);

    /* get the arctan of the error for the Proportional loop for angular velocity calculation */
    Real error_atan2 = atan2(sin(error), cos(error));

    /* calculate angular velocity */
    stRobotData.fBaseAngularWheelSpeed = Kp* error_atan2;

    /* Clamp the speed so that it's not greater than MaxSpeed */
    stRobotData.fBaseAngularWheelSpeed = Min(stRobotData.fBaseAngularWheelSpeed, MaxSpeed);



    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;

    LOG<< "fLeftWheelSpeed: "<< fLeftWheelSpeed << std::endl;
    LOG<< "fRightWheelSpeed: "<< fRightWheelSpeed << std::endl;
    LOG<< "fBaseAngularWheelSpeed: "<< stRobotData.fBaseAngularWheelSpeed << std::endl;
    LOG<< "error_atan2: "<< error_atan2 << std::endl;


    fLeftWheelSpeed = ((2 * 1) - (stRobotData.fBaseAngularWheelSpeed * FOOTBOT_INTERWHEEL_DISTANCE)) /(2* FOOTBOT_RADIUS);
    fRightWheelSpeed =((2 * 1) + (stRobotData.fBaseAngularWheelSpeed * FOOTBOT_INTERWHEEL_DISTANCE)) /(2* FOOTBOT_RADIUS);

    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);

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
