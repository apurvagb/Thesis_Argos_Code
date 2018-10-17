#include "thesis_loop_functions.h"
//#include <controllers/thesis_sample/thesis.h>

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

void CThesisLoopFunctions::Init(TConfigurationNode& t_tree) {
   /*
    * Go through all the robots in the environment
    * and create an entry in the waypoint map for each of them
    */
 
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Create a waypoint vector */
      m_tWaypoints[pcFB] = std::vector<CVector3>();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
       
   }
    SimulatorTicksperSec = 32;
    RobotReachedWayPoint = 0;
    WaitCounter = 0;
    FirstCheck = 1;
    CollinearFlag = 0;
}

/****************************************/
/****************************************/

void CThesisLoopFunctions::Reset() {
   /*
    * Clear all the waypoint vectors
    */
   /* Get the map of all foot-bots from the space */

   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Clear the waypoint vector */
      m_tWaypoints[pcFB].clear();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
    RobotReachedWayPoint = 0;
    WaitCounter = 0;
    NewWayPointAdded = 0;
    FirstCheck = 1;
    CollinearFlag = 0;
}

/****************************************/
/****************************************/

void CThesisLoopFunctions::PostStep() {
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Add the current position of the foot-bot if it's sufficiently far from the last */
      if(SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
                        m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
         m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
      }
   }
}
/****************************************/
/****************************************/
void CThesisLoopFunctions::PreStep() {
    UInt16 counter = 0;
    Real RobotCourseAngle;
    
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    /* Get the hadndle to each robot and check if any one waypoint reached*/
    for(CSpace::TMapPerType::iterator it2 = m_cFootbots.begin();
        it2 != m_cFootbots.end();
        ++it2)
    {
        /* Get handle to foot-bot entity and controller of the current robot */
        CFootBotEntity& cFootBot2 = *any_cast<CFootBotEntity*>(it2->second);
        CFootBotThesis& cController2 = dynamic_cast<CFootBotThesis&>(cFootBot2.GetControllableEntity().GetController());
        
        CFootBotThesis::RobotData& stRobotDataCopy = cController2.GetRobotData();
        RobotReachedWayPoint |= stRobotDataCopy.WaypointReached;
        NewWayPointAdded |= stRobotDataCopy.WaypointReached;
        
    }
    
    /* check collinearity and intersection if target reached or new waypoint added or its the start of code */
    if(RobotReachedWayPoint == 1 or FirstCheck == 1 or NewWayPointAdded == 1)
    {
    
        /* Get the hadndle to each robot */
        for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
            it != m_cFootbots.end();
            ++it)
        {

            /* Get handle to foot-bot entity and controller of the current robot */
            CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
            CFootBotThesis& cController = dynamic_cast<CFootBotThesis&>(cFootBot.GetControllableEntity().GetController());

            CFootBotThesis::RobotData& sRobotDataprevious = cController.GetRobotData();
            CFootBotThesis::IntersectionData& sIntersectionDataprevious = cController.GetIntersectionData();

            
            /* Get the hadndle to each next robot */
            for(CSpace::TMapPerType::iterator it1 = std::next(it, 1);
                                it1 != m_cFootbots.end();
                                ++it1)

            {

                CFootBotEntity& cFootBot1 = *any_cast<CFootBotEntity*>(it1->second);
                CFootBotThesis& cController1 = dynamic_cast<CFootBotThesis&>(cFootBot1.GetControllableEntity().GetController());

                CFootBotThesis::RobotData& sRobotDatanext = cController1.GetRobotData();
                CFootBotThesis::IntersectionData& sIntersectionDatanext = cController1.GetIntersectionData();
                

                /* check if robot's end waypoint is collinear in other robot's start and end waypoint */
                CheckCollinearity(sRobotDataprevious, sRobotDatanext, sIntersectionDatanext);
            
                
                if(sRobotDataprevious.CollinearFlag !=1)
                {
                    /* find intersection between paths of robots */
                    Find_Intersection(sRobotDataprevious, sRobotDatanext, sIntersectionDatanext);
    
                    
                    if(sIntersectionDatanext.Intersection_flag == 1)
                    {
                        /* take actions to avoid intersections */
                        IntersectionCollisionCheck(sRobotDataprevious, sRobotDatanext, sIntersectionDatanext);
                    }
                    
                    
                }
                
                if(sRobotDataprevious.Waypoint_Added == true and !sRobotDataprevious.WaypointStack.empty())
                {

                    sRobotDataprevious.TargetWaypoint =  sRobotDataprevious.WaypointStack.top();
                    sRobotDataprevious.WaypointStack.pop();
                    sRobotDataprevious.Waypoint_Added = false;
                }



            }
            sRobotDataprevious.Checked = 1;

//            if(!sRobotDataprevious.WaypointStack.empty())
//            {
//                sRobotDataprevious.TargetWaypoint =  sRobotDataprevious.WaypointStack.top();
//                sRobotDataprevious.WaypointStack.pop();
//            }
            

        }
        /* reset the firstcheck flag */
        FirstCheck = 0;
        
        
        /* reset the RobotReachedWayPoint flag */
        RobotReachedWayPoint = 0;
        
        /* reset the RobotReachedWayPoint flag */
        NewWayPointAdded = 0;
        
        CollinearFlag = 0;
    }
    
}
/****************************************************************************************************************/
/* Function to calculate intersection points of robot paths */
/****************************************************************************************************************/
void CThesisLoopFunctions::Find_Intersection(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2,
                                             CFootBotThesis::IntersectionData &ptr3){

    Real x_inter;
    Real y_inter;
    Real A1, A2, B1, B2, C1, C2, det;

    /* get the start and target position of robot 1 */
    CVector3 StartPosition_Robot1 = ptr1.StartWaypoint;
    CVector3 TargetPosition_Robot1 = ptr1.TargetWaypoint;

    /* get the start and target position of robot 2 */
    CVector3 StartPosition_Robot2 = ptr2.StartWaypoint;
    CVector3 TargetPosition_Robot2 = ptr2.TargetWaypoint;

    /*A1 = Robot1_goal_y - Robot1_start_y*/
    A1 = TargetPosition_Robot1.GetY() - StartPosition_Robot1.GetY();



    /*B1 = Robot1_start_x - Robot1_goal_x*/
    B1 = StartPosition_Robot1.GetX() - TargetPosition_Robot1.GetX();



    /* C1 = A1 * Robot1_start_x + B1 * Robot1_start_y */
    C1 = A1 * StartPosition_Robot1.GetX() + B1 * StartPosition_Robot1.GetY();


    /*A2 = Robot2_goal_y - Robot2_start_y*/
    A2 = TargetPosition_Robot2.GetY() - StartPosition_Robot2.GetY();

    /*B2 = Robot2_start_x - Robot2_goal_x*/
    B2 = StartPosition_Robot2.GetX() - TargetPosition_Robot2.GetX();

    /* C2 = A2 * Robot2_start_x + B2 * Robot2_start_y */
    C2 = A2 * StartPosition_Robot2.GetX() + B2 * StartPosition_Robot2.GetY();

    det = A1*B2 - A2*B1;

    if(det == 0)
    {
        /* Lines are parallel */
        ptr3.Intersection_flag= 0;
    }

    /* Lines intersect and find the intersection point */
    else{
        x_inter = (B2*C1 - B1*C2)/det;

        y_inter = (A1*C2 - A2*C1)/det;

        /* Check if intersection point is out of bound for the line segment */
        if((x_inter < std::max(std::min(StartPosition_Robot1.GetX(), TargetPosition_Robot1.GetX()),
                               std::min(StartPosition_Robot2.GetX(), TargetPosition_Robot2.GetX()))) or
           (x_inter > std::min(std::max(StartPosition_Robot1.GetX(), TargetPosition_Robot1.GetX()),
                               std::max(StartPosition_Robot2.GetX(), TargetPosition_Robot2.GetX()))))
        {
            ptr3.Intersection_flag = 0;
        }
        else if((y_inter < std::max(std::min(StartPosition_Robot1.GetY(), TargetPosition_Robot1.GetY()),
                                    std::min(StartPosition_Robot2.GetY(), TargetPosition_Robot2.GetY()))) or
                (y_inter > std::min(std::max(StartPosition_Robot1.GetY(), TargetPosition_Robot1.GetY()),
                                    std::max(StartPosition_Robot2.GetY(), TargetPosition_Robot2.GetY()))))
        {
            ptr3.Intersection_flag = 0;
        }
        else
        {
            ptr3.Intersection_flag = 1;
            ptr3.IntersectionPoint.Set(x_inter, y_inter, 0);
            ptr3.Robot_ID_Intersectingwith = ptr1.id_robot;
        }
    }
}

/********************************************************************************************/
/* Function to calculate distance to Target*/
/********************************************************************************************/
Real CThesisLoopFunctions::CalculateDistance(CVector3 cPosition1, CVector3 cPosition2){
    Real dist_y, dist_x, distance;

    dist_x = cPosition2.GetX() - cPosition1.GetX();
    dist_y = cPosition2.GetY() - cPosition1.GetY();

    distance = sqrt((dist_y * dist_y)+(dist_x * dist_x));

    return distance;
}

/**************************************************************************************************************************/
/* Function to calculate time in terms of ticks */
/**************************************************************************************************************************/
UInt16 CThesisLoopFunctions::GetTicksToWait(Real dist, Real speed)
{
    UInt16 wait_ticks = std::ceil((abs(dist) * SimulatorTicksperSec) / speed);

    return wait_ticks;
}


/**************************************************************************************************************************/
/* Function to calculate time required by robot to reach intersection point */
/**************************************************************************************************************************/
void CThesisLoopFunctions::IntersectionCollisionCheck(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2,
                                                      CFootBotThesis::IntersectionData &ptr3){

    UInt16 TicksToWait_Robot1, TicksToWait_Robot2, TicksToWaitforSafedistance, TimeToIntersection, TimeDiff;
    Real DistanceToIntersection_Robot1, DistanceToIntersection_Robot2, IntersectionDistance;
    Real AdjustedVelocity;
    bool Intersection_flag;
    

    /* Get the distance between start point and intersection point */
    DistanceToIntersection_Robot1 = CalculateDistance(ptr1.StartWaypoint, ptr3.IntersectionPoint);
    DistanceToIntersection_Robot2 = CalculateDistance(ptr2.StartWaypoint, ptr3.IntersectionPoint);
    
    /* calculate the time required to reach the intersection point */
    TicksToWait_Robot1 = GetTicksToWait(DistanceToIntersection_Robot1, ptr1.fLinearWheelSpeed) + ptr1.StopTurningTime;

    TicksToWait_Robot2 = GetTicksToWait(DistanceToIntersection_Robot2, ptr2.fLinearWheelSpeed)+ ptr2.StopTurningTime;
    
    
    TimeDiff = abs(TicksToWait_Robot1 - TicksToWait_Robot2);

    TicksToWaitforSafedistance = GetTicksToWait(Safedistance , MaxLinearSpeed);


    //if the difference between the times is equal to safe distance time between two robots
    if(TimeDiff <= TicksToWaitforSafedistance)
    {

        /* there is a chance of collision */
        /* slow down the velocity of robot 2 as its priority is lower */

        if(DistanceToIntersection_Robot1 < DistanceToIntersection_Robot2)
        {
            IntersectionDistance = DistanceToIntersection_Robot2;
            TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
            
            if(AdjustedVelocity < MinLinearSpeed)
            {
                AdjustedVelocity = MinLinearSpeed;
                Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
                Real stop_time = abs(Time - TimeToIntersection);
                ptr2.StopTurningTime = stop_time;
            }
            ptr2.fLinearWheelSpeed = AdjustedVelocity;

        }
        /* slow down the velocity of robot 1 as its priority is lower */
        else if(DistanceToIntersection_Robot2 < DistanceToIntersection_Robot1)
        {
            IntersectionDistance = DistanceToIntersection_Robot1;
            TimeToIntersection = TicksToWait_Robot2 + TicksToWaitforSafedistance;
            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
            if(AdjustedVelocity < MinLinearSpeed)
            {
                AdjustedVelocity = MinLinearSpeed;
                Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
                Real stop_time = abs(Time - TimeToIntersection);
                ptr1.StopTurningTime = stop_time;
            }
            ptr1.fLinearWheelSpeed = AdjustedVelocity;
        }
        else{
            if(ptr1.GoingToNest)
            {
                IntersectionDistance = DistanceToIntersection_Robot2;
                TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
                AdjustedVelocity = (IntersectionDistance/ TimeToIntersection) * SimulatorTicksperSec;
                
                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MinLinearSpeed;
                    Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
                    Real stop_time = abs(Time - TimeToIntersection);
                    ptr2.StopTurningTime = stop_time;
                }
                ptr2.fLinearWheelSpeed = AdjustedVelocity;
            }
            else{
                IntersectionDistance = DistanceToIntersection_Robot1;
                TimeToIntersection = TicksToWait_Robot2 + TicksToWaitforSafedistance;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MinLinearSpeed;
                    Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
                    Real stop_time = abs(Time - TimeToIntersection);
                    ptr1.StopTurningTime = stop_time;
                }
                ptr1.fLinearWheelSpeed = AdjustedVelocity;
            }
        }

        }

    /* Reset the flag */
    ptr3.Intersection_flag = 0;
  
}
/***********************************************************************************************************/
/* Function to set the priority of the robot */
/***********************************************************************************************************/
void CThesisLoopFunctions::SetPriority(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2)
{
//    UInt16 i;
    
    CVector3 ptr1_next, ptr2_next;
    Real DistanceTotalR1, DistanceTotalR2;
    std::stack<CVector3>ptr1WaypointStackCopy, ptr2WaypointStackCopy;
    
    CVector3 ptr1_previous = ptr1.StartWaypoint;
    CVector3 ptr2_previous = ptr2.StartWaypoint;
    
    ptr1WaypointStackCopy = ptr1.WaypointStack;
    ptr2WaypointStackCopy = ptr2.WaypointStack;
    
        while(!ptr1WaypointStackCopy.empty())
        {
            ptr1_next = ptr1WaypointStackCopy.top();
            DistanceTotalR1 += CalculateDistance(ptr1_previous, ptr1_next);
            ptr1_previous = ptr1_next;
            ptr1WaypointStackCopy.pop();
    
        }
    
        while(!ptr2WaypointStackCopy.empty())
        {
            ptr2_next = ptr2WaypointStackCopy.top();
            DistanceTotalR2 += CalculateDistance(ptr2_previous, ptr2_next);
            ptr2_previous = ptr2_next;
            ptr2WaypointStackCopy.pop();
            
        }


    //if distance to target for robot 1 is greater than robot 2
    if(DistanceTotalR1 > DistanceTotalR2)
    {
        ptr1.Priority = ptr2.id_robot;
        ptr2.Priority = ptr1.id_robot;
    }
    else if(DistanceTotalR1< DistanceTotalR2)
    {
        ptr2.Priority = ptr1.id_robot;
        ptr2.id_robot = ptr2.id_robot;
    }
    // distance is equal
    else{
        ptr1.Priority = ptr1.id_robot;
        ptr2.Priority = ptr2.id_robot;
    }
}

/***********************************************************************************************************/
/* Function to avoid turning collision */
/***********************************************************************************************************/
void CThesisLoopFunctions::CheckTurningIntersection(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2)
{
    /* distance between two centers of circles */
    Real Distance_Centers= ((ptr1.StartWaypoint.GetX() - ptr2.StartWaypoint.GetX())*(ptr1.StartWaypoint.GetX() - ptr2.StartWaypoint.GetX()))+
                           ((ptr1.StartWaypoint.GetY() - ptr2.StartWaypoint.GetY()) * (ptr1.StartWaypoint.GetY() - ptr2.StartWaypoint.GetY()));
    
    /* distance between two radius */
    Real Distance_Radius_Sum = Safedistance;
    
    /* robots intersect or touch each other */
    if(Distance_Centers <= Distance_Radius_Sum)
    {
        SetPriority(ptr1, ptr2);
        
        if(ptr1.Priority > ptr2.Priority)
        {
            ptr1.StopTurningTime = ptr1.StopTurningTime + 0;
        }
        else{
            
            ptr2.StopTurningTime = ptr2.StopTurningTime  + ptr1.StopTurningTime + (ptr1.Intial_TurningWaitTime * 150);
        }
    }
    
}

/*****************************************************************************************************************/
/* Function to check if robot sart positions are too close */
/*****************************************************************************************************************/
void CThesisLoopFunctions::CheckRobotPlacement(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2)
{

    UInt16 time_for_safe_gap;
    
    Real distance = CalculateDistance(ptr1.StartWaypoint, ptr2.StartWaypoint);

    Real distance_to_goal1 = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);

    Real distance_to_goal2 = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);
    

    
    if(distance <= Robot_Gap_Distance)
    {
        
        if(distance_to_goal1 > distance_to_goal2)
        {
            time_for_safe_gap = GetTicksToWait(0.5, ptr2.fLinearWheelSpeed);
            ptr1.StopTurningTime = ptr1.StopTurningTime + (ptr2.Intial_TurningWaitTime * 150) + time_for_safe_gap;
        }
        else{
            time_for_safe_gap = GetTicksToWait(0.5, ptr1.fLinearWheelSpeed);
            ptr2.StopTurningTime = ptr2.StopTurningTime + (ptr1.Intial_TurningWaitTime * 150) + time_for_safe_gap;
        }
        
    }
    
}

/*****************************************************************************************************************/
/* Function to add a new way point */
/*****************************************************************************************************************/
void CThesisLoopFunctions::AddNewWayPoint(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2, UInt8 ptrIndex)
{
    CVector3 AddedWaypoint;
    Real diff, x, y;
    /* add waypoint for ptr1 */
    if(ptrIndex == 1)
    {
        // add a waypoint
        if(ptr1.TargetWaypoint.GetX() > ptr2.TargetWaypoint.GetX())
        {
            
            x = ptr2.TargetWaypoint.GetX() - 0.5;
            y = ptr2.TargetWaypoint.GetY();
            
            if(x < (Arena_Minimum + 0.4))
            {
                diff = abs((Arena_Minimum + 0.4) - ptr2.TargetWaypoint.GetX());
                x = ptr2.TargetWaypoint.GetX() - diff;
            }
            else if(x > (Arena_Maximum - 0.4))
            {
                
                diff = abs(ptr2.TargetWaypoint.GetX() - (Arena_Maximum - 0.4));
                x = ptr2.TargetWaypoint.GetX() + diff;
            }

            AddedWaypoint.Set(x, y, 0);
            diff= 0;
            x=0;
            y=0;
        }
        else{
            
            x = ptr2.TargetWaypoint.GetX() + 0.5;
            y = ptr2.TargetWaypoint.GetY();
            
            if(x < (Arena_Minimum + 0.4))
            {
                diff = abs((Arena_Minimum + 0.4) - ptr2.TargetWaypoint.GetX());
                x = ptr2.TargetWaypoint.GetX() - diff;
            }
            else if(x > (Arena_Maximum - 0.4))
            {
                
                diff = abs(ptr2.TargetWaypoint.GetX() - (Arena_Maximum - 0.4));
                x = ptr2.TargetWaypoint.GetX() + diff;
            }
            
            AddedWaypoint.Set(x, y, 0);
            diff= 0;
            x=0;
            y=0;
        }
        if(ptr1.WaypointCounter <= 5)
        {
            /* add a way point before the final goal */
            ptr1.WaypointStack.push(AddedWaypoint);
            
            ptr1.Waypoint_Added = true;
            ptr1.WaypointCounter++;
        }
    }
    /* add waypoint for ptr2 */
    else{
        // add a waypoint
        if(ptr1.TargetWaypoint.GetX() > ptr2.TargetWaypoint.GetX())
        {
            Real x = ptr1.TargetWaypoint.GetX() - 0.5;
            Real y = ptr1.TargetWaypoint.GetY();
            
            if(x < (Arena_Minimum + 0.4))
            {
                Real diff = abs((Arena_Minimum + 0.4) - ptr1.TargetWaypoint.GetX());
                x = ptr1.TargetWaypoint.GetX() - diff;
            }
            else if(x > (Arena_Maximum - 0.4))
            {
                
                Real diff = abs(ptr1.TargetWaypoint.GetX() - (Arena_Maximum - 0.4));
                x = ptr1.TargetWaypoint.GetX() + diff;
            }
            
            AddedWaypoint.Set(x, y, 0);
            diff= 0;
            x=0;
            y=0;
        }
        else{
            
            Real x = ptr1.TargetWaypoint.GetX() + 0.5;
            Real y = ptr1.TargetWaypoint.GetY();
            
            if(x < (Arena_Minimum + 0.4))
            {
                diff = abs((Arena_Minimum + 0.4) - ptr1.TargetWaypoint.GetX());
                x = ptr1.TargetWaypoint.GetX() - diff;
            }
            else if(x > (Arena_Maximum - 0.4))
            {
                
                diff = abs(ptr1.TargetWaypoint.GetX() - (Arena_Maximum - 0.4));
                x = ptr1.TargetWaypoint.GetX() + diff;
            }
            
            AddedWaypoint.Set(x, y, 0);
            diff= 0;
            x=0;
            y=0;
        }
        if(ptr2.WaypointCounter <= 5)
        {
            /* add a way point before the final goal */
            ptr2.WaypointStack.push(AddedWaypoint);

            ptr2.Waypoint_Added = true;
            ptr2.WaypointCounter++;
        }
    }
    
    
}

/*****************************************************************************************************************/
/* Function to check if robot start and target points are collinear */
/*****************************************************************************************************************/
void CThesisLoopFunctions::CheckCollinearity(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2,
                                             CFootBotThesis::IntersectionData &ptr3)
{
    
    CVector3 AddedWaypoint;
    Real Angle, Distance;
    
    ptr1.CollinearFlag = 0;
    ptr2.CollinearFlag = 0;
    
    if(ptr1.Waypoint_Added == 1)
    {
        ptr1.Waypoint_Added = 0;
    }
    else if(ptr2.Waypoint_Added ==1)
    {
        ptr2.Waypoint_Added = 0;
    }
    
    /* check if robot 1 start, target and robot 2 target points are collinear */
    Real Area_R1R2Goal =  ptr1.StartWaypoint.GetX() * (ptr2.TargetWaypoint.GetY() - ptr1.TargetWaypoint.GetY()) +
                        ptr2.TargetWaypoint.GetX() * (ptr1.TargetWaypoint.GetY() - ptr1.StartWaypoint.GetY()) +
                        ptr1.TargetWaypoint.GetX() * (ptr1.StartWaypoint.GetY() - ptr2.TargetWaypoint.GetY());
    
    
    /* check if robot 2 start, target and robot 1 target points are collinear */
    Real Area_R2R1Goal =  ptr1.TargetWaypoint.GetX() * (ptr2.StartWaypoint.GetY() - ptr2.TargetWaypoint.GetY()) +
    ptr2.StartWaypoint.GetX() * (ptr2.TargetWaypoint.GetY() - ptr1.TargetWaypoint.GetY()) +
    ptr2.TargetWaypoint.GetX() * (ptr1.TargetWaypoint.GetY() - ptr2.StartWaypoint.GetY());
    

    if(0 <= abs(Area_R1R2Goal) and abs(Area_R1R2Goal) <= 0.1)
    {
        ptr1.dist = 1;
        /* check if robot 2 target point is between robot 1 start and target point */
        
        Real distance_1 = CalculateDistance(ptr1.StartWaypoint, ptr2.TargetWaypoint);
        Real distance_2 = CalculateDistance(ptr1.TargetWaypoint, ptr2.TargetWaypoint);
        Real distance_Total = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);
        

        if(((distance_Total - 0.01) <= distance_1 + distance_2) and ((distance_Total + 0.01) >= distance_1 + distance_2))
        {
            ptr1.CollinearFlag = 1;
//            ptr2.CollinearFlag = 1;
            ptr1.Robot_ID_Collinearwith = ptr2.id_robot;
            AddNewWayPoint(ptr1, ptr2, 1);

        }
        
    }

    else if(0 <= abs(Area_R2R1Goal) and abs(Area_R2R1Goal) <= 0.1)
    {
         ptr1.dist = 2;
        /* check if robot 1 target point is between robot 2 start and target point */
        
        Real ddistance_1 = CalculateDistance(ptr2.StartWaypoint, ptr1.TargetWaypoint);
        Real ddistance_2 = CalculateDistance(ptr1.TargetWaypoint, ptr2.TargetWaypoint);
        Real ddistance_Total = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);
        

        if(((ddistance_Total- 0.01) <= ddistance_1 + ddistance_2) and ((ddistance_Total + 0.01) >= ddistance_1 + ddistance_2))
        {
            ptr1.CollinearFlag = 1;
//            ptr2.CollinearFlag = 1;
            ptr2.Robot_ID_Collinearwith = ptr1.id_robot;
            AddNewWayPoint(ptr1, ptr2, 2);

        }
    }
    
    else
    {
        ptr1.dist = 3;
        CheckRobotHeadingCourse(ptr1, ptr2, ptr3);
    }
    
}


/*************************************************************************************************************************************************************/
/* Function to check if robot course is very close */
/*************************************************************************************************************************************************************/
void CThesisLoopFunctions::CheckRobotHeadingCourse(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2, CFootBotThesis::IntersectionData &ptr3)
{
    
    Real RobotCourseAngle;
    CVector3 WaypointAdd;
    
    Real distanceR1 = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);
    Real distanceR2 = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);
    
    /* find the angle between lines */
    RobotCourseAngle = CalculateAngleBetweenRobotCourse(ptr1, ptr2);
    
    Real slopeline1 = (ptr1.TargetWaypoint.GetY()- ptr1.StartWaypoint.GetY())/(ptr1.TargetWaypoint.GetX()- ptr1.StartWaypoint.GetX());
    Real slopeline2 = (ptr2.TargetWaypoint.GetY()- ptr2.StartWaypoint.GetY())/(ptr2.TargetWaypoint.GetX()- ptr2.StartWaypoint.GetX());
    
    Real y_intercept1 = ptr1.TargetWaypoint.GetY() + (slopeline1 * ptr1.TargetWaypoint.GetX());
    Real y_intercept2 = ptr2.TargetWaypoint.GetY() + (slopeline1 * ptr2.TargetWaypoint.GetX());
    
    
    Find_Intersection(ptr1, ptr2, ptr3);
    
    if(0<= RobotCourseAngle and RobotCourseAngle <= OverlappingCourseAngle)
    {
        if(abs(y_intercept2 - y_intercept1) >=Safedistance and ptr3.Intersection_flag!=1)
        {
          /* Lines are parallel and sufficiently away */
        }
        else
        {
            ptr1.CollinearFlag = 1;
//            ptr2.CollinearFlag = 1;
            if(ptr1.GoingToNest)
            {
                
                AddNewWayPoint(ptr1, ptr2, 2);
            }
            else if(ptr2.GoingToNest)
            {
                
                AddNewWayPoint(ptr1, ptr2, 1);
            }
            else
            {
                
                Real distance1 = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);
                Real distance2 = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);
                
                // add waypoint for 2nd robot
                if(distance1 < distance2)
                {
                    ptr2.dist = distance1;
                    AddNewWayPoint(ptr1, ptr2, 2);
                    
                }
                // add waypoint for 1st robot
                else
                {
                    ptr2.dist = distance2;
                    AddNewWayPoint(ptr1, ptr2, 1);
                    
                }
            }
            
        }
        ptr3.Intersection_flag = 0;
    }
    
}



/*************************************************************************************************************************************************************/
/* Function to check if robot course is very close */
/*************************************************************************************************************************************************************/
void CThesisLoopFunctions::DistanceBetweenCourse(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2)
{
    
    CVector3 WaypointAdd;
    /* find if target point is too close to the course of the robot */
   
    /* linear equation parameters for Robot 1 */
    Real A1 = (ptr1.StartWaypoint.GetY() - ptr1.TargetWaypoint.GetY());
    Real B1 = (ptr1.TargetWaypoint.GetX() - ptr1.StartWaypoint.GetX());
    Real C1 = ((ptr1.StartWaypoint.GetX() * ptr1.TargetWaypoint.GetY()) - (ptr1.TargetWaypoint.GetX()*ptr1.StartWaypoint.GetY()));
          
    /* linear equation parameters for Robot 2 */
    Real A2 = (ptr2.StartWaypoint.GetY() - ptr2.TargetWaypoint.GetY());
    Real B2 = (ptr2.TargetWaypoint.GetX() - ptr2.StartWaypoint.GetX());
    Real C2 = ((ptr2.StartWaypoint.GetX() * ptr2.TargetWaypoint.GetY()) - (ptr2.TargetWaypoint.GetX() * ptr2.StartWaypoint.GetY()));
          
          
    /* check if current point of Robot 1 is close to robot 2 course */
          
    Real distR1 = abs(A2*ptr1.CurrentPos.GetX() + B2*ptr1.CurrentPos.GetY() + C2)/(sqrt(A2*A2 + B2*B2));
          
//    Real distR2 = abs(A1*ptr2.CurrentPos.GetX() + B1*ptr2.CurrentPos.GetY() + C1)/(sqrt(A1*A1 + B1*B1));
    
    /* target point of robot 1 is close to Robot 2 course*/
    if(distR1 < Safedistance)
    {
        WaitCounter = 50;
        // add waypoint for 2nd robot
        if(ptr1.GoingToNest)
        {
            ptr2.CollinearFlag = 1;

        }
        // add waypoint for 1st robot
        else if(ptr2.GoingToNest)
        {
            ptr1.CollinearFlag = 1;
  
        }
        else{
            Real distance1 = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);
            Real distance2 = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);
            
            // add waypoint for 2nd robot
            if(distance1 < distance2)
            {
                ptr2.CollinearFlag = 1;

            }
            // add waypoint for 1st robot
            else{
                ptr1.CollinearFlag = 1;

            }
        }
        /* add waypoint for Robot 2 */
       
            
    }
    
}

/*****************************************************************************************************************/
/* Function to find the angle between two lines */
/*****************************************************************************************************************/
Real CThesisLoopFunctions::CalculateAngleBetweenRobotCourse(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2)
{

    CDegrees headingToTargetR1 = ToDegrees((ptr1.TargetWaypoint - ptr1.StartWaypoint).GetZAngle());
    
    CDegrees headingToTargetR2 = ToDegrees((ptr2.TargetWaypoint - ptr2.StartWaypoint).GetZAngle());
    
    /* get the current heading angle of the robot */
    Real AngleRobotCourse = abs(180 - (abs(headingToTargetR1.GetValue()) + abs(headingToTargetR2.GetValue())));
    
    return AngleRobotCourse;

}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CThesisLoopFunctions, "thesis_loop_functions")
