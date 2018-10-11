#ifndef THESIS_LOOP_FUNCTIONS_H
#define THESIS_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/thesis_sample/thesis.h>

using namespace argos;

class CThesisLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;
   
public:

   virtual ~CThesisLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void PostStep();
    
    virtual void PreStep();
    
   inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }
    
   void Find_Intersection(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData& ptr2,
                          CFootBotThesis::IntersectionData& ptr3);

   Real CalculateDistance(CVector3 cPosition1, CVector3 cPosition2);

   UInt16 GetTicksToWait(Real dist, Real speed);

   void IntersectionCollisionCheck(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2,
                                    CFootBotThesis::IntersectionData &ptr3);

   void SetPriority(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2);
    
   void CheckTurningIntersection(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2);
    
   void CheckRobotPlacement(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2);
    
   void collinear(CFootBotThesis::RobotData& ptr1, CFootBotThesis::RobotData &ptr2);
    

private:
    
    bool Done_flag;
    UInt16 SimulatorTicksperSec;
    const Real FOOTBOT_RADIUS                   = 0.085036758f;
    const Real FOOTBOT_INTERWHEEL_DISTANCE      = 0.14f;
    const Real Safedistance = (2 * FOOTBOT_RADIUS);
    const Real MaxLinearSpeed = 10.0f;
    const Real Robot_Gap_Distance = 0.5f;
};

#endif
