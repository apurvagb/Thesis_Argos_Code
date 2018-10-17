#include "thesis_qtuser_functions.h"
#include "thesis_loop_functions.h"

/****************************************/
/****************************************/

CThesisQTUserFunctions::CThesisQTUserFunctions() :
   m_cTrajLF(dynamic_cast<CThesisLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
        RegisterUserFunction<CThesisQTUserFunctions,CFootBotEntity>(&CThesisQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CThesisQTUserFunctions::DrawInWorld() {
   /* Go through all the robot waypoints and draw them */
   for(CThesisLoopFunctions::TWaypointMap::const_iterator it = m_cTrajLF.GetWaypoints().begin();
       it != m_cTrajLF.GetWaypoints().end();
       ++it) {
      DrawWaypoints(it->second);
   }
}

/****************************************/
/****************************************/

void CThesisQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]));
         ++unStart;
         ++unEnd;
      }
   }
}

void CThesisQTUserFunctions::Draw(CFootBotEntity& c_entity) {
    /* The position of the text is expressed wrt the reference point of the footbot
     * For a foot-bot, the reference point is the center of its base.
     * See also the description in
     * $ argos3 -q foot-bot
     */
    DrawText(CVector3(0.0, 0.0, 0.3),   // position
             c_entity.GetId().c_str()); // text
}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CThesisQTUserFunctions, "thesis_qtuser_functions")
