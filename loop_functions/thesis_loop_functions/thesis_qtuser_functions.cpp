#include "thesis_qtuser_functions.h"
#include "thesis_loop_functions.h"

/****************************************/
/****************************************/

CThesisQTUserFunctions::CThesisQTUserFunctions() :
   m_cTrajLF(dynamic_cast<CThesisLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
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

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CThesisQTUserFunctions, "thesis_qtuser_functions")
