#ifndef THESIS_QTUSER_FUNCTIONS_H
#define THESIS_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CThesisLoopFunctions;

class CThesisQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CThesisQTUserFunctions();

   virtual ~CThesisQTUserFunctions() {}

   virtual void DrawInWorld();

private:

   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);
   void Draw(CFootBotEntity& c_entity);

private:

   CThesisLoopFunctions& m_cTrajLF;

};

#endif
