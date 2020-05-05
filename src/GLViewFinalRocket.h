#pragma once

#include "GLView.h"
#include "PxPhysicsAPI.h"
#include "PhysicsModule.h"
#include "SoundModule.h"
#include "WORocket.h"
#include "WOPhysXActor.h"
#include "RocketContact.h"

namespace Aftr
{
   class Camera;
   class PhysicsModule;

/**
   \class GLViewFinalRocket
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewFinalRocket : public GLView
{
public:
   static GLViewFinalRocket* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewFinalRocket();
   virtual void updateWorld(); ///< Called once per frame
   virtual void loadMap(); ///< Called once at startup to build this module's scene
   virtual void createFinalRocketWayPoints();
   virtual void onResizeWindow( GLsizei width, GLsizei height );
   virtual void onMouseDown( const SDL_MouseButtonEvent& e );
   virtual void onMouseUp( const SDL_MouseButtonEvent& e );
   virtual void onMouseMove( const SDL_MouseMotionEvent& e );
   virtual void onKeyDown( const SDL_KeyboardEvent& key );
   virtual void onKeyUp( const SDL_KeyboardEvent& key );

   void shootRocket();
   void resetBoxes();
   void createExplosion(Vector center);
   WORocket* startupRocket(Vector spawnLoc, Vector direction);
   WOPhysXActor* startupTrigger(Vector spawnLoc);

   PhysicsModule* physicsEngine;
   SoundModule* gameSounds;

protected:
   GLViewFinalRocket( const std::vector< std::string >& args );
   virtual void onCreate();
   WO* rocketLauncher;
   WOPhysXActor* player;
   std::map<WOPhysXActor*, int> boxes;
   int numBoxes = 0;
   std::map<WORocket*, WOPhysXActor*> liveRockets;
   std::map<WOPhysXActor*, WOPhysXActor*> objects;
   RocketContact* contact;
};

/** \} */

} //namespace Aftr
