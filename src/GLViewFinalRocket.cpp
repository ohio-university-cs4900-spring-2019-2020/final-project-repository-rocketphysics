#include "GLViewFinalRocket.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "AftrGLRendererBase.h"

//If we want to use way points, we need to include this.
#include "FinalRocketWayPoints.h"
#include "PhysicsModule.h"
#include "WOPhysXActor.h"
#include "WORocket.h"
#include "Mat4.h"


using namespace Aftr;
using namespace physx;

GLViewFinalRocket* GLViewFinalRocket::New( const std::vector< std::string >& args )
{
   GLViewFinalRocket* glv = new GLViewFinalRocket( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewFinalRocket::GLViewFinalRocket( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewFinalRocket::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewFinalRocket::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewFinalRocket::onCreate()
{
   //GLViewFinalRocket::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1

}


GLViewFinalRocket::~GLViewFinalRocket()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewFinalRocket::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.
   //this->rocketLauncher->getModel()->setDisplayMatrix(this->cam->getDisplayMatrix());
   this->rocketLauncher->setPosition(this->cam->getPosition() - this->cam->getNormalDirection() * (0.2f) + this->cam->getLookDirection() * (0.1f));
   rocketLauncher->getModel()->setDisplayMatrix(this->cam->getDisplayMatrix() * Mat4::rotateIdentityMat(Vector(0, -1, 0), Aftr::PI / 1.7));
   this->physicsEngine->simulate();

}


void GLViewFinalRocket::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewFinalRocket::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
   this->rocketLauncher->setPosition(this->cam->getPosition() - this->cam->getNormalDirection() * (0.2f) + this->cam->getLookDirection() * (0.1f));
   rocketLauncher->getModel()->setDisplayMatrix(this->cam->getDisplayMatrix() * Mat4::rotateIdentityMat(Vector(0, -1, 0), Aftr::PI / 1.7));
}


void GLViewFinalRocket::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewFinalRocket::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewFinalRocket::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if( key.keysym.sym == SDLK_f )
   {
	   this->shootRocket();
   }
   if (key.keysym.sym == SDLK_r)
   {
	   this->resetBoxes();
   }
   if (key.keysym.sym == SDLK_t)
   {
	   this->createTrigger();
   }
}


void GLViewFinalRocket::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void Aftr::GLViewFinalRocket::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_water+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_dust+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_winter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/early_morning+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_afternoon+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy3+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day2+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_deepsun+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_evening+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning2+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_noon+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_warp+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_Hubble_Nebula+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_gray_matter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_easter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_hot_nebula+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_ice_field+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_lemon_lime+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_milk_chocolate+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_solar_bloom+6.jpg" );
   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_thick_rb+6.jpg" );

   float ga = 0.1f; //Global Ambient Light level for this module
   ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
   WOLight* light = WOLight::New();
   light->isDirectionalLight( true );
   light->setPosition( Vector( 0, 0, 100 ) );
   //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
   //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
   light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
   light->setLabel( "Light" );
   worldLst->push_back( light );

   //Create the SkyBox
   WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
   wo->setPosition( Vector( 0,0,0 ) );
   wo->setLabel( "Sky Box" );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   worldLst->push_back( wo );

   ////Create the infinite grass plane (the floor)
   wo = WO::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
   wo->setPosition( Vector( 0, 0, 0 ) );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
   grassSkin.getMultiTextureSet().at( 0 )->setTextureRepeats( 50.0f );
   grassSkin.setAmbient( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Color of object when it is not in any light
   grassSkin.setDiffuse( aftrColor4f( 1.0f, 1.0f, 1.0f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object)
   grassSkin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
   grassSkin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
   wo->setLabel( "Grass" );
   worldLst->push_back( wo );
   
   createFinalRocketWayPoints();

   this->gameSounds = SoundModule::init();
   this->gameSounds->play_sound_2D("../mm/sounds/bensound-jazzyfrenchy.ogg", true, false, true);
   this->gameSounds->get_sound_2D().at(0)->setVolume(0.3f);

   this->physicsEngine = PhysicsModule::New();
   PxMaterial* gMaterial = this->physicsEngine->getPhysics()->createMaterial(.5f, .5f, .6f);
   PxRigidStatic* groundPlane = PxCreatePlane(*this->physicsEngine->getPhysics(), PxPlane(PxVec3(0, 0, 1), 0), *gMaterial);
   this->physicsEngine->getScene()->addActor(*groundPlane);

   this->rocketLauncher = WO::New("../mm/models/Pencil/Dwarf_Pencil.obj", Vector(0.1f, 0.1f, 0.1f));
   this->rocketLauncher->getModel()->setDisplayMatrix(this->cam->getDisplayMatrix());
   this->rocketLauncher->setPosition(this->cam->getPosition() - (this->cam->getNormalDirection() * (0.2f)) + (this->cam->getLookDirection() * 0.2f));
   rocketLauncher->getModel()->setDisplayMatrix(Mat4::rotateIdentityMat(Vector(0, -1, 0), Aftr::PI / 1.7));
   worldLst->push_back(rocketLauncher);

   for (int i = 0; i < 3; i++) {
	   PxTransform trans;
	   if (i == 0)
		  trans = PxTransform(PxVec3(25, 25, 3));
	   else if (i == 1)
		   trans = PxTransform(PxVec3(30, 25, 3));
	   else
		   trans = PxTransform(PxVec3(27.5, 25, 8));
	   PxShape* shape = this->physicsEngine->physics->createShape(PxBoxGeometry(PxVec3(2.0f, 2.0f, 2.0f)), *this->physicsEngine->physics->createMaterial(.5f, .3f, .45f));
	   PxRigidDynamic* actor = PxCreateDynamic(*this->physicsEngine->physics, trans, *shape, 5.0f);
	   WOPhysXActor* box = WOPhysXActor::New(actor, shinyRedPlasticCube, Vector(1, 1, 1));
	   if (i == 0)
		   box->setPosition(Vector(25, 25, 3));
	   else if (i==1) 
		   box->setPosition(Vector(30, 25, 3));
	   else
		   box->setPosition(Vector(27.5, 25, 8));
	   box->physxActor->userData = box;
	   worldLst->push_back(box);
	   this->physicsEngine->scene->addActor(*box->physxActor);
	   
	   this->boxes.insert(std::pair(box, numBoxes));
	   numBoxes++;
   }
}


void GLViewFinalRocket::createFinalRocketWayPoints()
{
   // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
   WayPointParametersBase params(this);
   params.frequency = 5000;
   params.useCamera = true;
   params.visible = true;
   WOWayPointSpherical* wayPt = WOWP1::New( params, 3 );
   wayPt->setPosition( Vector( 50, 0, 3 ) );
   worldLst->push_back( wayPt );
}

void GLViewFinalRocket::shootRocket() {
	Vector position = this->cam->getPosition();
	Vector direction = this->cam->getLookDirection();
	Vector spawnLoc = Vector(position.x + (2.0f * direction.x), position.y + (2.0f * direction.y), position.z + (2.0f * direction.z)) - (this->cam->getNormalDirection() * 0.2f);

	PxTransform trans = PxTransform(PxVec3(spawnLoc.x, spawnLoc.y, spawnLoc.z));
	PxShape* shape = this->physicsEngine->physics->createShape(PxSphereGeometry(0.2f), *this->physicsEngine->physics->createMaterial(.5f, .3f, .45f));
	shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, false);
	PxRigidDynamic* actor = PxCreateDynamic(*this->physicsEngine->physics, trans, *shape, 5.0f);
	std::cout << actor->getMass() << std::endl;

	WORocket* wo = WORocket::New(actor, "../mm/models/sphere.dae", Vector(0.1f, 0.1f, 0.1f));
	wo->setPosition(spawnLoc);
	/*Mat4 pose;
	pose = Mat4::rotateIdentityMat(Vector(1, 0, 0), Aftr::PI / 2);
	wo->getModel()->setDisplayMatrix(pose);*/
	wo->physxActor->userData = wo;
	worldLst->push_back(wo);

	std::cout << "Shot rocket from position " << wo->getPosition() 
		<< "\nIn direction " << direction << std::endl;
	this->physicsEngine->scene->addActor(*wo->physxActor);

	PxVec3 pvec_dir = PxVec3(direction.x * 300.0f, direction.y * 300.0f, direction.z * 300.0f + 15.0f);
	wo->physxActor->addForce(pvec_dir);

}

void GLViewFinalRocket::resetBoxes() {
	for (auto& i : this->boxes) {
		if (i.second == 0)
			i.first->physxActor->setGlobalPose(PxTransform(PxVec3(25, 25, 3)));
		else if (i.second == 1)
			i.first->physxActor->setGlobalPose(PxTransform(PxVec3(30, 25, 3)));
		else
			i.first->physxActor->setGlobalPose(PxTransform(PxVec3(27.5, 25, 8)));
	}
}

void GLViewFinalRocket::createTrigger() {
	PxShape* triggerShape;

}
