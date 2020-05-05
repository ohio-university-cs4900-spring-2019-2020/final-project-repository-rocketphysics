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
#include <PxSceneDesc.h>

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
   // Update first-person rocket launcher model
   this->rocketLauncher->setPosition(this->cam->getPosition() - this->cam->getNormalDirection() * (0.2f) + this->cam->getLookDirection() * (0.1f));
   rocketLauncher->getModel()->setDisplayMatrix(this->cam->getDisplayMatrix() * Mat4::rotateIdentityMat(Vector(0, -1, 0), Aftr::PI / 1.7));
   // Simulate physics
   this->physicsEngine->simulate();
   // Sync rocket and trigger objects
   for (auto& i : liveRockets) {
	   i.second->setPosition(i.first->getPosition());
	   i.second->setDisplayMatrix(i.first->getDisplayMatrix());
   }
   for (auto& i : objects) {
	   i.second->setPosition(i.first->getPosition());
	   i.second->setDisplayMatrix(i.first->getDisplayMatrix());
   }
   //this->cam->setPosition(player->getPosition());

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
   if (key.keysym.sym == SDLK_e)
   {
	   Vector position = this->cam->getPosition();
	   Vector direction = this->cam->getLookDirection();
	   float dist = 5.5f;
	   Vector spawnLoc = Vector(position.x + (dist * direction.x), position.y + (dist * direction.y), position.z + (dist * direction.z));
	   this->createExplosion(spawnLoc);
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

   // Init sound
   this->gameSounds = SoundModule::init();
   this->gameSounds->play_sound_2D("../mm/sounds/bensound-jazzyfrenchy.ogg", true, false, true);
   this->gameSounds->get_sound_2D().at(0)->setVolume(0.09f);

   // Init physics
   this->physicsEngine = PhysicsModule::New();
   PxMaterial* gMaterial = this->physicsEngine->getPhysics()->createMaterial(.5f, .5f, .6f);
   PxRigidStatic* groundPlane = PxCreatePlane(*this->physicsEngine->getPhysics(), PxPlane(PxVec3(0, 0, 1), 0), *gMaterial);
   groundPlane->setName("Grass");
   this->physicsEngine->scene->addActor(*groundPlane);
   //this->physicsEngine->scene->setFlag(PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS, false);

   // Add simulation callback to the engine
   this->contact = new RocketContact();
   this->physicsEngine->scene->setSimulationEventCallback(this->contact);

   // First-person "rocket launcher" model
   this->rocketLauncher = WO::New("../mm/models/Pencil/Dwarf_Pencil.obj", Vector(0.1f, 0.1f, 0.1f));
   this->rocketLauncher->getModel()->setDisplayMatrix(this->cam->getDisplayMatrix());
   this->rocketLauncher->setPosition(this->cam->getPosition() - (this->cam->getNormalDirection() * (0.2f)) + (this->cam->getLookDirection() * 0.2f));
   rocketLauncher->getModel()->setDisplayMatrix(Mat4::rotateIdentityMat(Vector(0, -1, 0), Aftr::PI / 1.7));
   worldLst->push_back(rocketLauncher);

   for (int i = 0; i < 3; i++) {
	   PxTransform trans2;
	   if (i == 0)
		   trans2 = PxTransform(PxVec3(25, 25, 3));
	   else if (i == 1)
		   trans2 = PxTransform(PxVec3(30, 25, 3));
	   else
		   trans2 = PxTransform(PxVec3(27.5, 25, 8));
	   PxShape* shape2 = this->physicsEngine->physics->createShape(PxBoxGeometry(PxVec3(2.0f, 2.0f, 2.0f)), *this->physicsEngine->physics->createMaterial(.5f, .3f, .45f));
	   shape2->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true); // Flags
	   shape2->setFlag(PxShapeFlag::eTRIGGER_SHAPE, false);
	   PxRigidDynamic* actor2 = PxCreateDynamic(*this->physicsEngine->physics, trans2, *shape2, 5.0f);
	   actor2->setName("SS_Box");
	   actor2->setMass(50);
	   WOPhysXActor* box2 = WOPhysXActor::New(actor2, shinyRedPlasticCube, Vector(1, 1, 1));
	   if (i == 0)
		   box2->setPosition(Vector(25, 25, 3));
	   else if (i == 1)
		   box2->setPosition(Vector(30, 25, 3));
	   else
		   box2->setPosition(Vector(27.5, 25, 8));
	   box2->setLabel("Box2");
	   box2->physxActor->userData = box2;
	   box2->getModel()->renderBBox = true;
	   std::cout << box2->getModel()->getBoundingBox() << std::endl;
	   worldLst->push_back(box2);
	   this->physicsEngine->scene->addActor(*box2->physxActor);

	   // Trigger shape box
	   PxTransform trans;
	   if (i == 0)
		   trans = PxTransform(PxVec3(25, 25, 3));
	   else if (i == 1)
		   trans = PxTransform(PxVec3(30, 25, 3));
	   else
		   trans = PxTransform(PxVec3(27.5, 25, 8));
	   PxShape* shape = this->physicsEngine->physics->createShape(PxBoxGeometry(PxVec3(2.2f, 2.2f, 2.2f)), *this->physicsEngine->physics->createMaterial(.5f, .3f, .45f));
	   shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false); // Flags
	   shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
	   PxRigidDynamic* actor = PxCreateDynamic(*this->physicsEngine->physics, trans, *shape, 1.0f);
	   actor->setName("TS_Box");
	   WOPhysXActor* box = WOPhysXActor::New(actor, shinyRedPlasticCube, Vector(1, 1, 1));
	   if (i == 0)
		   box->setPosition(Vector(25, 25, 3));
	   else if (i == 1)
		   box->setPosition(Vector(30, 25, 3));
	   else
		   box->setPosition(Vector(27.5, 25, 8));
	   box->setLabel("Box2");
	   box->physxActor->userData = box;
	   worldLst->push_back(box);
	   this->physicsEngine->scene->addActor(*box->physxActor);

	   this->objects.insert(std::pair(box2, box));
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

// Creates simulation shape and trigger shape and maps them together
void GLViewFinalRocket::shootRocket() {
	Vector position = this->cam->getPosition();
	Vector direction = this->cam->getLookDirection();
	float dist = 2.5f;
	Vector spawnLoc = Vector(position.x + (dist * direction.x), position.y + (dist * direction.y), position.z + (dist * direction.z)) - (this->cam->getNormalDirection() * 0.2f);

	std::cout << "Shot rocket from position " << position
	<< "\nIn direction " << direction << std::endl;

	this->liveRockets.insert(std::pair(startupRocket(spawnLoc, direction), startupTrigger(spawnLoc)));
}


// Creates a spherical physics object eith eSIMULATION_SHAPE raised
WORocket* GLViewFinalRocket::startupRocket(Vector spawnLoc, Vector direction) {
	PxTransform trans = PxTransform(PxVec3(spawnLoc.x, spawnLoc.y, spawnLoc.z));
	PxShape* shape = this->physicsEngine->physics->createShape(PxSphereGeometry(0.2f), *this->physicsEngine->physics->createMaterial(.5f, .3f, .45f));
	shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, false); // Flags
	shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
	PxRigidDynamic* rocketActor = PxCreateDynamic(*this->physicsEngine->physics, trans, *shape, 5.0f);
	rocketActor->setName("Rocket_SS");
	rocketActor->setMass(4);

	WORocket* wo = WORocket::New(rocketActor, "../mm/models/sphere.dae", Vector(0.1f, 0.1f, 0.1f));
	wo->isVisible = false;
	wo->setPosition(spawnLoc);
	wo->setLabel("Rocket");
	wo->physxActor->userData = wo;
	worldLst->push_back(wo);

	this->physicsEngine->scene->addActor(*wo->physxActor);

	float force = 9000.0f;
	PxVec3 pvec_dir = PxVec3(direction.x * force, direction.y * force, direction.z * force + 100.0f);
	wo->physxActor->addForce(pvec_dir);
	return wo;
}

// Creates a spherical physics object eith eTRIGGER_SHAPE raised 
WOPhysXActor* GLViewFinalRocket::startupTrigger(Vector spawnLoc) {
	PxTransform trans = PxTransform(PxVec3(spawnLoc.x, spawnLoc.y, spawnLoc.z));
	PxShape* shape = this->physicsEngine->physics->createShape(PxSphereGeometry(0.4f), *this->physicsEngine->physics->createMaterial(.5f, .3f, .45f));
	shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false); // Flags
	shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
	PxRigidDynamic* actor = PxCreateDynamic(*this->physicsEngine->physics, trans, *shape, 5.0f);
	actor->setName("Rocket_TS");

	WOPhysXActor* wo = WOPhysXActor::New(actor, "../mm/models/sphere.dae", Vector(0.1f, 0.1f, 0.1f));
	wo->setPosition(spawnLoc);
	wo->setLabel("RocketTrigger");
	wo->physxActor->userData = wo;
	worldLst->push_back(wo);

	this->physicsEngine->scene->addActor(*wo->physxActor);

	return wo;
}

// Resets the 3 physics boxes to their original stack
void GLViewFinalRocket::resetBoxes() {
	int index = 0;
	for (auto& i : this->objects) {
		if (index == 0) {
			i.first->physxActor->setGlobalPose(PxTransform(PxVec3(25, 25, 3)));
		}
		else if (index == 1) {
			i.first->physxActor->setGlobalPose(PxTransform(PxVec3(30, 25, 3)));
		}
		else {
			i.first->physxActor->setGlobalPose(PxTransform(PxVec3(27.5, 25, 8)));
		}
		i.first->physxActor->clearForce();
		i.first->physxActor->setLinearVelocity(PxVec3(0,0,0));
		i.first->physxActor->setAngularVelocity(PxVec3(0, 0, 0));
		index++;
	}
}

void GLViewFinalRocket::createExplosion(Vector center) {
	std::cout << "Exploding:\n";
	this->gameSounds->play_sound_3D("../mm/sounds/explosion.wav", center, false, false, true);
	this->gameSounds->get_sound_3D().back()->setVolume(10.0f);
	this->gameSounds->get_sound_3D().back()->setMinDistance(15.0f);
	PxU32 numActors = 0;
	PxActor** activeActors = this->physicsEngine->scene->getActiveActors(numActors);
	for (PxU32 i = 0; i < numActors; i++) {
		WOPhysXActor* wo = static_cast<WOPhysXActor*>(activeActors[i]->userData);
		if (wo != nullptr && wo->physxActor != nullptr) {
			Vector objectCenter = wo->getPosition();
			Vector explosion = objectCenter - center;
			if ((pow(explosion.x, 2.0f) + pow(explosion.y, 2.0f) + pow(explosion.y, 2.0f)) < 625.0f) {
				std::cout << " " << wo->physxActor->getMass() << ": " << objectCenter << ", " << explosion << std::endl;
				float force = 17000.0f;
				wo->physxActor->addForce(PxVec3(explosion.x * force, explosion.y * force, explosion.z * force));
			}
		}
	}
}
