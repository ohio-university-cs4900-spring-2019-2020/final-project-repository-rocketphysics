#pragma once

#include "PxPhysicsAPI.h"
#include <map>
#include "WOPhysXActor.h"
#include "RocketContact.h"

namespace Aftr {

	class ModelDataSharedID;
	class WOPhysXActor;
	class RocketContact;
	
	class PhysicsModule {
	public:
		PhysicsModule();
		~PhysicsModule();
		static PhysicsModule* New();
		void simulate();
		physx::PxPhysics* getPhysics();
		physx::PxScene* getScene();
		physx::PxFoundation* getFoundation();
		physx::PxPhysics* physics;
		physx::PxScene* scene;
		physx::PxFoundation* foundation;
		physx::PxSimulationEventCallback* callback;

	protected:
		
		physx::PxPvd* pvd;
		physx::PxU32 version = PX_PHYSICS_VERSION;
		physx::PxCooking* cooking;
		physx::PxDefaultCpuDispatcher* dispatch;
		physx::PxMaterial* material;
		physx::PxDefaultAllocator allocator;
		physx::PxDefaultErrorCallback error;
		physx::PxPvdSceneClient* client;		
	};

} // Namespace Aftr