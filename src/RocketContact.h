#pragma once

#include "PxPhysicsAPI.h"
#include "PhysicsModule.h"
#include "WOPhysXActor.h"
#include "WORocket.h"

using namespace physx;

namespace Aftr {

	class RocketContact : public PxSimulationEventCallback {

	public:
		RocketContact();
		~RocketContact();
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count);
		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs);
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
	};

} // Namespace Aftr