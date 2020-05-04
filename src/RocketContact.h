#pragma once

#include "PxPhysicsAPI.h"
#include "PhysicsModule.h"

namespace Aftr {

	class RocketContact : public PxSimulationEventCallback {

	public:

		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count);
	protected:
		WORocket(physx::PxRigidDynamic* pxActor);
	};

} // Namespace Aftr