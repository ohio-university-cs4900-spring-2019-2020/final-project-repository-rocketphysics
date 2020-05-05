#include "RocketContact.h"
#include "GLViewFinalRocket.h"
#include "WorldContainer.h"

using namespace Aftr;
using namespace physx;

RocketContact::RocketContact() {

}
RocketContact::~RocketContact() {

}
void RocketContact::onTrigger(PxTriggerPair* pairs, PxU32 count) {
	for (PxU32 i = 0; i < count; i++)
	{
		std::cout << "Trigger: " << pairs[i].otherActor->getName() << ", " << pairs[i].triggerActor->getName() << std::endl;
		if (pairs[i].otherActor->getName() == "Grass" && pairs[i].triggerActor->getName() == "Rocket_TS") {
			PxTransform trans = pairs[i].triggerActor->getGlobalPose();
			((GLViewFinalRocket*)ManagerGLView::getGLView())->createExplosion(Vector(trans.p.x, trans.p.y, trans.p.z));
		}
	}
}

void RocketContact::onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) {
	for (PxU32 i = 0; i < nbPairs; i++)
	{
		const PxContactPair& cp = pairs[i];

		//if (cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
		//{
			//if ((pairHeader.actors[0]->getName() == "Rocket_SS") ||
			//	(pairHeader.actors[1]->getName() == "Rocket_SS"))
			//{
				std::cout << "Contact: " << pairHeader.actors[0]->getName() << ", " << pairHeader.actors[0]->getName() << std::endl;
			//}
		//}
	}
}
