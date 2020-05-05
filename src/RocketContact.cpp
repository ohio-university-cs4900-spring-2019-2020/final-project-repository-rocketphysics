#include "RocketContact.h"


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
		//std::cout << pairs[i].triggerShape->getName() << std::endl;
		// ignore pairs when shapes have been deleted
		/*if (pairs[i].flags & (PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER |
			PxTriggerPairFlag::eREMOVED_SHAPE_OTHER))
			continue;

		if ((&pairs[i].otherShape->getActor() == rocketActor) ||
			(&pairs[i].triggerShape->getActor() == rocketActor))
		{
			int i = 0;
		}*/
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
