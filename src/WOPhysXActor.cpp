#include "WOPhysXActor.h"

#include <iostream>

using namespace Aftr;
using namespace physx;

WOPhysXActor* WOPhysXActor::New(physx::PxRigidDynamic* pxActor,
	const std::string& path,
	Vector scale,
	MESH_SHADING_TYPE shadingType)
{
	WOPhysXActor* wo = new WOPhysXActor(pxActor);
	wo->onCreate(path, scale, shadingType);
	return wo;
}

WOPhysXActor::WOPhysXActor(physx::PxRigidDynamic* pxActor) : IFace(this), WO() {
	this->physxActor = pxActor;
}

void WOPhysXActor::onCreate(const std::string& path,
	Vector scale,
	MESH_SHADING_TYPE shadingType) {
	WO::onCreate(path, scale, shadingType);
}

void WOPhysXActor::onTrigger() {
	std::cout << "Triggered!\n";
	delete this;
}

void WOPhysXActor::onContact() {
	std::cout << "Contacted!\n";
	delete this;
}

void WOPhysXActor::setDisplayMatrix(Mat4 matrix) {
	WO::getModel()->setDisplayMatrix(matrix);
}
