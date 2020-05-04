#include "WORocket.h"
#include "GLViewFinalRocket.h"

#include <iostream>

using namespace Aftr;
using namespace physx;

WORocket* WORocket::New(physx::PxRigidDynamic* pxActor,
	const std::string& path,
	Vector scale,
	MESH_SHADING_TYPE shadingType)
{
	WORocket* wo = new WORocket(pxActor);
	wo->onCreate(path, scale, shadingType);
	return wo;
}

WORocket::WORocket(physx::PxRigidDynamic* pxActor) : IFace(this), WO() {
	this->physxActor = pxActor;
}

void WORocket::onCreate(const std::string& path,
	Vector scale,
	MESH_SHADING_TYPE shadingType) {
	WO::onCreate(path, scale, shadingType);
}

void WORocket::setDisplayMatrix(Mat4 matrix) {
	WO::getModel()->setDisplayMatrix(matrix);
}
