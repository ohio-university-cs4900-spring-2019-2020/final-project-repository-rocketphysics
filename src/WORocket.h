#pragma once

#include "WO.h"
#include "Model.h"
#include "PhysicsModule.h"
#include "PxPhysicsAPI.h"

namespace Aftr {
	class PhysicsModule;

	class WORocket : public WO {
	public:
		WOMacroDeclaration(WORocket, WO);
		static WORocket* New(physx::PxRigidDynamic* pxActor = nullptr,
			const std::string& path = "../mm/models/sphere.dae",
			Vector scale = Vector(1, 1, 1),
			MESH_SHADING_TYPE shadingType = MESH_SHADING_TYPE::mstAUTO);
		virtual void onCreate(const std::string& path,
			Vector scale = Vector(1, 1, 1),
			MESH_SHADING_TYPE shadingType = MESH_SHADING_TYPE::mstAUTO);
		void setDisplayMatrix(Mat4 matrix);
		physx::PxRigidDynamic* physxActor;

		int index;
	protected:
		WORocket(physx::PxRigidDynamic* pxActor);
	};

} // Namespace Aftr