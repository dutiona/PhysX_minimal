#pragma once

#include <memory>
#include <vector>
#include <array>

#include "vendor/PhysX_wrapper.h"

namespace impl {
	class MySimulationEventCallback;
}

namespace PVD {
	using namespace physx::debugger;
	using namespace physx::debugger::comm;
}

class PhysX_engine{
public:

	PhysX_engine();
	~PhysX_engine();

	void init();
	void initPvd();
	void createStaticActor(const physx::PxTransform& pose, const physx::PxVec3& boxBounds);
	void createDynamicActor(const physx::PxTransform& pose, physx::PxReal ray);
	void createConstraint();
	void run(bool showRes);
	void showResults();

private:

	friend class impl::MySimulationEventCallback;

	void initFoundation();
	void initProfileZoneManager();
	void initCudaContextManager();
	void initPhysics();
	void initCooking();
	void initExtensions();
	void initCpuDispatcher();
	void initScene();

	physx::PxFoundation* foundation_;
	physx::PxPhysics* physics_;
	physx::PxCooking* cooking_;
	physx::PxScene* scene_;
	physx::PxProfileZoneManager* profileZoneManager_;
	physx::PxCpuDispatcher* cpuDispatcher_;
	physx::PxCudaContextManager* cudaContextManager_;
	physx::PxU32 nbThreads_;
	PVD::PvdConnection* pvdConnection_;

	physx::PxRigidStatic* staticActor_;
	physx::PxMaterial* staticActorMaterial_;
	physx::PxShape* staticShape_;

	physx::PxRigidDynamic* dynamicActor_;
	physx::PxMaterial* dynamicActorMaterial_;
	physx::PxShape* dynamicShape_;

	std::unique_ptr<impl::MySimulationEventCallback> simulationEventCallback_;

	static physx::PxDefaultErrorCallback defaultErrorCallback_;
	static physx::PxDefaultAllocator defaultAllocatorCallback_;
	static const physx::PxSimulationFilterShader defaultFilterShader_;
};
