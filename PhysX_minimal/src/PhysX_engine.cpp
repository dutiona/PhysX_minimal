#include "PhysX_engine.h"

#include <stdexcept>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

using namespace physx;

PxDefaultErrorCallback PhysX_engine::defaultErrorCallback_;
PxDefaultAllocator PhysX_engine::defaultAllocatorCallback_;
const PxSimulationFilterShader PhysX_engine::defaultFilterShader_ = &PxDefaultSimulationFilterShader;

namespace impl{

	class MySimulationEventCallback : public PxSimulationEventCallback{
	public:

		MySimulationEventCallback(PhysX_engine* engine);
		virtual ~MySimulationEventCallback();

		virtual void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) override;
		virtual void onWake(PxActor** actors, PxU32 count) override;
		virtual void onSleep(PxActor** actors, PxU32 count) override;
		virtual void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override;
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) override;

	private:
		PhysX_engine* engine_;
	};
}

//Public interface

PhysX_engine::PhysX_engine() :
nbThreads_(8)
{}

void PhysX_engine::init(){
	initFoundation();
	initProfileZoneManager();
	initCudaContextManager();
	initPhysics();
	initCooking();
	initExtensions();
	initCpuDispatcher();
	initScene();
}

void PhysX_engine::initPvd(){
#ifdef RENDERER_PVD
	// check if PvdConnection manager is available on this platform
	if (physics_->getPvdConnectionManager() == nullptr)
		return;

	// setup connection parameters
	const auto pvd_host_ip = "127.0.0.1";	// IP of the PC which is running PVD
	int	port = 5425;						// TCP port to connect to, where PVD is listening
	unsigned int timeout = 100;				// timeout in milliseconds to wait for PVD to respond,
	// consoles and remote PCs need a higher timeout.
	PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();
	// and now try to connect
	pvdConnection_ = PxVisualDebuggerExt::createConnection(physics_->getPvdConnectionManager(), pvd_host_ip, port, timeout, connectionFlags);
#endif
}

void PhysX_engine::createStaticActor(const physx::PxTransform& pose, const PxVec3& boxBounds){
	staticActorMaterial_ = physics_->createMaterial(0.5f, 0.5f, 0.1f);    //static friction, dynamic friction, restitution
	if (!staticActorMaterial_){
		throw std::runtime_error{ "createMaterial failed!" };
	}

	staticActor_ = physics_->createRigidStatic(pose);
	staticShape_ = staticActor_->createShape(PxBoxGeometry{ boxBounds }, *staticActorMaterial_);
	scene_->addActor(*staticActor_);
}

void PhysX_engine::createDynamicActor(const physx::PxTransform& pose, PxReal ray){
	dynamicActorMaterial_ = physics_->createMaterial(0.5f, 0.5f, 0.1f);    //static friction, dynamic friction, restitution
	if (!dynamicActorMaterial_){
		throw std::runtime_error{ "createMaterial failed!" };
	}

	dynamicActor_ = physics_->createRigidDynamic(pose);
	dynamicShape_ = dynamicActor_->createShape(PxSphereGeometry{ ray }, *dynamicActorMaterial_);
	PxRigidBodyExt::updateMassAndInertia(*dynamicActor_, 1.0);
	scene_->addActor(*dynamicActor_);
}

void PhysX_engine::run(bool showres){
	auto accumulator = 0.0f;
	auto stepSize = 1.0f / 60.0f;

	for (int i = 0; i < 1000; i++){
		scene_->simulate(stepSize);
		scene_->fetchResults(true);
		if (showres) showResults();
	}
}

void PhysX_engine::showResults(){
	const auto lastPos = dynamicActor_->getGlobalPose();
	std::cout << "Position : (x=" << lastPos.p.x << ", y=" << lastPos.p.y << ", z=" << lastPos.p.z << ")" << std::endl;
}


//Private impl

void PhysX_engine::initFoundation(){
	foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, defaultAllocatorCallback_, defaultErrorCallback_);
	if (!foundation_){
		throw std::runtime_error{ "PxCreateFoundation failed !" };
	}
}

void PhysX_engine::initProfileZoneManager(){
	profileZoneManager_ = &PxProfileZoneManager::createProfileZoneManager(foundation_);
	if (!profileZoneManager_){
		throw std::runtime_error{ "PxProfileZoneManager::createProfileZoneManager failed!" };
	}
}

void PhysX_engine::initCudaContextManager(){
#if PX_SUPPORT_GPU_PHYSX
	auto cudaContextManagerDesc = PxCudaContextManagerDesc{};
	cudaContextManager_ = PxCreateCudaContextManager(*foundation_, cudaContextManagerDesc, profileZoneManager_);
	if (cudaContextManager_){
		if (!cudaContextManager_->contextIsValid()){
			cudaContextManager_->release();
			cudaContextManager_ = nullptr;
		}
	}
#else
	cudaContextManager_ = nullptr;
#endif // PX_SUPPORT_GPU_PHYSX
	}

void PhysX_engine::initPhysics(){
	bool recordMemoryAllocations = true;
	physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, PxTolerancesScale(), recordMemoryAllocations, profileZoneManager_);
	if (!physics_){
		throw std::runtime_error{ "PxCreatePhysics failed!" };
	}
}

void PhysX_engine::initCooking(){
	auto toleranceScale = PxTolerancesScale{};
	toleranceScale.length = 10e-3f;
	toleranceScale.mass = 1000;
	toleranceScale.speed = 1e-1f;
	auto cookingParams = PxCookingParams{ toleranceScale };
	cookingParams.areaTestEpsilon = 10e-5f;
	cookingParams.buildTriangleAdjacencies = false;
	cookingParams.meshCookingHint = PxMeshCookingHint::eSIM_PERFORMANCE;
	cookingParams.meshPreprocessParams = PxMeshPreprocessingFlags{ PxMeshPreprocessingFlag::eREMOVE_DUPLICATED_TRIANGLES | PxMeshPreprocessingFlag::eWELD_VERTICES };
	cookingParams.meshSizePerformanceTradeOff = 1.0;
	cookingParams.meshWeldTolerance = 10e-5f;
	cookingParams.suppressTriangleMeshRemapTable = false;
	cookingParams.targetPlatform = PxPlatform::ePC;
	cooking_ = PxCreateCooking(PX_PHYSICS_VERSION, *foundation_, cookingParams);
	if (!cooking_){
		throw std::runtime_error{ "PxCreateCooking failed!" };
	}
}

void PhysX_engine::initExtensions(){
	if (!PxInitExtensions(*physics_)){
		throw std::runtime_error{ "PxInitExtensions failed!" };
	}
}

void PhysX_engine::initCpuDispatcher(){
	cpuDispatcher_ = PxDefaultCpuDispatcherCreate(nbThreads_);
	if (!cpuDispatcher_){
		throw std::runtime_error{ "PxDefaultCpuDispatcherCreate failed!" };
	}
}

void PhysX_engine::initScene(){
	auto sceneDesc = PxSceneDesc{ physics_->getTolerancesScale() };

	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.bounceThresholdVelocity = 0.01f * physics_->getTolerancesScale().speed;
	sceneDesc.broadPhaseCallback = nullptr;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eSAP;
	sceneDesc.ccdContactModifyCallback = nullptr;
	sceneDesc.ccdMaxPasses = 1;
	sceneDesc.contactModifyCallback = nullptr;
	sceneDesc.contactReportStreamBufferSize = 8192;
	sceneDesc.cpuDispatcher = cpuDispatcher_;
	sceneDesc.dynamicStructure = PxPruningStructure::eDYNAMIC_AABB_TREE;
	sceneDesc.dynamicTreeRebuildRateHint = 100;
	sceneDesc.filterCallback = nullptr;
	sceneDesc.filterShader = defaultFilterShader_;
	sceneDesc.filterShaderData = nullptr;
	sceneDesc.filterShaderDataSize = 0;
	sceneDesc.flags = PxSceneFlags{
		//| PxSceneFlag::eENABLE_ACTIVETRANSFORMS
		PxSceneFlag::eENABLE_CCD
		//| PxSceneFlag::eDISABLE_CCD_RESWEEP
		//| PxSceneFlag::eADAPTIVE_FORCE
		| PxSceneFlag::eENABLE_KINEMATIC_STATIC_PAIRS
		| PxSceneFlag::eENABLE_KINEMATIC_PAIRS
		| PxSceneFlag::eENABLE_PCM
		//| PxSceneFlag::eDISABLE_CONTACT_REPORT_BUFFER_RESIZE
		//| PxSceneFlag::eDISABLE_CONTACT_CACHE
		//| PxSceneFlag::eREQUIRE_RW_LOCK
		| PxSceneFlag::eENABLE_STABILIZATION
		//| PxSceneFlag::eENABLE_AVERAGE_POINT
	};
	sceneDesc.frictionOffsetThreshold = 0.04f * physics_->getTolerancesScale().length;
	sceneDesc.frictionType = PxFrictionType::ePATCH;
	sceneDesc.gpuDispatcher = cudaContextManager_->getGpuDispatcher();
	auto limits = PxSceneLimits{};
	limits.maxNbActors = 0;				// no limit
	limits.maxNbAggregates = 0;			// no limit
	limits.maxNbBodies = 0;				// no limit
	limits.maxNbConstraints = 0;		// no limit
	limits.maxNbDynamicShapes = 0;		// no limit
	limits.maxNbObjectsPerRegion = 0;	// no limit
	limits.maxNbRegions = 0;			// no limit
	limits.maxNbStaticShapes = 0;		// no limit
	sceneDesc.limits = limits;
	sceneDesc.maxNbContactDataBlocks = 65536;
	sceneDesc.nbContactDataBlocks = 0;
	simulationEventCallback_ = std::make_unique<impl::MySimulationEventCallback>(this);
	sceneDesc.simulationEventCallback = &*simulationEventCallback_;
	sceneDesc.simulationOrder = PxSimulationOrder::eCOLLIDE_SOLVE;
	sceneDesc.solverBatchSize = 128;
	sceneDesc.staticStructure = PxPruningStructure::eDYNAMIC_AABB_TREE;
	sceneDesc.userData = nullptr;

	scene_ = physics_->createScene(sceneDesc);
	if (!scene_){
		throw std::runtime_error{ "createScene failed!" };
	}
}

PhysX_engine::~PhysX_engine(){
	// L'ordre est important mais est ici sans doute faux...
	if (staticActorMaterial_ != nullptr) staticActorMaterial_->release();
	if (dynamicActorMaterial_ != nullptr) dynamicActorMaterial_->release();
	if (staticActor_ != nullptr) staticActor_->release();
	if (dynamicActor_ != nullptr) dynamicActor_->release();
	if (pvdConnection_ != nullptr) pvdConnection_->release();
	if (scene_ != nullptr) scene_->release();
	if (cooking_ != nullptr) cooking_->release();
	if (physics_ != nullptr) physics_->release();
	if (foundation_ != nullptr) foundation_->release();
	if (cudaContextManager_ != nullptr) cudaContextManager_->release();
	if (profileZoneManager_ != nullptr) profileZoneManager_->release();
}

namespace impl{
	MySimulationEventCallback::MySimulationEventCallback(PhysX_engine* engine) :
		engine_(engine)
	{}

	MySimulationEventCallback::~MySimulationEventCallback()
	{}


	void MySimulationEventCallback::onConstraintBreak(PxConstraintInfo* constraints, PxU32 count){
		//TODO
	}

	void MySimulationEventCallback::onWake(PxActor** actors, PxU32 count){
		//TODO
	}

	void MySimulationEventCallback::onSleep(PxActor** actors, PxU32 count){
		//TODO
	}

	void MySimulationEventCallback::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs){
		//TODO
	}

	void MySimulationEventCallback::onTrigger(PxTriggerPair* pairs, PxU32 count){
		//TODO
	}
}