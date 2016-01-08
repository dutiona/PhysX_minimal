#include "PhysX_engine.h"

#include <memory>
#include <thread>
#include <array>
#include <vector>
#include <iostream>


int main(int /*argc*/, char* /*argv[]*/){

	//PhysX engine
	auto engine = std::make_unique<PhysX_engine>();
	engine->init();
	engine->initPvd();

	//Box
	engine->createStaticActor(physx::PxTransform{ physx::PxVec3{ 0, 0, 0 }, physx::PxQuat{ 3.14159f / 16.f, {1.0f, 0.0f, 0.0f} } }, physx::PxVec3{ 10, 1, 10 });
	//Sphere
	engine->createDynamicActor(physx::PxTransform{ physx::PxVec3{ 5, 10, 5 } }, physx::PxReal{ 2 });

	//Run physX engine
	engine->run(true);
	engine->showResults();

	std::cout << "Appuyez sur une enter pour continuer..." << std::endl;
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

	return EXIT_SUCCESS;
}