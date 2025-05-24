#include "TestScenario2.h"
#include <memory>
#include "Cylinder.h"
#include "Sphere.h"

void TestScenario2::setupFixedObjects()
{
	float radius = randomFloat(0.4f, 0.7f);
	float scale = radius * 2.0f;
	//float x = randomFloat(-axisLength, axisLength);
	float x = 0.0f;
	float y = -globals::AXIS_LENGTH + scale;
	float z = 0.0f;
	//float z = randomFloat(-axisLength, axisLength);

	auto fixedCylinder = std::make_unique<PhysicsObject>(
		std::make_unique<Cylinder>(
			DirectX::XMFLOAT3(x, y, z),
			DirectX::XMFLOAT3(90.0f, 0.0f, 0.0f),
			DirectX::XMFLOAT3(scale, scale, scale)), true);
	fixedCylinder->LoadModel("cylinder.sjg");
	ConstantBuffer cb = fixedCylinder->getConstantBuffer();
	cb.LightColour = { 1-radius,1-radius,radius, 1.0f };
	cb.DarkColour = cb.LightColour;
	fixedCylinder->setConstantBuffer(cb);
	addPhysicsObject(std::move(fixedCylinder));

	auto sphere = std::make_unique<PhysicsObject>(
		std::make_unique<Sphere>(
			DirectX::XMFLOAT3(x + 0.2f, 2.5f, z),
			DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f),
			DirectX::XMFLOAT3(0.2f, 0.2f, 0.2f)), false);
	sphere->LoadModel("sphere.sjg");
	addPhysicsObject(std::move(sphere));
}

void TestScenario2::onLoad()
{
	OutputDebugString(L">>>>>>>>>> TestScenario2::onLoad\n");

	spawnRoom();
	setupFixedObjects();
	initObjects();
}

void TestScenario2::onUnload()
{
	OutputDebugString(L">>>>>>>>>> TestScenario2::onUnload\n");

	unloadScenario();
}

void TestScenario2::onUpdate(float dt)
{
	updateMovement(dt);
}

void TestScenario2::ImGuiMainMenu()
{
	applySharedGUI();
}