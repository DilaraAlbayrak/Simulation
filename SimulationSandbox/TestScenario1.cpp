#include "TestScenario1.h"
#include <memory>
#include "Sphere.h"

void TestScenario1::setupFixedObjects()
{
	float radius = randomFloat(0.5f, 0.8f);
	float scale = radius * 2.0f;
	//float x = randomFloat(-axisLength, axisLength);
	float x = 0.0f;
	float y = -globals::AXIS_LENGTH + scale;
	float z = 0.0f;
	//float z = randomFloat(-axisLength, axisLength);

	auto fixedSphere = std::make_unique<PhysicsObject>(
		std::make_unique<Sphere>(
			DirectX::XMFLOAT3(x, y, z),
			DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f),
			DirectX::XMFLOAT3(scale, scale, scale)), true);
	fixedSphere->LoadModel("sphere.sjg");
	ConstantBuffer cb = fixedSphere->getConstantBuffer();
	cb.LightColour = { 1 - radius,radius,radius, 1.0f };
	cb.DarkColour = cb.LightColour;
	fixedSphere->setConstantBuffer(cb);
	addPhysicsObject(std::move(fixedSphere));

	auto sphere = std::make_unique<PhysicsObject>(
		std::make_unique<Sphere>(
			DirectX::XMFLOAT3(x+0.2f, 2.5f, z),
			DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f),
			DirectX::XMFLOAT3(0.2f, 0.2f, 0.2f)), false);
	sphere->LoadModel("sphere.sjg");
	addPhysicsObject(std::move(sphere));
}

void TestScenario1::onLoad()
{
	OutputDebugString(L">>>>>>>>>> TestScenario1::onLoad\n");

	spawnRoom();
	setupFixedObjects();
	initObjects();
}

void TestScenario1::onUnload()
{
	OutputDebugString(L">>>>>>>>>> TestScenario1::onUnload\n");

	unloadScenario();
}

void TestScenario1::onUpdate(float dt)
{
	updateMovement(dt);
}

void TestScenario1::ImGuiMainMenu()
{
	applySharedGUI();
}