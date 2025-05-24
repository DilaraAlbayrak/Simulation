#include "Scenario2.h"
#include "Cylinder.h"
#include "Sphere.h"
#include "Cube.h"
#include <memory>

void Scenario2::setupFixedObjects()
{
	auto createFixedCylinder = [&](const DirectX::XMFLOAT3& position,
		const DirectX::XMFLOAT3& rotation,
		float radius)
	{
		float scale = radius * 2.0f;
		DirectX::XMFLOAT3 scaleVec = { scale, scale, scale };

		auto cylinder = std::make_unique<PhysicsObject>(
			std::make_unique<Cylinder>(
				position,
				rotation,
				scaleVec),
			true, // isFixed
			50.0f, // mass
			Material::MAT2 // material
		);

		cylinder->LoadModel("cylinder.sjg");

		ConstantBuffer cb = cylinder->getConstantBuffer();
		cb.LightColour = { radius, radius, 1-radius, 1.0f };
		cb.DarkColour = cb.LightColour;
		cylinder->setConstantBuffer(cb);

		addPhysicsObject(std::move(cylinder));
	};

	float radius;
	float scale;

	radius = Scenario::randomFloat(0.3f, 0.6f);
	scale = radius * 2.0f;
	createFixedCylinder(
		{ 0.5f, -globals::AXIS_LENGTH + scale + 0.01f, 0.5f },
		{ 0.0f, 0.0f, 0.0f },
		radius
	);

	radius = Scenario::randomFloat(0.3f, 0.6f);
	scale = radius * 2.0f;
	createFixedCylinder(
		{ -globals::AXIS_LENGTH + scale + 0.01f, 0.8f, 1.0f },
		{ 0.0f, 0.0f, 90.0f },
		radius
	);

	radius = Scenario::randomFloat(0.3f, 0.6f);
	scale = radius * 2.0f;
	createFixedCylinder(
		{ globals::AXIS_LENGTH - scale - 0.01f, 0.8f, -1.2f },
		{ 0.0f, 0.0f, 90.0f },
		radius
	);

	radius = Scenario::randomFloat(0.3f, 0.6f);
	scale = radius * 2.0f;
	createFixedCylinder(
		{ -0.8f, 0.5f, -globals::AXIS_LENGTH + scale + 0.01f },
		{ 90.0f, 0.0f, 0.0f },
		radius
	);
}

void Scenario2::onLoad()
{
	OutputDebugString(L">>>>>>>>>> Scenario2::onLoad\n");

	spawnRoom();

	setupFixedObjects();

	setNumMovingSpheres(100);
	setMinRadius(0.1f);
	setMaxRadius(0.1f);
	generateSpawnData(globals::AXIS_LENGTH);

	initObjects();
}

void Scenario2::onUnload()
{
	OutputDebugString(L">>>>>>>>>> Scenario2::onUnload\n");

	unloadScenario();
}

void Scenario2::onUpdate(float dt)
{
	updateMovement(dt);
}

void Scenario2::ImGuiMainMenu()
{
	applySharedGUI();
}