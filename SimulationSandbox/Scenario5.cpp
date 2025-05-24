#include "Scenario5.h"
#include "Cube.h"
#include <memory>

void Scenario5::setupFixedObjects()
{
	auto createFixedCube = [&](const DirectX::XMFLOAT3& position, const DirectX::XMFLOAT3& scale)
	{
		auto cube = std::make_unique<PhysicsObject>(
			std::make_unique<Cube>(
				position,
				DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f),
				scale),
			true, // isFixed
			50.0f, // mass
			Material::MAT2 // material
		);

		cube->LoadModel("cube.sjg");

		ConstantBuffer cb = cube->getConstantBuffer();
		cb.LightColour = { scale.x * 0.3f, scale.y * 0.3f, scale.z * 0.4f, 1.0f };
		cb.DarkColour = cb.LightColour;
		cube->setConstantBuffer(cb);

		addPhysicsObject(std::move(cube));
	};

	auto randomScale = [](float min = 1.2f, float max = 2.0f)
	{
		return DirectX::XMFLOAT3(
			Scenario::randomFloat(min, max),
			Scenario::randomFloat(min, max),
			Scenario::randomFloat(min, max)
		);
	};

	DirectX::XMFLOAT3 scale;

	scale = randomScale();
	createFixedCube(
		{ 0.5f, -globals::AXIS_LENGTH + scale.y * 0.5f + 0.01f, 0.5f },
		scale
	);

	scale = randomScale();
	createFixedCube(
		{ -globals::AXIS_LENGTH + scale.x * 0.5f + 0.01f, 0.8f, 1.0f },
		scale
	);

	scale = randomScale();
	createFixedCube(
		{ globals::AXIS_LENGTH - scale.x * 0.5f - 0.01f, 0.8f, -1.2f },
		scale
	);

	scale = randomScale();
	createFixedCube(
		{ -0.8f, 0.5f, -globals::AXIS_LENGTH + scale.z * 0.5f + 0.01f },
		scale
	);
}

void Scenario5::onLoad()
{
	OutputDebugString(L">>>>>>>>>> Scenario5::onLoad\n");

	spawnRoom();

	setupFixedObjects();

	setNumMovingSpheres(100);
	setMinRadius(0.1f);
	setMaxRadius(0.1f);
	generateSpawnData(globals::AXIS_LENGTH);

	initObjects();
}

void Scenario5::onUnload()
{
	OutputDebugString(L">>>>>>>>>> Scenario5::onUnload\n");

	unloadScenario();
}

void Scenario5::onUpdate(float dt)
{
	updateMovement(dt);
}

void Scenario5::ImGuiMainMenu()
{
	applySharedGUI();
}