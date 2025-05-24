#include "Scenario4.h"
#include <memory>

void Scenario4::setupFixedObjects()
{
}

void Scenario4::onLoad()
{
	OutputDebugString(L">>>>>>>>>> Scenario4::onLoad\n");

	spawnRoom();

	initObjects();
}

void Scenario4::onUnload()
{
	OutputDebugString(L">>>>>>>>>> Scenario4::onUnload\n");

	unloadScenario();
}

void Scenario4::onUpdate(float dt)
{
	updateMovement(dt);
}

void Scenario4::ImGuiMainMenu()
{
	applySharedGUI();
}