#pragma once
#include "Scenario.h"

class TestScenario2 : public Scenario
{
protected:
	void setupFixedObjects() override;

public:
	TestScenario2(const CComPtr <ID3D11Device>& pDevice, const CComPtr <ID3D11DeviceContext>& pContext) : Scenario(pDevice, pContext) {}
	~TestScenario2() = default;

	void onLoad() override;
	void onUnload() override;
	void onUpdate(float dt = 0.016) override;
	void ImGuiMainMenu() override;
};