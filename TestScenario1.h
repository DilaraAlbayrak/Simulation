#pragma once
#include "Scenario.h"

class TestScenario1 : public Scenario
{
protected:
	void setupFixedObjects() override;

public:
	TestScenario1(const CComPtr <ID3D11Device>& pDevice, const CComPtr <ID3D11DeviceContext>& pContext) : Scenario(pDevice, pContext) {}
	~TestScenario1() = default;

	void onLoad() override;
	void onUnload() override;
	void onUpdate(float dt = 0.016) override;
	void ImGuiMainMenu() override;
};

