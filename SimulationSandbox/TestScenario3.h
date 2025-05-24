#pragma once
#include "Scenario.h"

class TestScenario3 : public Scenario
{
protected:
	void setupFixedObjects() override;

public:
	TestScenario3(const CComPtr <ID3D11Device>& pDevice, const CComPtr <ID3D11DeviceContext>& pContext) : Scenario(pDevice, pContext) {}
	~TestScenario3() = default;

	void onLoad() override;
	void onUnload() override;
	void onUpdate(float dt = 0.016) override;
	void ImGuiMainMenu() override;
};