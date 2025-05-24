#pragma once
#include "Scenario.h"

class Scenario2 : public Scenario
{
protected:
	void setupFixedObjects() override;

public:
	Scenario2(const CComPtr <ID3D11Device>& pDevice, const CComPtr <ID3D11DeviceContext>& pContext) : Scenario(pDevice, pContext) {}
	~Scenario2() = default;

	void onLoad() override;
	void onUnload() override;
	void onUpdate(float dt = 0.016) override;
	void ImGuiMainMenu() override;
};

