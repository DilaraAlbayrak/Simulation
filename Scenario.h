#pragma once
#include "PhysicsObject.h"
#include "ShaderManager.h"

using globals::AXIS_LENGTH;

class Scenario
{
private:
	std::vector<std::unique_ptr<PhysicsObject>> physicsObjects;
	std::vector<CComPtr<ID3D11Buffer>> vertexBuffers;
	std::vector<CComPtr<ID3D11Buffer>> indexBuffers;
	std::vector<CComPtr<ID3D11InputLayout>> inputLayouts;
	std::vector<CComPtr<ID3D11Buffer>> constantBuffers;
	CComPtr <ID3D11Device> device;
	CComPtr <ID3D11DeviceContext> context;
	CComPtr<ID3D11VertexShader> vertexShader;
	CComPtr<ID3D11PixelShader> pixelShader;
	CComPtr<ID3DBlob> vertexShaderBlob;
	CComPtr<ID3DBlob> pixelShaderBlob;
	std::vector<UINT> indexCounts;

	int integrationMethod = 0;

	int numMovingSpheres = 25;
	float minRadius = 0.01f;
	float maxRadius = 0.01f;
	size_t spawnIndex = 0;
	size_t numSpawned = 0;
	std::vector<std::tuple<float, float, float>> spawnData = {};

	HRESULT initRenderingResources(PhysicsObject* obj);

protected:
	Scenario(const CComPtr <ID3D11Device>& pDevice, const CComPtr <ID3D11DeviceContext>& pContext) : device(pDevice), context(pContext) {}
	void initObjects(const std::wstring& shaderFile = L"Simulation.fx");
	void unloadScenario();
	void applySharedGUI();

	const int getIntegrationMethod() const { return integrationMethod; }
	void updateMovement(float dt);

	void spawnRoom();
	void generateSpawnData(float areaHalfSize = globals::AXIS_LENGTH) {
		spawnData = generateUniform2DPositions(numMovingSpheres, areaHalfSize, minRadius, maxRadius);
		spawnIndex = 0;
	}
	std::vector<std::tuple<float, float, float>> generateUniform2DPositions(int n = 25, float areaHalfSize = globals::AXIS_LENGTH, float minRadius = 0.3f, float maxRadius = 0.3f);
	void spawnMovingSphere();

	virtual void setupFixedObjects() = 0; // Pure virtual function to be implemented in derived classes

	// helper method(s)
	static float randomFloat(float min, float max);

public:
	virtual ~Scenario() = default;

	virtual void onLoad() = 0;
	virtual void onUnload() = 0;
	virtual void onUpdate(float dt = 0.016) = 0;
	virtual void ImGuiMainMenu() = 0;

	void renderObjects();
	void onFrameUpdate(float dt = 0.016);

	std::vector<std::unique_ptr<PhysicsObject>>& getPhysicsObjects() { return physicsObjects; }
	void addPhysicsObject(std::unique_ptr<PhysicsObject> obj)
	{
		physicsObjects.push_back(std::move(obj)); 
	}

	void setNumMovingSpheres(int n) { numMovingSpheres = n; }
	void setMinRadius(float r) { minRadius = r; }
	void setMaxRadius(float r) { maxRadius = r; }
};

