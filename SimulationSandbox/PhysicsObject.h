#pragma once
#include "SJGLoader.h"

struct ConstantBuffer
{
    DirectX::XMMATRIX World;
    DirectX::XMFLOAT4 LightColour = { 0.8f, 0.8f, 0.8f, 1.0f };  // White
    DirectX::XMFLOAT4 DarkColour = { 0.2f, 0.2f, 0.2f, 1.0f };  // Black
    DirectX::XMFLOAT2 CheckerboardSize = { 1.0f, 1.0f }; // Adjust tile size
    DirectX::XMFLOAT2 Padding = { 0.0f, 0.0f };
};

class PhysicsObject
{
private:
	std::vector<Vertex> _vertices;
	std::vector<int> _indices;
	ConstantBuffer _constantBuffer;

public:
	PhysicsObject() = default;
	virtual ~PhysicsObject() = default;

	bool LoadModel(const std::string& filename)
	{
		return SJGLoader::Load(filename, _vertices, _indices);
	}

	const std::vector<Vertex>& getVertices() const { return _vertices; }
	const std::vector<int>& getIndices() const { return _indices; }
};

