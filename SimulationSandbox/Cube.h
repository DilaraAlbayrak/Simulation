#pragma once
#include "Collider.h"

class Cube : public Collider
{
public:
	Cube(DirectX::XMFLOAT3 position = { 0.0f, 0.0f, 0.0f },
		DirectX::XMFLOAT3 rotation = { 0.0f, 0.0f, 0.0f },
		DirectX::XMFLOAT3 scale = { 1.0f, 1.0f, 1.0f })
		: Collider(position, rotation, scale) {
	}
	~Cube() = default;

	bool isColliding(const Collider& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const override;
};

