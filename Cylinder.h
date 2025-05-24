#pragma once
#include "Collider.h"

class Cylinder : public Collider
{
private:
	float _radius = 1.0f;
	float _height = 2.0f;

	void updateDimensionTransformation(); // updates radius and height based on axis direction
public:
	Cylinder(
		DirectX::XMFLOAT3 position = { 0.0f, 0.0f, 0.0f },
		DirectX::XMFLOAT3 rotation = { 0.0f, 0.0f, 0.0f },
		DirectX::XMFLOAT3 scale = { 1.0f, 1.0f, 1.0f })
		: Collider(position, rotation, scale) {
		updateDimensionTransformation();
	}
	~Cylinder() = default;

	DirectX::XMFLOAT3 getAxis() const;

	bool isColliding(const Collider& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const override;

	float getRadius() const { return _radius; }
	float getHeight() const { return _height; }
};

