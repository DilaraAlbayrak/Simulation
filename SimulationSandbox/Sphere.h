#pragma once
#include "Collider.h"

class Sphere : public Collider
{
private:
	float _radius = 1.0f;
	static constexpr float EPSILON = 1e-4f;
public:
	Sphere(
		DirectX::XMFLOAT3 position = { 0.0f, 0.0f, 0.0f },
		DirectX::XMFLOAT3 rotation = { 0.0f, 0.0f, 0.0f },
		DirectX::XMFLOAT3 scale = { 1.0f, 1.0f, 1.0f })
		: Collider(position, rotation, scale) { _radius = scale.x; }
	~Sphere() = default;

	bool isColliding(const Collider& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const override;
	bool isCollidingWithSphere(const Sphere& sphere, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const;
	bool isCollidingWithPlane(const Plane& plane, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const;
	bool isCollidingWithCylinder(const Cylinder& cylinder, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const;
	bool isCollidingWithCube(const Cube& cube, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const;

	float getRadius() const { return _radius; }
};

