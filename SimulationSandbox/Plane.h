#pragma once
#include "Collider.h"

class Plane : public Collider
{
private:
	DirectX::XMFLOAT3 _normal;
public:
	Plane(DirectX::XMFLOAT3 position = { 0.0f, 0.0f, 0.0f },
		DirectX::XMFLOAT3 rotation = { 0.0f, 0.0f, 0.0f },
		DirectX::XMFLOAT3 scale = { 1.0f, 1.0f, 1.0f },
		DirectX::XMFLOAT3 normal = { 0.0f, 1.0f, 0.0f })
		: Collider(position, rotation, scale), _normal(normal) {
	}
	~Plane() = default;

	bool isColliding(const Collider& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const override;

	void setNormal(DirectX::XMFLOAT3 normal) { _normal = normal; }
	DirectX::XMFLOAT3 getNormal() const { return _normal; }
};