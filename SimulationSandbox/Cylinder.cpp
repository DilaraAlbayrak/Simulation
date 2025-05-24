#include "Cylinder.h"
#include "Sphere.h"
#include <algorithm>

using namespace DirectX;

//DirectX::XMFLOAT3 Cylinder::getAxis() const
//{
//    // Local up vector (cylinder axis in model space)
//    XMVECTOR localAxis = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
//
//    // Get world matrix (transpose is already applied in Collider)
//    XMMATRIX world = updateWorldMatrix();
//
//    // Extract rotation + scale part only (drop translation)
//    XMMATRIX rotScaleOnly = world;
//    rotScaleOnly.r[3] = XMVectorSet(0, 0, 0, 1); // zero out translation
//
//    // Transform local axis to world space
//    XMVECTOR worldAxis = XMVector3TransformNormal(localAxis, rotScaleOnly);
//    worldAxis = XMVector3Normalize(worldAxis);
//
//    DirectX::XMFLOAT3 out;
//    XMStoreFloat3(&out, worldAxis);
//
//    return out;
//}

DirectX::XMFLOAT3 Cylinder::getAxis() const
{
    XMVECTOR localAxis = XMVectorSet(0, 1, 0, 0);  // Model space Y-axis
    XMMATRIX world = updateWorldMatrix();
    world.r[3] = XMVectorSet(0, 0, 0, 1); // remove translation

    XMVECTOR worldAxis = XMVector3TransformNormal(localAxis, world);
    worldAxis = XMVector3Normalize(worldAxis);

    XMFLOAT3 result;
    XMStoreFloat3(&result, worldAxis);
    return result;
}

void Cylinder::updateDimensionTransformation()
{
    _radius = getScale().x;
    _height = _radius;
    /*if (getAxis().y > 0.1f)
		_radius *= 0.5f;*/		
}

bool Cylinder::isColliding(const Collider& other, XMFLOAT3& outNormal, float& penetrationDepth) const
{
    if (const Sphere* sphere = dynamic_cast<const Sphere*>(&other)) {
        bool result = sphere->isCollidingWithCylinder(*this, outNormal, penetrationDepth);
        // Invert normal to match this cylinder as 'self'
        outNormal.x = -outNormal.x;
        outNormal.y = -outNormal.y;
        outNormal.z = -outNormal.z;
        return result;
    }
    return false;
}

