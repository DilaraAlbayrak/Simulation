#include "Cube.h"
#include "Sphere.h"

bool Cube::isColliding(const Collider& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const
{
    if (const Sphere* sphere = dynamic_cast<const Sphere*>(&other)) {
        bool result = sphere->isCollidingWithCube(*this, outNormal, penetrationDepth);
        
        outNormal.x = -outNormal.x;
        outNormal.y = -outNormal.y;
        outNormal.z = -outNormal.z;
        return result;
    }
    return false;
}
