#include "Plane.h"
#include "Sphere.h"

bool Plane::isColliding(const Collider& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const
{
    if (const Sphere* sphere = dynamic_cast<const Sphere*>(&other)) {
        bool result = sphere->isCollidingWithPlane(*this, outNormal, penetrationDepth);
        // Invert normal to point from this plane outward
        outNormal.x = -outNormal.x;
        outNormal.y = -outNormal.y;
        outNormal.z = -outNormal.z;
        return result;
    }
    return false;
}
