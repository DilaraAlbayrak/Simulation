#include <algorithm>
#include <cmath>

#include "Sphere.h"
#include "Plane.h"
#include "Cylinder.h"
#include "Cube.h"

using namespace DirectX;

bool Sphere::isColliding(const Collider& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const
{
    // Check if it's a Sphere
    if (const Sphere* sphere = dynamic_cast<const Sphere*>(&other))
        return isCollidingWithSphere(*sphere, outNormal, penetrationDepth);

    // Check if it's a Plane
    if (const Plane* plane = dynamic_cast<const Plane*>(&other))
        return isCollidingWithPlane(*plane, outNormal, penetrationDepth);

    // Check if it's a Cylinder
    if (const Cylinder* cylinder = dynamic_cast<const Cylinder*>(&other))
        return isCollidingWithCylinder(*cylinder, outNormal, penetrationDepth);

    // Check if it's a Cube
    if (const Cube* cube = dynamic_cast<const Cube*>(&other))
        return isCollidingWithCube(*cube, outNormal, penetrationDepth);

    return false;
}

bool Sphere::isCollidingWithSphere(const Sphere& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const
{
    float dx = getPosition().x - other.getPosition().x;
    float dy = getPosition().y - other.getPosition().y;
    float dz = getPosition().z - other.getPosition().z;

    float distanceSquared = dx * dx + dy * dy + dz * dz;
    float sumRadii = getRadius() + other.getRadius();
    float sumRadiiSquared = sumRadii * sumRadii;

    if (distanceSquared <= sumRadiiSquared + EPSILON)
    {
        float distance = sqrtf(distanceSquared);

        if (distance > 1e-5f)
        {
            outNormal.x = dx / distance;
            outNormal.y = dy / distance;
            outNormal.z = dz / distance;
        }
        else
        {
            outNormal = { 1.0f, 0.0f, 0.0f }; // Arbitrary direction
        }

        penetrationDepth = sumRadii - distance;
        return true;
    }

    outNormal = { 0.0f, 0.0f, 0.0f };
    penetrationDepth = 0.0f;
    return false;
}

bool Sphere::isCollidingWithPlane(const Plane& plane, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const
{
    XMFLOAT3 planeNormal = plane.getNormal();
    XMFLOAT3 planePoint = plane.getPosition();
    XMFLOAT3 spherePos = getPosition();

    XMVECTOR vPlaneNormal = XMLoadFloat3(&planeNormal);
    XMVECTOR vPlanePoint = XMLoadFloat3(&planePoint);
    XMVECTOR vSpherePos = XMLoadFloat3(&spherePos);

    vPlaneNormal = XMVector3Normalize(vPlaneNormal);

    XMVECTOR planeToSphere = XMVectorSubtract(vSpherePos, vPlanePoint);

    float distance = XMVectorGetX(XMVector3Dot(planeToSphere, vPlaneNormal));

    float absDistance = fabsf(distance);

    if (absDistance <= _radius + EPSILON)
    {
        penetrationDepth = _radius - absDistance;

        if (distance < 0)
            vPlaneNormal = XMVectorNegate(vPlaneNormal);

        XMStoreFloat3(&outNormal, vPlaneNormal);
        return true;
    }

	// No collision
	outNormal = XMFLOAT3(0.0f, 0.0f, 0.0f);
    penetrationDepth = 0.0f;
    return false;
}

bool Sphere::isCollidingWithCylinder(const Cylinder& cylinder, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const
{
    using namespace DirectX;

    XMFLOAT3 sphereCenter = getPosition();
    XMFLOAT3 cylinderCenter = cylinder.getPosition();
    XMFLOAT3 axisDir = cylinder.getAxis();
	//XMFLOAT3 axisDir = { 0.0f, 1.0f, 0.0f }; // Assuming Y-axis for cylinder direction
    float cylinderHalfHeight = cylinder.getHeight() *0.5f;
    float cylinderRadius = cylinder.getRadius();

    XMVECTOR vAxisDir = XMVector3Normalize(XMLoadFloat3(&axisDir));
    XMVECTOR vCylinderCenter = XMLoadFloat3(&cylinderCenter);
    XMVECTOR vSphereCenter = XMLoadFloat3(&sphereCenter);

    // Start and end of cylinder axis
    XMVECTOR start = XMVectorSubtract(vCylinderCenter, XMVectorScale(vAxisDir, cylinderHalfHeight));
    XMVECTOR end = XMVectorAdd(vCylinderCenter, XMVectorScale(vAxisDir, cylinderHalfHeight));

    XMVECTOR axis = XMVectorSubtract(end, start);
    XMVECTOR sphereToStart = XMVectorSubtract(vSphereCenter, start);

    float axisLenSq = XMVectorGetX(XMVector3LengthSq(axis));
    if (axisLenSq < 1e-6f) {
        outNormal = { 0, 0, 0 };
        penetrationDepth = 0.0f;
        return false;
    }

    float t = XMVectorGetX(XMVector3Dot(sphereToStart, axis)) / axisLenSq;
    t = std::clamp(t, 0.0f, 1.0f);

    XMVECTOR closest = XMVectorAdd(start, XMVectorScale(axis, t));
    XMVECTOR diff = XMVectorSubtract(vSphereCenter, closest);
    float distSq = XMVectorGetX(XMVector3LengthSq(diff));

    float combinedRadius = getRadius() + cylinderRadius;
    float combinedRadiusSq = combinedRadius * combinedRadius;

    if (distSq <= combinedRadiusSq + EPSILON) {
        float dist = sqrtf(distSq);

        if (dist > 1e-5f) {
            XMStoreFloat3(&outNormal, XMVectorScale(diff, 1.0f / dist));
        }
        else {
            outNormal = { 0.0f, 1.0f, 0.0f };
        }

        penetrationDepth = combinedRadius - dist;
        return true;
    }

    outNormal = { 0, 0, 0 };
    penetrationDepth = 0.0f;
    return false;
}

bool Sphere::isCollidingWithCube(const Cube& cube, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const
{
    DirectX::XMFLOAT3 sphereCenter = getPosition();
    float radius = getRadius();

    DirectX::XMFLOAT3 cubeCenter = cube.getPosition();
    DirectX::XMFLOAT3 cubeScale = cube.getScale();

    // Calculate cube's min and max corners
    float cubeMinX = cubeCenter.x - cubeScale.x * 0.5f;
    float cubeMaxX = cubeCenter.x + cubeScale.x * 0.5f;
    float cubeMinY = cubeCenter.y - cubeScale.y * 0.5f;
    float cubeMaxY = cubeCenter.y + cubeScale.y * 0.5f;
    float cubeMinZ = cubeCenter.z - cubeScale.z * 0.5f;
    float cubeMaxZ = cubeCenter.z + cubeScale.z * 0.5f;

    // Clamp sphere center to the closest point on the cube
    float closestX = std::max(cubeMinX, std::min(sphereCenter.x, cubeMaxX));
    float closestY = std::max(cubeMinY, std::min(sphereCenter.y, cubeMaxY));
    float closestZ = std::max(cubeMinZ, std::min(sphereCenter.z, cubeMaxZ));

    // Compute the vector from closest point to sphere center
    float dx = sphereCenter.x - closestX;
    float dy = sphereCenter.y - closestY;
    float dz = sphereCenter.z - closestZ;
    float distanceSq = dx * dx + dy * dy + dz * dz;

    float radiusSq = radius * radius;

    if (distanceSq <= radiusSq + EPSILON)
    {
        float distance = sqrtf(distanceSq);

        if (distance > 1e-5f)
        {
            outNormal.x = dx / distance;
            outNormal.y = dy / distance;
            outNormal.z = dz / distance;
        }
        else
        {
			outNormal = DirectX::XMFLOAT3(1.0f, 0.0f, 0.0f); // arbitrary value
        }

        penetrationDepth = radius - distance;
        return true;
    }

    // No collision
    penetrationDepth = 0.0f;
    outNormal = DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f); 
    return false;
}
