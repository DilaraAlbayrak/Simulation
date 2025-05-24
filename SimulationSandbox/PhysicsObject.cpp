#include "PhysicsObject.h"
#include "Sphere.h"

using namespace DirectX;

void PhysicsObject::moveSemiImplicitEuler(float dt)
{
    velocity.x += acceleration.x * dt;
    velocity.y += acceleration.y * dt;
    velocity.z += acceleration.z * dt;

    DirectX::XMFLOAT3 deltaPos = { velocity.x * dt, velocity.y * dt, velocity.z * dt };
    _collider->incrementPosition(deltaPos);
}

void PhysicsObject::moveRK4(float dt)
{
    DirectX::XMFLOAT3 k1_v = velocity;
    DirectX::XMFLOAT3 k1_a = acceleration;

    DirectX::XMFLOAT3 mid_v = {
        velocity.x + 0.5f * dt * k1_a.x,
        velocity.y + 0.5f * dt * k1_a.y,
        velocity.z + 0.5f * dt * k1_a.z
    };
    DirectX::XMFLOAT3 k2_v = mid_v;
    DirectX::XMFLOAT3 k2_a = k1_a;

    mid_v = {
        velocity.x + 0.5f * dt * k2_a.x,
        velocity.y + 0.5f * dt * k2_a.y,
        velocity.z + 0.5f * dt * k2_a.z
    };
    DirectX::XMFLOAT3 k3_v = mid_v;
    DirectX::XMFLOAT3 k3_a = k2_a;

    DirectX::XMFLOAT3 end_v = {
        velocity.x + dt * k3_a.x,
        velocity.y + dt * k3_a.y,
        velocity.z + dt * k3_a.z
    };
    DirectX::XMFLOAT3 k4_v = end_v;
    DirectX::XMFLOAT3 k4_a = k3_a;

    velocity.x += (dt / 6.0f) * (k1_a.x + 2.0f * k2_a.x + 2.0f * k3_a.x + k4_a.x);
    velocity.y += (dt / 6.0f) * (k1_a.y + 2.0f * k2_a.y + 2.0f * k3_a.y + k4_a.y);
    velocity.z += (dt / 6.0f) * (k1_a.z + 2.0f * k2_a.z + 2.0f * k3_a.z + k4_a.z);

    DirectX::XMFLOAT3 deltaPos = { (dt / 6.0f) * (k1_v.x + 2.0f * k2_v.x + 2.0f * k3_v.x + k4_v.x),
                                   (dt / 6.0f) * (k1_v.y + 2.0f * k2_v.y + 2.0f * k3_v.y + k4_v.y),
                                   (dt / 6.0f) * (k1_v.z + 2.0f * k2_v.z + 2.0f * k3_v.z + k4_v.z) };

    _collider->incrementPosition(deltaPos);
}

void PhysicsObject::moveVerlet(float dt)
{
    DirectX::XMFLOAT3 currentPos = _collider->getPosition();

    DirectX::XMFLOAT3 deltaPos = {
    velocity.x * dt + 0.5f * acceleration.x * dt * dt,
    velocity.y * dt + 0.5f * acceleration.y * dt * dt,
    velocity.z * dt + 0.5f * acceleration.z * dt * dt
    };

    velocity.x += acceleration.x * dt;
    velocity.y += acceleration.y * dt;
    velocity.z += acceleration.z * dt;

    _collider->incrementPosition(deltaPos);
}

void PhysicsObject::Update(float deltaTime)
{
	if (!_collider) return;
    if (isFixed) return;

   // previousPosition = _collider->getPosition();

    acceleration = {globals::gravity.x * inverseMass,
                    globals::gravity.y * inverseMass,
                    globals::gravity.z * inverseMass };

    if (integrationMethod == IntegrationMethod::SEMI_IMPLICIT_EULER)
    {
        moveSemiImplicitEuler(deltaTime);
    }
    else if (integrationMethod == IntegrationMethod::RK4)
    {
        moveRK4(deltaTime);
    }
	else if (integrationMethod == IntegrationMethod::VERLET)
	{
		moveVerlet(deltaTime);
	}

    _constantBuffer.World = _collider->updateWorldMatrix();
}

void PhysicsObject::resolveCollision(PhysicsObject& other, const DirectX::XMFLOAT3& collisionNormal, float penetrationDepth)
{
    if (isFixed && other.isFixed) return;

    // Relative velocity
    DirectX::XMFLOAT3 relativeVelocity = {
        velocity.x - other.velocity.x,
        velocity.y - other.velocity.y,
        velocity.z - other.velocity.z
    };

    // Velocity along collision normal
    float velocityAlongNormal =
        relativeVelocity.x * collisionNormal.x +
        relativeVelocity.y * collisionNormal.y +
        relativeVelocity.z * collisionNormal.z;

    if (velocityAlongNormal > -1e-4f) return; // Skip separating

    float invMassA = isFixed ? 0.0f : inverseMass;
    float invMassB = other.isFixed ? 0.0f : other.inverseMass;
    float invMassSum = invMassA + invMassB;
    if (invMassSum <= 1e-5f) return;

    // look up elasticity
	int matA = static_cast<int>(material);
	int matB = static_cast<int>(other.material);
	float e = elasticityLookup[matA][matB];

    float impulseMagnitude = -(1.0f + e) * velocityAlongNormal / invMassSum;

    DirectX::XMFLOAT3 impulse = {
        impulseMagnitude * collisionNormal.x,
        impulseMagnitude * collisionNormal.y,
        impulseMagnitude * collisionNormal.z
    };

    if (!isFixed) {
        velocity.x += impulse.x * invMassA;
        velocity.x *= 0.99f;
        velocity.y += impulse.y * invMassA;
		velocity.y *= 0.99f;
        velocity.z += impulse.z * invMassA;
		velocity.z *= 0.99f;
    }

    if (!other.isFixed) {
        other.velocity.x -= impulse.x * invMassB;
		other.velocity.x *= 0.99f;
        other.velocity.y -= impulse.y * invMassB;
		other.velocity.y *= 0.99f;
        other.velocity.z -= impulse.z * invMassB;
		other.velocity.z *= 0.99f;
    }

    // Gravity-based tangent sliding correction
    auto applySlidingCorrection = [](DirectX::XMFLOAT3& vel, const DirectX::XMFLOAT3& normal) {

        XMVECTOR v = XMLoadFloat3(&vel);
        XMVECTOR n = XMLoadFloat3(&normal);

        XMVECTOR vNormal = XMVectorScale(n, XMVectorGetX(XMVector3Dot(v, n)));
        XMVECTOR vTangent = XMVectorSubtract(v, vNormal);
        XMVECTOR g = XMLoadFloat3(&globals::gravity);
        XMVECTOR gNormal = XMVectorScale(n, XMVectorGetX(XMVector3Dot(g, n)));
        XMVECTOR gTangent = XMVectorSubtract(g, gNormal);

        float vTangentSq = XMVectorGetX(XMVector3LengthSq(vTangent));
        float gTangentSq = XMVectorGetX(XMVector3LengthSq(gTangent));

        // Apply sliding impulse if sphere is stuck
        if (vTangentSq < 0.001f && gTangentSq > 0.001f) {
            float correctionScale = 0.2f;

            // if it's slow
            if (vTangentSq < 1e-3f)
                correctionScale = 0.3f;

            // if it's too slow
            if (vTangentSq < 1e-5f)
                correctionScale = 0.4f;

            v = XMVectorAdd(v, XMVectorScale(gTangent, correctionScale));
        }

        v = XMVectorAdd(v, XMVectorScale(gTangent, 0.1f));

        XMStoreFloat3(&vel, v);
        };

    if (!isFixed) {
        applySlidingCorrection(velocity, collisionNormal);
    }
    if (!other.isFixed) {
        applySlidingCorrection(other.velocity, collisionNormal);
    }

    // Rollback to previous position (required for stability)
    _collider->setPosition(previousPosition);
	other._collider->setPosition(other.previousPosition);
}
