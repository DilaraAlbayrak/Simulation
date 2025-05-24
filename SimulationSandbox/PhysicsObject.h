#pragma once  
#include <memory>  
#include "SJGLoader.h"  
#include "Collider.h" 
#include "globals.h"

using globals::gravity;

enum class IntegrationMethod  
{  
SEMI_IMPLICIT_EULER,  
RK4,
VERLET
};  

enum class Material
{
	MAT1,
	MAT2,
	MAT3,
	MAT4,
	MAT5
};

static float elasticityLookup[5][5] = {
	{ 0.70f, 0.50f, 0.45f, 0.60f, 0.30f },
	{ 0.60f, 0.50f, 0.35f, 0.55f, 0.40f },
	{ 0.45f, 0.35f, 0.75f, 0.30f, 0.45f },
	{ 0.70f, 0.55f, 0.30f, 0.65f, 0.50f },
	{ 0.30f, 0.40f, 0.45f, 0.50f, 0.35f }
};

struct ConstantBuffer  
{  
   DirectX::XMMATRIX World;  
   DirectX::XMFLOAT4 LightColour = { 0.3f,0.5f,0.7f, 1.0f };
   DirectX::XMFLOAT4 DarkColour = { 0.3f,0.4f,0.5f, 1.0f }; 
   DirectX::XMFLOAT2 CheckerboardSize = { 1.0f, 1.0f }; // Adjust tile size  
   DirectX::XMFLOAT2 Padding = { 0.0f, 0.0f };  
};  

class PhysicsObject  
{  
private:  
	std::unique_ptr<Collider> _collider = nullptr;  

	std::vector<Vertex> _vertices;  
	std::vector<int> _indices;  
	ConstantBuffer _constantBuffer;  

	bool isFixed = false;  
	DirectX::XMFLOAT3 velocity = { 0.0f, 0.0f, 0.0f };  
	DirectX::XMFLOAT3 angularVelocity = { 0.0f, 0.0f, 0.0f };  
	DirectX::XMFLOAT3 acceleration = { 0.0f, 0.0f, 0.0f };  
	DirectX::XMFLOAT3 previousPosition = { 0.0f, 0.0f, 0.0f };
	//DirectX::XMFLOAT3 gravitationalForce; // Gravity vector  
	float mass = 1.0f;  
	float inverseMass = 1.0f;   
	Material material = Material::MAT1;

	IntegrationMethod integrationMethod = IntegrationMethod::SEMI_IMPLICIT_EULER;
	void moveSemiImplicitEuler(float dt);  
	void moveRK4(float dt);  
	void moveVerlet(float dt);

public:  
	PhysicsObject(std::unique_ptr<Collider> col, bool fixed = false, float objMass = 1.0f, Material mat = Material::MAT1)
		: _collider(std::move(col)), isFixed(fixed)  
	{  
		mass = objMass;  
		inverseMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
		material = mat;
		previousPosition = _collider->getPosition();
		integrationMethod = IntegrationMethod::VERLET;
		_constantBuffer.World = _collider->updateWorldMatrix();  
	}  

	void Update(float dt);  

	bool LoadModel(const std::string& filename)  
	{  
		return SJGLoader::Load(filename, _vertices, _indices);  
	}  

	void savePreviousPosition()
	{
		previousPosition = _collider->getPosition();
	}

	bool checkCollision(const PhysicsObject& other, DirectX::XMFLOAT3& outNormal, float& penetrationDepth) const
	{  
		if (!_collider || !other._collider) return false;

		return _collider->isColliding(*other._collider, outNormal, penetrationDepth);
	}  
	void resolveCollision(PhysicsObject& other, const DirectX::XMFLOAT3& collisionNormal, float penetrationDepth);

	const Collider& getCollider() const { return *_collider; }  
	Collider& getCollider() { return *_collider; }
	const std::vector<Vertex>& getVertices() const { return _vertices; }  
	const std::vector<int>& getIndices() const { return _indices; }  
	const DirectX::XMMATRIX& getTransformMatrix() const { return _constantBuffer.World; }  
	const ConstantBuffer getConstantBuffer() const { return _constantBuffer; }  
	const float getMass() const { return mass; }  
	const DirectX::XMFLOAT3 getPosition() const { return _collider->getPosition(); }  
	const DirectX::XMFLOAT3& getVelocity() const { return velocity; }  

	const bool getFixed() const { return isFixed; }  
	void setFixed(bool fixed) { isFixed = fixed; }  

	void setVelocity(const DirectX::XMFLOAT3& newVel) { velocity = newVel; }  
	void setAngularVelocity(const DirectX::XMFLOAT3& newAngVel) { angularVelocity = newAngVel; }  
	void setConstantBuffer(const ConstantBuffer& cb) { _constantBuffer = cb; }  
	void setIntegrationMethod(int method) { integrationMethod = static_cast<IntegrationMethod>(method); }  
	void setMass(float newMass) { mass = newMass; inverseMass = (mass != 0.0f) ? 1.0f / mass : 0.0f; }  
};  
