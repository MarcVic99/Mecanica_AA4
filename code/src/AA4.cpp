#include "AA4.h"
#include <glm/gtc/matrix_transform.hpp>

extern bool renderCube;
extern bool renderParticles;
namespace Cube 
{
	extern void updateCube(const glm::mat4& transform);
}

namespace AA4 
{
#pragma region Simulation

	AA4Simulator::AA4Simulator() 
	{
		float mass = 5.f;
		float angle = 30.f;
		glm::vec3 velInit = glm::vec3(0.f, 0.f, 0.f);
		
		glm::mat3 rotRoll = glm::mat3(glm::vec3(1,0,0), 
			glm::vec3 (0, glm::cos(glm::degrees(angle)), -glm::sin(glm::degrees(angle))),
			glm::vec3(0, glm::sin(glm::degrees(angle)), glm::cos(glm::degrees(angle))));

		glm::mat3 rotPitch = glm::mat3(glm::vec3(glm::cos(glm::degrees(angle)), 0, glm::sin(glm::degrees(angle))),
			glm::vec3(0, 1, 0),
			glm::vec3(-glm::sin(glm::degrees(angle)), 0, glm::cos(glm::degrees(angle))));

		glm::mat3 rotYaw = glm::mat3(glm::vec3(glm::cos(glm::degrees(angle)), -glm::sin(glm::degrees(angle)), 0),
			glm::vec3(-glm::sin(glm::degrees(angle)), glm::cos(glm::degrees(angle)), 0),
			glm::vec3(0, 0, 1));

		glm::mat3 rotation = rotYaw * rotPitch * rotRoll;

		simulatedObject = new RigidCube(
			mass, 
			glm::vec3(0.f, 5.f, 0.f), 
			velInit, 
			glm::vec3(0.f, 1.f, 0.f), 
			rotation
		);

		renderCube = true;
		renderParticles = false;
	}

	void AA4Simulator::Update(float dt) 
	{
		RbState previousState = simulatedObject->GetState();
		RbState newState = SemiImplicitEuler(simulatedObject, dt);

		// Manage collisions with obstacles

		simulatedObject->SetState(newState);
	}

	void AA4Simulator::RenderUpdate() 
	{
		simulatedObject->Render();
	}

	void AA4Simulator::RenderGui() {}

	AA4Simulator::~AA4Simulator() 
	{
		renderCube = false;
		delete simulatedObject;
	}

#pragma endregion

#pragma region RigidBodies

	RbState RigidBody::GetState() const 
	{
		return state;
	}
	void RigidBody::SetState(RbState state) 
	{
		this->state = state;
	}

	glm::vec3 RigidBody::GetLinearVelocity() const 
	{
		// TODO
		return glm::vec3();
	}

	glm::mat3 RigidBody::GetInverseInertiaTensor() const 
	{
		glm::mat3 rotMat = GetRotationMatrix();

		return rotMat * InverseIbody * glm::transpose(rotMat);
	}

	glm::vec3 RigidBody::GetAngularVelocity() const 
	{
		return glm::vec3();
	}

	glm::mat3 RigidBody::GetRotationMatrix() const 
	{
		// TODO
		return state.rotation;
	}

	float RigidBody::GetMass() const
	{
		return rbMass;
	}
	float RigidBody::GetMassInverse() const 
	{
		// TODO
		return 0.f;
	}

	glm::mat4 RigidBody::GetTransformMatrix() const 
	{
		// TODO
		return glm::translate(glm::mat4(), state.centerOfMass) * glm::mat4(GetRotationMatrix());
	}

	void RigidCube::Render() const 
	{
		Cube::updateCube(GetTransformMatrix());	
	}

	glm::mat3 RigidCube::ComputeIbody(float mass, float depth, float width, float height) {
		
		glm::mat3 inertiaTensor = glm::mat3(glm::vec3(1 / 12 * mass * ((height * height) + (depth * depth)), 0, 0),
			glm::vec3(0, 1 / 12 * mass * ((width * width) + (depth * depth)), 0),
			glm::vec3(0, 0, 1 / 12 * mass * ((width * width) + (height * height))));

		return inertiaTensor;
	}

	void RigidWall::Render() const 
	{
		// DO NOTHING
	}

	void RigidSphere::Render() const 
	{
		// TODO
	}

#pragma endregion
	RbState SemiImplicitEuler(const RigidBody* rb, float dt)
	{
		// TODO
		RbState current = rb->GetState();

		glm::vec3 newP = current.P + dt; //*F;
		glm::vec3 newL = current.L + dt; //* torque

		glm::vec3 newVelocity = newP / rb->GetMass();

		glm::vec3 newLinearMomentum = current.L;
		glm::vec3 newCoM = current.centerOfMass + dt * newVelocity;// newLinearMomentum;

		glm::mat3 newInverseIbody = rb->GetInverseInertiaTensor();

		glm::vec3 vecW = newInverseIbody * newL;
		glm::mat3 w(0.f, -vecW.z, vecW.y,
					vecW.z, 0.f, -vecW.x,
					-vecW.y, vecW.x, 0.f);

		glm::mat3 newRotation = current.rotation + dt * (w * current.rotation);

		return { newCoM, newRotation, newP, newL };
	}
}