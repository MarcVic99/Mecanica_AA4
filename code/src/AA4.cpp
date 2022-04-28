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
			glm::vec3 (0,glm::cos(glm::degrees(angle)), -glm::sin(glm::degrees(angle))),
			glm::vec3(0,glm::sin(glm::degrees(angle)), glm::cos(glm::degrees(angle))));

		glm::mat3 rotPitch = glm::mat3(glm::vec3(glm::cos(glm::degrees(angle)), 0, glm::sin(glm::degrees(angle))),
			glm::vec3(0, 1, 0),
			glm::vec3(-glm::sin(glm::degrees(angle)), 0, glm::cos(glm::degrees(angle))));

		glm::mat3 rotYaw = glm::mat3(glm::vec3(glm::cos(glm::degrees(angle)), -glm::sin(glm::degrees(angle)), 0),
			glm::vec3(-glm::sin(glm::degrees(angle)), glm::cos(glm::degrees(angle)), 0),
			glm::vec3(0, 0, 1));

		glm::mat4 rotation = rotYaw * rotPitch * rotRoll;

		simulatedObject = new RigidCube(mass, glm::vec3(0.f, 5.f, 0.f), velInit, glm::vec3(0.f, 1.f, 0.f), glm::mat3(rotation));

		simulatedObject->SetState({
						glm::vec3(0.f, 5.f, 0.f),
						mass * velInit,
						rotYaw * rotPitch * rotRoll 
		});

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
		// TODO
		return glm::mat3();
	}

	glm::vec3 RigidBody::GetAngularVelocity() const 
	{
		// TODO
		return glm::vec3();
	}

	glm::mat3 RigidBody::GetRotationMatrix() const 
	{
		// TODO
		return state.angularMomentum;
	}

	float RigidBody::GetMassInverse() const 
	{
		// TODO
		return 0.f;
	}

	glm::mat4 RigidBody::GetTransformMatrix() const 
	{
		// TODO
		return glm::translate(glm::mat4(), state.centerOfMass);
	}

	void RigidCube::Render() const 
	{
		Cube::updateCube(GetTransformMatrix());
		Cube::updateCube(GetRotationMatrix());
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

		glm::vec3 newLinearMomentum = current.linearVelocity;
		glm::vec3 newCoM = current.centerOfMass + dt * newLinearMomentum;

		glm::mat3 newAngularMomentum = current.angularMomentum;

			
		return { newCoM, newLinearMomentum, newAngularMomentum };
	}
}