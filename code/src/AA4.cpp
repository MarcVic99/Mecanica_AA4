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
		glm::vec3 velInit = glm::vec3(0.f, 0.2f, 0.f);
		
		glm::mat3 rotRoll = glm::mat3(glm::vec3(1,0,0), 
			glm::vec3 (0, glm::cos(glm::radians(angle)), -glm::sin(glm::radians(angle))),
			glm::vec3(0, glm::sin(glm::radians(angle)), glm::cos(glm::radians(angle))));

		glm::mat3 rotPitch = glm::mat3(glm::vec3(glm::cos(glm::radians(angle)), 0, glm::sin(glm::radians(angle))),
			glm::vec3(0, 1, 0),
			glm::vec3(-glm::sin(glm::radians(angle)), 0, glm::cos(glm::radians(angle))));

		glm::mat3 rotYaw = glm::mat3(glm::vec3(glm::cos(glm::radians(angle)), -glm::sin(glm::radians(angle)), 0),
			glm::vec3(glm::sin(glm::radians(angle)), glm::cos(glm::radians(angle)), 0),
			glm::vec3(0, 0, 1));

		glm::mat3 rotation = rotYaw * rotPitch * rotRoll;
		//glm::mat3 rotation = glm::mat3(1.f);

		simulatedObject = new RigidCube(
			mass, 
			glm::vec3(0.f, 5.f, 0.f), 
			velInit, 
			glm::vec3(0.f, 0.2f, 0.f), 
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
		// Make it use quaternions
		glm::quat testQ;
		
		// Access state.quat
		// Convert to 3x3 mat
		//		(0.5 - y^2 - z^2, xy - wz, xz - wy)
		// 2 *	(xy - wz, 0.5 - x^2 - z^2, yz + wx)
		//		(xz + wy, yz - wx, 0.5 - x^2 - y^2)
		glm::mat3 result = 2.f * glm::mat3(0.5f - testQ.y * testQ.y - testQ.z * testQ.z, testQ.x * testQ.y + testQ.w * testQ.z, testQ.x * testQ.z - testQ.w * testQ.y,
										testQ.x * testQ.y - testQ.w * testQ.z, 0.5f - testQ.x * testQ.x - testQ.z * testQ.z, testQ.y * testQ.z + testQ.w * testQ.x,
										testQ.x * testQ.z + testQ.w * testQ.y, testQ.y * testQ.z - testQ.w * testQ.x, 0.5f - testQ.x * testQ.x - testQ.y * testQ.y);

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
		
		glm::mat3 inertiaTensor = glm::mat3(glm::vec3(1.f / 12.f * mass * ((height * height) + (depth * depth)), 0.f, 0.f),
			glm::vec3(0.f, 1.f / 12.f * mass * ((width * width) + (depth * depth)), 0.f),
			glm::vec3(0.f, 0.f, 1.f / 12.f * mass * ((width * width) + (height * height))));

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

	/*glm::quat RigidBody::GetQuat(float angle, glm::vec3 axis)
	{
		return state.rotQuat;
	}*/

#pragma endregion
	RbState SemiImplicitEuler(const RigidBody* rb, float dt)
	{
		// TODO
		RbState current = rb->GetState();

		// P(t + dt) = P(t) + dt * F(t)
		glm::vec3 newP = current.P; // + dt; //*F;

		// L(t + dt) = L(t) + dt * torque(t)
		glm::vec3 newL = current.L; // + dt; //* torque

		// v(t + dt) = P(t + dt) / M
		glm::vec3 newVelocity = newP / rb->GetMass();

		// x(t + dt) = x(t) + dt * v(t + dt)
		glm::vec3 newCoM = current.centerOfMass + dt * newVelocity; // Nice

		// I(t)^-1 = R(t) * Ibody^-1 * R(t)^T
		glm::mat3 newInverseIbody = rb->GetInverseInertiaTensor();

		// w(t) = I(t)^-1 * L(t + dt)
		glm::vec3 vecW = newInverseIbody * newL;
		glm::mat3 w(0.f, -vecW.z, vecW.y,
					vecW.z, 0.f, -vecW.x,
					-vecW.y, vecW.x, 0.f);

		// R(t + dt) = R(t) + dt * (w(t) * R(t))
		glm::mat3 newRotation = current.rotation + dt * (w * current.rotation);

		printf("X: %f\n", newCoM.x);
		printf("Y: %f\n", newCoM.y);
		printf("Z: %f\n", newCoM.z);
		/*printf("X: %f\n", newP.x);
		printf("Y: %f\n", newP.y);
		printf("Z: %f\n", newP.z);*/



		return { newCoM, newRotation, newP, newL };
	}
}