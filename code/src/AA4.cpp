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
		
		/*glm::mat3 rotRoll = glm::mat3(glm::vec3(1,0,0), 
			glm::vec3 (0, glm::cos(glm::radians(angle)), -glm::sin(glm::radians(angle))),
			glm::vec3(0, glm::sin(glm::radians(angle)), glm::cos(glm::radians(angle))));

		glm::mat3 rotPitch = glm::mat3(glm::vec3(glm::cos(glm::radians(angle)), 0, glm::sin(glm::radians(angle))),
			glm::vec3(0, 1, 0),
			glm::vec3(-glm::sin(glm::radians(angle)), 0, glm::cos(glm::radians(angle))));

		glm::mat3 rotYaw = glm::mat3(glm::vec3(glm::cos(glm::radians(angle)), -glm::sin(glm::radians(angle)), 0),
			glm::vec3(glm::sin(glm::radians(angle)), glm::cos(glm::radians(angle)), 0),
			glm::vec3(0, 0, 1));

		glm::mat3 rotation = rotYaw * rotPitch * rotRoll;*/
		
		simulatedObject = new RigidCube(
			mass, 
			glm::vec3(0.f, 5.f, 0.f), 
			velInit, 
			glm::vec3(0.f, 0.2f, 0.f), 
			simulatedObject->GetQuat(angle, glm::vec3(0.f, 1.f, 0.f))
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
		// Access state.quat
		// Convert to 3x3 mat
		//		(0.5 - y^2 - z^2, xy - wz, xz - wy)
		// 2 *	(xy - wz, 0.5 - x^2 - z^2, yz + wx)
		//		(xz + wy, yz - wx, 0.5 - x^2 - y^2)
		glm::mat3 result = 2.f * glm::mat3
			(0.5f - state.rotQuat.y * state.rotQuat.y - state.rotQuat.z * state.rotQuat.z, state.rotQuat.x * state.rotQuat.y + state.rotQuat.w * state.rotQuat.z, state.rotQuat.x * state.rotQuat.z - state.rotQuat.w * state.rotQuat.y,
			state.rotQuat.x * state.rotQuat.y - state.rotQuat.w * state.rotQuat.z, 0.5f - state.rotQuat.x * state.rotQuat.x - state.rotQuat.z * state.rotQuat.z, state.rotQuat.y * state.rotQuat.z + state.rotQuat.w * state.rotQuat.x,
			state.rotQuat.x * state.rotQuat.z + state.rotQuat.w * state.rotQuat.y, state.rotQuat.y * state.rotQuat.z - state.rotQuat.w * state.rotQuat.x, 0.5f - state.rotQuat.x * state.rotQuat.x - state.rotQuat.y * state.rotQuat.y);

		return result;
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

	glm::quat RigidBody::GetQuat(float angle, glm::vec3 axis) const
	{
		// TODO
		glm::quat resQuat;
		resQuat.x = axis.x * sin(angle / 2.f);
		resQuat.y = axis.y * sin(angle / 2.f);
		resQuat.z = axis.z * sin(angle / 2.f);
		resQuat.w = cos(angle / 2.f);

		return glm::normalize(resQuat);
	}

	glm::mat3 RigidBody::GetFace(int id) const
	{
		// TODO
		return faces[id];
	}

	glm::vec3 RigidBody::GetVertex(int id) const
	{
		// TODO
		return vertex[id];
	}

	bool RigidBody::DetectCollision(RigidBody* A, RigidBody* B) const
	{
		// TODO
		glm::mat3 aFace;

		// a * x + b * y + c * z + d = 0
		// Putting it in the form dot( (a,b,c,d), (x,y,z,1) ) > 0 
		// where positive dot product is in front of the plane and 
		// negative is behind could be useful/faster. Zero if it is exactly on the plane.
		//if (glm::dot(aFace, glm::vec4(A->vertex[0], 1.f)) > 0) {}

		for (int i = 0; i < A->faces.size(); i++)
		{
			 aFace = A->GetFace(i);

			 for (int i = 0; i < A->vertex.size(); i++)
			 {
				 A->GetVertex(i);
			 }

			 for (int i = 0; i < B->vertex.size(); i++)
			 {
				 B->GetVertex(i);
			 }
		}


		return true;
	}

#pragma endregion
	RbState SemiImplicitEuler(const RigidBody* rb, float dt)
	{
		// TODO
		RbState current = rb->GetState();

		/*if (DetectCollision(RigidBody cube, RigidBody floor))
		{
			return current;
		}*/

		// P(t + dt) = P(t) + dt * F(t)
		glm::vec3 newP = current.P; // + dt * F;

		// L(t + dt) = L(t) + dt * torque(t)
		glm::vec3 newL = current.L; // + dt * torque;

		// v(t + dt) = P(t + dt) / M
		glm::vec3 newVelocity = newP / rb->GetMass();

		// x(t + dt) = x(t) + dt * v(t + dt)
		glm::vec3 newCoM = current.centerOfMass + dt * newVelocity;

		// I(t)^-1 = R(t) * Ibody^-1 * R(t)^T
		glm::mat3 newInverseIbody = rb->GetInverseInertiaTensor();

		// w(t) = I(t)^-1 * L(t + dt)
		glm::vec3 vecW = newInverseIbody * newL;
		glm::mat3 w(0.f, -vecW.z, vecW.y,
					vecW.z, 0.f, -vecW.x,
					-vecW.y, vecW.x, 0.f);

		// R(t + dt) = R(t) + dt * (w(t) * R(t))
		//glm::mat3 newRotation = current.rotation + dt * (w * current.rotation);

		// �q(t) = 1/2 * w(t) * q(t)
		glm::quat derivQuat = (1.f / 2.f) * glm::quat(0.f, vecW) * current.rotQuat;
		
		// q(t + dt) = q(t) + dt * �q(t)
		glm::quat newQuat = glm::normalize(current.rotQuat + dt * derivQuat);

		/*printf("X: %f\n", newCoM.x);
		printf("Y: %f\n", newCoM.y);
		printf("Z: %f\n", newCoM.z);*/
		/*printf("X: %f\n", newP.x);
		printf("Y: %f\n", newP.y);
		printf("Z: %f\n", newP.z);*/

		return { newCoM, newP, newL, newQuat };
	}
}