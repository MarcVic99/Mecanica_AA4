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
		glm::vec3 velInit = glm::vec3(5.f, 0.f, 0.f);
		glm::vec3 cubeCoM = glm::vec3(0.f, 5.f, 0.f);
		
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
		
		obstacles.push_back(new RigidWall(glm::vec3(-5.f, 0.f, 0.f), glm::vec3(1.f, 0.f, 0.f))); // Left wall
		obstacles.push_back(new RigidWall(glm::vec3(5.f, 0.f, 0.f), glm::vec3(-1.f, 0.f, 0.f))); // Right wall
		obstacles.push_back(new RigidWall(glm::vec3(0.f, 0.f, 5.f), glm::vec3(0.f, 0.f, -1.f))); // Front wall
		obstacles.push_back(new RigidWall(glm::vec3(0.f, 0.f, -5.f), glm::vec3(0.f, 0.f, 1.f))); // Back wall
		obstacles.push_back(new RigidWall(glm::vec3(0.f, 10.f, 0.f), glm::vec3(0.f, -1.f, 0.f))); // Roof wall
		obstacles.push_back(new RigidWall(glm::vec3(0.f, 0.f, 0.f), glm::vec3(0.f, 1.f, 0.f))); // Floor wall

		std::vector<glm::vec3> cubeVertixs;
		cubeVertixs.push_back(glm::vec3(0.5f, 5.5f, 0.5f));
		cubeVertixs.push_back(glm::vec3(0.5f, 4.5f, 0.5f));
		cubeVertixs.push_back(glm::vec3(0.5f, 5.5f, -0.5f));
		cubeVertixs.push_back(glm::vec3(0.5f, 4.5f, -0.5f));
		cubeVertixs.push_back(glm::vec3(-0.5f, 5.5f, 0.5f));
		cubeVertixs.push_back(glm::vec3(-0.5f, 4.5f, 0.5f));
		cubeVertixs.push_back(glm::vec3(-0.5f, 5.5f, -0.5f));
		cubeVertixs.push_back(glm::vec3(-0.5f, 4.5f, -0.5f));

		simulatedObject = new RigidCube(
			mass,
			cubeCoM,
			velInit,
			glm::vec3(0.f, 0.2f, 0.f),
			simulatedObject->GetQuat(angle, glm::vec3(0.f, 1.f, 0.f)),
			cubeVertixs
		);

		renderCube = true;
		renderParticles = false;
	}

	void AA4Simulator::Update(float dt) 
	{
		RbState previousState = simulatedObject->GetState();
		RbState newState = SemiImplicitEuler(obstacles, simulatedObject, dt);

		// Manage collisions with obstacles
		simulatedObject->SetState(newState);

		simulatedObject->collDetected = simulatedObject->DetectCollision(obstacles, simulatedObject);
		if (simulatedObject->collDetected.isTraversing && !simulatedObject->collDetected.isContact)
		{
			// Has traversed
			simulatedObject->collDetected.isTraversing = false;
			simulatedObject->SetState(previousState);
			Update(dt / 2);
		}
		else if (simulatedObject->collDetected.isContact)
		{
			// Has collided
			//simulatedObject->Impulse(simulatedObject, obstacles[0], simulatedObject->collDetected.contactPoint);
		}
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

	// == RIGID BODY ==
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
		return state.P;
	}

	glm::mat3 RigidBody::GetInverseInertiaTensor() const 
	{
		glm::mat3 rotMat = GetRotationMatrix();

		return rotMat * InverseIbody * glm::transpose(rotMat);
	}

	glm::vec3 RigidBody::GetAngularVelocity() const 
	{
		return state.L;
	}

	glm::mat3 RigidBody::GetRotationMatrix() const 
	{
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

	glm::vec3 RigidBody::GetCenterOfMass() const
	{
		return state.centerOfMass;
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

	glm::vec4 RigidBody::GetFace(int id) const
	{
		// TODO
		return faces[id];
	}

	std::vector<glm::vec3> RigidBody::GetVertex() const
	{
		// TODO
		std::vector<glm::vec3> newVecs;
		glm::mat4 transMat = GetTransformMatrix();

		for (int i = 0; i < vertex.size(); i++)
		{
			glm::vec4 homogenicCoords = glm::vec4(vertex.at(i).x, vertex.at(i).y - 5, vertex.at(i).z, 1.f);

			newVecs.push_back(glm::vec3(transMat * homogenicCoords));
		}

		return newVecs;
	}

	CollisionDetected RigidBody::DetectCollision(std::vector<RigidBody*> planeRb, RigidBody* cubeRb) const
	{
		CollisionDetected detectedColl;

		// TODO
		glm::vec3 planePoint;
		glm::vec3 planeNorm;

		glm::vec3 collisionPoint = glm::vec3(1.f);
		std::vector<glm::vec3> currentCubeVerts = cubeRb->GetVertex();

		// a * x + b * y + c * z + d = 0
		// Putting it in the form dot( (a,b,c,d), (x,y,z,1) ) > 0 
		// where positive dot product is in front of the plane and 
		// negative is behind could be useful/faster. Zero if it is exactly on the plane.

		// == THING EXPLAINED IN CLASS ==
		// (n * p + d)
		// Check normal direction to see 
		// if it's really colliding with rbCube
		// == THING EXPLAINED IN CLASS ==

		// == THING EXPLAINED IN CLASS ==
		// supose we have plane: 2x + 3y + 3z - 16 = 0
		// & P1 = (4, 4, 4) & P2 = (-4, -4, 4)
		// we get the normal vector n = (2, 3, 3)
		// 
		// normal to point we get a positive or negative distance
		// we get a point on the plane with its equation
		// we solve it by searching the first variable that's not 0
		// & changing the others to 0, normalize it
		// == THING EXPLAINED IN CLASS ==


		for (int i = 0; i < planeRb.size(); i++) // Each plane in vector
		{

			for (int j = 0; j < planeRb[i]->faces.size(); j++) // Each face in plane
			{
				planeNorm = glm::vec3(planeRb[i]->faces[j]);

				if (planeRb[i]->faces[j].x != 0)
				{
					planePoint = glm::vec3(-planeRb[i]->faces[j].w / planeRb[i]->faces[j].x, 0.f, 0.f);
				}
				else if (planeRb[i]->faces[j].y != 0)
				{
					planePoint = glm::vec3(0.f, -planeRb[i]->faces[j].w / planeRb[i]->faces[j].y, 0.f);
				}
				else
				{
					planePoint = glm::vec3(0.f, 0.f, -planeRb[i]->faces[j].w / planeRb[i]->faces[j].z);
				}

				for (int k = 0; k < currentCubeVerts.size(); k++) // Each vertex in cube
				{
					glm::vec3 vecToCompare = currentCubeVerts[k] - planePoint;

					if (glm::dot(planeNorm, vecToCompare) > 0)
					{
						// In front of plane
						// we do nothing
					}
					else if (glm::dot(planeNorm, vecToCompare) < 0)
					{
						// Behind of plane (TRAVERSAL COLLISION)
						collisionPoint = currentCubeVerts[k];

						detectedColl.isContact = false;
						detectedColl.isTraversing = true;
						detectedColl.contactPoint = currentCubeVerts[k];
						detectedColl.planeNormal = planeNorm;
						return detectedColl;
					}
					else
					{
						// In plane (CONTACT COLLISION)
						if (collisionPoint == glm::vec3(1.f))
						{
							collisionPoint = currentCubeVerts[k];

							detectedColl.isContact = true;
							detectedColl.contactPoint = currentCubeVerts[k];
							detectedColl.planeNormal = planeNorm;
						}
					}
				}
			}
		}

		return detectedColl;
	}

	void RigidBody::Impulse(RigidBody* rb1, RigidBody* rb2, glm::vec3 contactPoint) const
	{
		glm::vec3 impulseJ;
		glm::vec3 torqueImpulse;

		// p·a(t0) = va(t0) + wa(t0) X (pa(t0) - xa(t0))
		// pa --> contact point
		glm::vec3 pointVelocity = glm::cross((rb1->GetLinearVelocity() + rb1->GetAngularVelocity()), (contactPoint - rb1->GetCenterOfMass()));

		// vrel = n(t0) * (p·a(t0) - p·b(t0))
		// p·a --> point velocity

		// =======

		// timpulse = (p - x(t)) X J
		torqueImpulse = glm::cross((rb1->vertex[0] - rb2->vertex[0]), impulseJ);

	}

	// == CUBE ==
	void RigidCube::Render() const 
	{
		Cube::updateCube(GetTransformMatrix());	
	}

	glm::mat3 RigidCube::ComputeIbody(float mass, float depth, float width, float height) {
		
		glm::mat3 inertiaTensor = glm::mat3(
			glm::vec3(1.f / 12.f * mass * ((height * height) + (depth * depth)), 0.f, 0.f),
			glm::vec3(0.f, 1.f / 12.f * mass * ((width * width) + (depth * depth)), 0.f),
			glm::vec3(0.f, 0.f, 1.f / 12.f * mass * ((width * width) + (height * height))));

		return inertiaTensor;
	}


	// == WALL / PLANE ==
	RigidWall::RigidWall(glm::vec3 planePoint, glm::vec3 planeN)
	{
		planeCenterOfMass = planePoint;
		planeNormal = planeN;

		planeD = CalculatePlaneD(planeNormal, planeCenterOfMass);

		faces.push_back(glm::vec4(planeNormal, planeD));
	}

	void RigidWall::Render() const 
	{
		// DO NOTHING
	}

	float RigidWall::CalculatePlaneD(glm::vec3 normalVector, glm::vec3 planePoint)
	{
		//Components of plane's normal vector
		float A, B, C, D;

		//Components of plane's point 
		float x, y, z;

		A = normalVector.x;
		B = normalVector.y;
		C = normalVector.z;

		x = planePoint.x;
		y = planePoint.y;
		z = planePoint.z;

		D = -(A * x) - (B * y) - (C * z);

		return D;
	}


	// == SPHERE ==
	void RigidSphere::Render() const 
	{
		// TODO
	}

#pragma endregion
	RbState SemiImplicitEuler(std::vector<RigidBody*> rbWall, RigidBody* rbCube, float dt)
	{
		// TODO
		RbState current = rbCube->GetState();

		// P(t + dt) = P(t) + dt * F(t)
		glm::vec3 newP = current.P; // + dt * F;

		// L(t + dt) = L(t) + dt * torque(t)
		glm::vec3 newL = current.L; // + dt * torque;

		// v(t + dt) = P(t + dt) / M
		glm::vec3 newVelocity = newP / rbCube->GetMass();

		// x(t + dt) = x(t) + dt * v(t + dt)
		glm::vec3 newCoM = current.centerOfMass + dt * newVelocity;

		// I(t)^-1 = R(t) * Ibody^-1 * R(t)^T
		glm::mat3 newInverseIbody = rbCube->GetInverseInertiaTensor();

		// w(t) = I(t)^-1 * L(t + dt)
		glm::vec3 vecW = newInverseIbody * newL;
		glm::mat3 w(0.f, -vecW.z, vecW.y,
					vecW.z, 0.f, -vecW.x,
					-vecW.y, vecW.x, 0.f);

		// R(t + dt) = R(t) + dt * (w(t) * R(t))
		//glm::mat3 newRotation = current.rotation + dt * (w * current.rotation);

		// ¨q(t) = 1/2 * w(t) * q(t)
		glm::quat derivQuat = (1.f / 2.f) * glm::quat(0.f, vecW) * current.rotQuat;
		
		// q(t + dt) = q(t) + dt * ¨q(t)
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