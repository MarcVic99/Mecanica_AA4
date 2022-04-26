#include "Spring.h"

glm::vec3 CalculateVectorBetweenTwoPointsSpring(glm::vec3 firstP, glm::vec3 secondP) 
{
	return firstP - secondP;
}

Spring::Spring()
{
}

Spring::~Spring()
{

}

glm::vec3 Spring::GetStrenghtBetweenTwoPositions(glm::vec3 pos1, glm::vec3 pos2, glm::vec3 vel1, glm::vec3 vel2)
{
	glm::vec3 resultingStr = { 0.f, 0.f, 0.f };

	glm::vec3 vectorPos2Pos1 = CalculateVectorBetweenTwoPointsSpring(pos1, pos2);
	glm::vec3 normalizedVectorPos2Pos1 = glm::normalize(vectorPos2Pos1);

	float dampingK = glm::dot(damping * (vel1 - vel2), normalizedVectorPos2Pos1);
	
	resultingStr = -(constantK * (glm::length(vectorPos2Pos1) - equilibriumDistance) + dampingK) * normalizedVectorPos2Pos1;

	return resultingStr;
}