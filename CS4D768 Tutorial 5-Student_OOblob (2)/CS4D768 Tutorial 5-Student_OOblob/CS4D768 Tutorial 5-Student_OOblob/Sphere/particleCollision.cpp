#include "particleCollision.h"
#include <iostream>
using namespace std;

unsigned particleCollision::addContact(ParticleContact* contact, unsigned limit)
{
	unsigned used = 0;

	Particle* firstParticle;
	Particle* secondParticle;
	Vector2 delta, contactNormal;
	float pentration = 0;
	float totalRadius = 0;
	float distance = 0;

	for (unsigned i = 0; i < numberofBlobs; i++)
	{
		for (unsigned j = (i + 1); j < numberofBlobs; j++)
		{
			firstParticle = pParticle + i;
			secondParticle = pParticle + j;

			delta = firstParticle->getPosition() - secondParticle->getPosition();
			distance = delta.magnitude();
			totalRadius = firstParticle->getRadius() + secondParticle->getRadius();
			firstParticle[-j].colour = false;		// makes all the particles colour bool false
			secondParticle[0].colour = false;
			if (distance <= totalRadius)
			{
				contact->contactNormal = delta.unit();
				contact->restitution = 1.0;
				contact->particle[0] = firstParticle;
				firstParticle[0].colour = true;		// returns true if the particles have collided
				contact->particle[1] = secondParticle;
				secondParticle[0].colour = true;	// returns true if the particles have collided
				contact->penetration = pentration;
				used++;
				contact++;
			}
		}
	}

	return used;
}
