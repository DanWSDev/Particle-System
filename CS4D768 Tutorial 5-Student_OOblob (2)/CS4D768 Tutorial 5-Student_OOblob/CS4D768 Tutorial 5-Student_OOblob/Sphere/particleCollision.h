#pragma once
#include "pcontacts.h"
#include <vector>

class particleCollision : public ParticleContactGenerator
{
	unsigned numberofBlobs = 0;
	
public:
	particleCollision(unsigned int numberofBlobs) : numberofBlobs(numberofBlobs)
	{
		pParticle = nullptr;
	};
	Particle* pParticle;

	virtual unsigned addContact(ParticleContact* contact, unsigned limit);
	bool checkCollision(Particle* firstParticle, Particle* secondParticle, Vector2* contactNormal, Vector2* delta);
};

