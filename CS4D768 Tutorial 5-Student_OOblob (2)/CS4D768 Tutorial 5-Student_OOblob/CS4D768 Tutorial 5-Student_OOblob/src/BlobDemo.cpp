/*
 * The Blob demo.
 *
 */
#include <gl/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include "particleCollision.h"
#include <stdio.h>
#include <cassert>
#include <iostream>         // added for print outs

using namespace std;        // added for print outs

#define PLATFORM_COUNT 18           // The amount of platforms within the simulation
#define Pcount 150              // the Amount of particles within the simulation

int weightL = 0;                // The left side of the scale weight 
int weightR = 0;                //  The right side of the scale weight

bool Lines = false;             // change this to true to show lines from each particle to particle
bool Points = false;                // change this to true to show points of the platforms
const Vector2 Vector2::GRAVITY = Vector2(0,-9.81);

/**
 * Platforms are two dimensional: lines on which the 
 * particles can rest. Platforms are also contact generators for the physics.
 */

class Platform : public ParticleContactGenerator
{
public:
    Vector2 start;
    Vector2 end;
    float restitutionPlatform;      // allows for each platform to have its own resitution value 


    int count;      // returns the index of the platform this is used for testing
    /**
     * Holds a pointer to the particles we're checking for collisions with. 
     */
    Particle *particle;

    virtual unsigned addContact(
        ParticleContact *contact, 
        unsigned limit
        );
};

unsigned Platform::addContact(ParticleContact *contact, 
                              unsigned limit)
{
    
	//const static float restitution = 0.8f;
    float restitution = restitutionPlatform;  
	unsigned used = 0;

    for (unsigned i = 0; i < Pcount; i++)
    {
        if (used >= limit) return used;
        // Check for penetration
        Vector2 toParticle = particle[i].getPosition() - start;
        Vector2 lineDirection = end - start;

        float projected = toParticle * lineDirection;
        float platformSqLength = lineDirection.squareMagnitude();
        float squareRadius = particle[i].getRadius() * particle[i].getRadius();;
        //   cout << "Detection Test Platform Start:  " << count << "   Particle: " << i << endl;  // Uncomment this to test detection of the start point
        if (projected <= 0)
        {

            // The blob is nearest to the start point
            if (toParticle.squareMagnitude() < squareRadius)
            {
             //   cout << "Collision Test Platform Start:  " << count << "   Particle: " << i << endl;      // uncomment this to test collision of the start point
                // We have a collision
                if ((count == 8 || count == 9) && weightL < 25)     // This is for the left side of the scale it checks if the platforms have been collide with a particle and if it weight less then 25
                {
                    weightL++;      // adds the weight to the left side of the scale
                }
                else if ((count == 10 || count == 11) && weightR < 25)  // This is for the right side of the scale it checks if the platforms have been collide with a particle and if it weight less then 25
                {
                    weightR++;      // adds the weight to the right side of the scale
                }
                else if (weightL > 25 && weightR > 25) //This checks if both weight values are greater then 25 
                {
                    weightL = 0;        // this will reset the values so that the scale can function again
                    weightR = 0;
                }
                
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = particle + i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }

        }
        else if (projected >= platformSqLength)
        {
            //  cout << "Detection Test Platform End:    " << count << "   Particle: " << i << endl; // uncomment this to test detection of the end point
            // The blob is nearest to the end point
            toParticle = particle[i].getPosition() - end;
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // We have a collision
                if ((count == 8 || count == 9) && weightL < 25)     // This is for the left side of the scale it checks if the platforms have been collide with a particle and if it weight less then 25
                {
                    weightL++;      // adds the weight to the left side of the scale
                }
                else if ((count == 10 || count == 11) && weightR < 25)  // This is for the right side of the scale it checks if the platforms have been collide with a particle and if it weight less then 25
                {
                    weightR++;      // adds the weight to the right side of the scale
                }
                else if (weightL > 25 && weightR > 25) //This checks if both weight values are greater then 25 
                {
                    weightL = 0;        // this will reset the values so that the scale can function again
                    weightR = 0;
                }

                // We have a collision
              //  cout << "Collision Test Platform End:    " << count << "   Particle: " << i << endl; // uncomment this to test collision of the end point
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = particle + i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }
        }
        else
        {
            //  cout << "Detection Test Platform Middle: " << count <<"   Particle: " << i << endl; // uncomment this to test detection of the middle of the platform
            // the blob is nearest to the middle.
            float distanceToPlatform = toParticle.squareMagnitude() - projected * projected / platformSqLength;
            if (distanceToPlatform < squareRadius)
            {
                if ((count == 8 || count == 9) && weightL < 25)     // This is for the left side of the scale it checks if the platforms have been collide with a particle and if it weight less then 25
                {
                    weightL++;      // adds the weight to the left side of the scale
                }
                else if ((count == 10 || count == 11) && weightR < 25)  // This is for the right side of the scale it checks if the platforms have been collide with a particle and if it weight less then 25
                {
                    weightR++;      // adds the weight to the right side of the scale
                }
                else if (weightL > 25 && weightR > 25) //This checks if both weight values are greater then 25 
                {
                    weightL = 0;        // this will reset the values so that the scale can function again
                    weightR = 0;
                }
                
                // We have a collision
              //  cout << "Collision Test Platform Middle: " << count <<"   Particle: " << i << endl; // uncomment this to test collision of the middle of the platform
                Vector2 closestPoint = start + lineDirection * (projected / platformSqLength);

                contact->contactNormal = (particle[i].getPosition() - closestPoint).unit();
                contact->restitution = restitution;
                contact->particle[0] = particle + i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - sqrt(distanceToPlatform);
                used++;
                contact++;
            }

            
        }
    }
        return used;
   
}


class BlobDemo : public Application
{
    Particle *blob;

    Platform *platform;

    particleCollision* pCollisionGenarator;

    ParticleWorld world;

public:
    /** Creates a new demo object. */
    BlobDemo();
    virtual ~BlobDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particles. */
    virtual void display();

    virtual void Scale();       // This was added to call the scale and to keep the display function more readable 

    /** Update the particle positions. */
    virtual void update();
	
};

// Method definitions
BlobDemo::BlobDemo():world(400,200)
{
	width = 800; height = 800; 
	nRange = 100.0;

  
  

    // Create the blob storage
    blob = new Particle[Pcount];        // Creates a instance of a particle from the particle array
   
    // Create the platform
    platform = new Platform[PLATFORM_COUNT];     // Creates a instance of a platform from the platform array
	

	platform[0].start = Vector2 ( -nRange + 10, nRange - 10 );
	platform[0].end   = Vector2 (nRange - 10, nRange - 10);

    platform[0].restitutionPlatform = 1.0f;


    platform[1].start = Vector2(-nRange + 10, -nRange + 10);
    platform[1].end = Vector2(nRange - 10, -nRange + 10);               // this creates the outside box 

    platform[1].restitutionPlatform = 1.0f;

    platform[2].start = Vector2(-nRange + 10, nRange - 10);
    platform[2].end = Vector2(-nRange + 10, -nRange + 10);
    platform[2].restitutionPlatform = 1.0f;
  
    platform[3].start = Vector2(nRange - 10, nRange - 10);
    platform[3].end = Vector2(nRange - 10, -nRange + 10);
    platform[3].restitutionPlatform = 1.0f;


    
    
    platform[4].start = Vector2(-50, -65);
    platform[4].end = Vector2(50, -65);             // creates SCALE line
    platform[4].restitutionPlatform = 0.0f;
  


    
    platform[5].start = Vector2(0, -65);
    platform[5].end = Vector2(10, -85);
    platform[5].restitutionPlatform = 0.0f;

    platform[6].start = Vector2(10, -85);
    platform[6].end = Vector2(-10, -85);
    platform[6].restitutionPlatform = 0.0f;         // Triangle Stand

    platform[7].start = Vector2(-10, -85);
    platform[7].end = Vector2(0, -65);
    platform[7].restitutionPlatform = 0.0f;

    platform[8].start = Vector2(-50, -65);
    platform[8].end = Vector2(-75, -40);
    platform[8].restitutionPlatform = 0.0f;
                                                // Left Cup
    platform[9].start = Vector2(-50, -65);
    platform[9].end = Vector2(-25, -40);
    platform[9].restitutionPlatform = 0.0f;


    platform[10].start = Vector2(50, -65);
    platform[10].end = Vector2(75, -40);
    platform[10].restitutionPlatform = 0.0f;
                                                // Right Cup
    platform[11].start = Vector2(50, -65);
    platform[11].end = Vector2(25, -40);
    platform[11].restitutionPlatform = 0.0f;



    platform[12].start = Vector2(-5, 5);
    platform[12].end = Vector2(5, 5);
    platform[12].restitutionPlatform = 1.0f;


    platform[13].start = Vector2(5, 5);
    platform[13].end = Vector2(10, 12.5);
    platform[13].restitutionPlatform = 1.0f;
  
    platform[14].start = Vector2(-5, 5);
    platform[14].end = Vector2(-10, 12.5);
    platform[14].restitutionPlatform = 1.0f;
                                                    //Hexagon
    platform[15].start = Vector2(-10, 12.5);
    platform[15].end = Vector2(-5, 20);
    platform[15].restitutionPlatform = 1.0f;

    platform[16].start = Vector2(10, 12.5);
    platform[16].end = Vector2(5, 20);
    platform[16].restitutionPlatform = 1.0f;

    platform[17].start = Vector2(5, 20);
    platform[17].end = Vector2(-5, 20);
    platform[17].restitutionPlatform = 1.0f;
   

    // Make sure the platform knows which particle it should collide with.
   

   for (unsigned i = 0; i < PLATFORM_COUNT; i++)
    {
            platform[i].count = i;          // Returns the index to the count variable to the platform this is done for testing and to tip the scales 
            platform[i].particle = blob;

        world.getContactGenerators().push_back(platform + i);
    }

   pCollisionGenarator = new particleCollision(Pcount);

   pCollisionGenarator->pParticle = blob;                   // Allows for particle on particle collision to take place

   world.getContactGenerators().push_back(pCollisionGenarator);

 
       for (unsigned j = 0; j < Pcount; j++)
       {

           // Create the blob


           blob[j].setPosition(-87.0 + j * 3, 90.0f);                   //Creates the first 50 particles in a line
           if (j >= 50)                                                     
               blob[j].setPosition(-217.0 + j * 3, 80.0f);              //Creates the second of 50 particles in a line and moves it down
           if (j >= 100)
               blob[j].setPosition(-387.0 + j * 3, 70.0f);               //Finally creates the thrid of 50 particles in a line and moves it down
                                                                        // When setting the particles it is quite a challenge to position a large amount without causing errors. Also the double if statement should be revistied or changed but when changed to else if does not follow the same way as expected.
           blob[j].setRadius(rand() % 3 + 1);               // Randoming picks a value for the radius between 1 and 3
           blob[j].setVelocity(0, 0);
     
           blob[j].setDamping(1.0);                         //sets the damping of the blob 
           blob[j].setAcceleration(Vector2::GRAVITY * 10.0f);           // sets the gravity of the blob

           blob[j].setMass(10.0f);                          // sets the mass of the blob
           blob[j].clearAccumulator();

           world.getParticles().push_back(blob + j);
       }
   
    
}


BlobDemo::~BlobDemo()
{
    delete blob;
}


void BlobDemo::Scale()      // Moves the scale depending on the weight  this is like a seesaw 
{
   // cout << "WieghtL: " << weightL << "WieghtR: " << weightR << endl; //Scale Testing uncomment if the values need to be seen

    if (weightL > weightR)      // if the left side is greater then the right sided
    {

        platform[4].start = Vector2(-50, -65 - weightL);
        platform[4].end = Vector2(50, -65 + weightL);             // SCALE line

         // Left Cup
        platform[8].start = Vector2(-50, -65 - weightL);
        platform[8].end = Vector2(-75, -40 - weightL);

                                                            // updates the position of the platforms to account for the weight increase on the left side of the scale
        platform[9].start = Vector2(-50, -65 - weightL);
        platform[9].end = Vector2(-25, -40 - weightL);


        // Right Cup
        platform[10].start = Vector2(50, -65 + weightL);
        platform[10].end = Vector2(75, -40 + weightL);


        platform[11].start = Vector2(50, -65 + weightL);
        platform[11].end = Vector2(25, -40 + weightL);
    }
    else  if (weightR > weightL)
    {

        platform[4].start = Vector2(-50, -65 + weightR);
        platform[4].end = Vector2(50, -65 - weightR);             // SCALE line

         // Left Cup
        platform[8].start = Vector2(-50, -65 + weightR);
        platform[8].end = Vector2(-75, -40 + weightR);


        platform[9].start = Vector2(-50, -65 + weightR);        // updates the position of the platforms to account for the weight increase on the right side of the scale
        platform[9].end = Vector2(-25, -40 + weightR);


        // Right Cup
        platform[10].start = Vector2(50, -65 - weightR);
        platform[10].end = Vector2(75, -40 - weightR);


        platform[11].start = Vector2(50, -65 - weightR);
        platform[11].end = Vector2(25, -40 - weightR);
    }

    else
    {

        platform[4].start = Vector2(-50, -65);
        platform[4].end = Vector2(50, -65);             // SCALE line

         // Left Cup
        platform[8].start = Vector2(-50, -65);
        platform[8].end = Vector2(-75, -40);


        platform[9].start = Vector2(-50, -65);                  // if the scale is not heaiver on one side then make the scale balanced and reset the position
        platform[9].end = Vector2(-25, -40);


        // Right Cup
        platform[10].start = Vector2(50, -65);
        platform[10].end = Vector2(75, -40);


        platform[11].start = Vector2(50, -65);
        platform[11].end = Vector2(25, -40);
        weightL = 0;                                // makes the values of the weight zero as there is no particles on the scale
        weightR = 0;
    }

}

void BlobDemo::display()
{
  
    Scale();        // calls the scale function

  Application::display();



  if (Lines == true)            
  {

      glBegin(GL_LINES);        // allows for lines to be drawn

      glColor3ub(0, 255, 0);            // sets the colour of the lines to green

      for (unsigned l = 0; l < Pcount; l++)     // the two for loops point to every other particle expect for itself
      {

          for (unsigned k = 1; k < Pcount; k++)
          {
              const Vector2& p1 = blob[k].getPosition();        // gets the position of each particle 
              const Vector2& p2 = blob[l].getPosition();

              glVertex2f(p1.x, p1.y);                   // draws the line between the two particles
              glVertex2f(p2.x, p2.y);

          }

      }
      glEnd(); 
  }
  
  glBegin(GL_LINES);

  glColor3f(0, 1, 1);       // sets the colour to blue

  for (unsigned i = 0; i < PLATFORM_COUNT; i++)         // for every platform within the simulation
  {


      const Vector2& p0 = platform[i].start;
      const Vector2& p1 = platform[i].end;          // the start and end point to the platform 


      
      if (i >= 4)
      {
          glColor3ub(255,0,255);            // purple colour for the scale and the hexagon
      }
      else                      // displays diffent colour platforms 
      {
          glColor3f(0, 1, 1); // the first 4 platforms draw the box
      }
     
      glVertex2f(p0.x, p0.y);
      glVertex2f(p1.x, p1.y);     // Draws the platforms from the start and end point
  }

  if (Points == true)           
  {

      for (unsigned i = 0; i < PLATFORM_COUNT; i++)
      {
          const Vector2& p0 = platform[i].start;
          const Vector2& p1 = platform[i].end;          // the start and end point to the platform 

          glPushMatrix();               // push the gl stack
          glTranslatef(p0.x, p0.y, 0);          // gets the start point of the platform
          glColor3ub(0, 255, 0);        // sets the colour to green 
          glutWireSphere(3, 3, 2);          //displays a triangle on the start ponint.
          glPopMatrix();                // pops the gl stack
          glPushMatrix();               // push the gl stack
          glTranslatef(p1.x, p1.y, 0);          // gets the end point of the platform
          glColor3ub(255, 0, 0);            // sets the colour to red
          glutWireSphere(3, 3, 2);          //displays a triangle on the end ponint.
          glPopMatrix();                // pops the gl stack
      }
  }
  glEnd();

  
   for (unsigned j = 0; j < Pcount; j++)            // For every particle within the array
   {

       glColor3ub(rand()% 255 + 1,255,255);         // sets the colour randomly selects the red value 
       const Vector2& p = blob[j].getPosition();        // gets the position of the particle
       if (blob[j].colour)         // returns the value of the colour bool set by the particle this bool will become true when two particles collide 
       {
           glColor3ub(rand() % 255 + 1, 0, 0); // sets the colour randomly selects the red value 
       }
       glPushMatrix();  // push the gl stack 
       glTranslatef(p.x, p.y, 0);   // sets the position of the particle making the center the zero point
       glutSolidSphere(blob[j].getRadius(), 12, 12);        // draws the particle by using the radius value
       

       if ((p.x < -90.0f) || (p.y < -90.f) || (p.x > 90.0f) || (p.y > 90.0f))           // if any of the particles leave the outside box 
       {
           blob[j].setPosition(0, 0);       // Set the postion of the particles to return // this is testing to see if the particles leave the box
       }


       char name[]{'0' + j , "\0"};         //display particle indexs to be by the particle
       glColor3ub(255, 255, 255);       // sets the colour to white
       glRasterPos2f(0.0f, 10.0f);      // moves the text up the y axis to be above the particle
       for (int l = 0; l < strlen(name); l++)       // character array to display the variable
       glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, name[l]);      // font family and font size for the characters that are being diplayed to the screen
      // std::cout << "Particle" << j << ".x : " << p.x << "  Particle" << j << ".y : " << p.y << std::endl;  // prints out particles location

       glPopMatrix();        // pops the gl stack

   }

   char name[] = "Create By Daniel Wiegold Stannard - 17736382"; // displays my name
   glColor3ub(255, 255, 255); // set the colour to white
   glRasterPos3f(-50.0f, -95.0f, 0.0f); //moves the character array at the bottom of the screen
   for (int p = 0; p < strlen(name); p++)    // character array to display the variable
       glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, name[p]);  // font family and font size for the characters that are being diplayed to the screen


	glutSwapBuffers();
  
}

void BlobDemo::update()
{
    // Recenter the axes
	float duration = timeinterval/1000;
    // Run the simulation
    world.runPhysics(duration);

    Application::update();
}

const char* BlobDemo::getTitle()
{
    return "Blob Demo";
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new BlobDemo();
}