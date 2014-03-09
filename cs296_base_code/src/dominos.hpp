/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_
#include "dominos.cpp"
#include "callbacks.hpp"
#include "cs296_base.hpp"


  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  
 
  
  class dominos_t : public  cs296::base_sim_t
  {
  public:
   
   
  //b2Body* body;
  b2Body* m_bodyA;
		b2Body* m_bodyB;
		b2Body* m_bodyC;
		b2RevoluteJoint* m_joint;
		b2RevoluteJoint* m_joint3;
		b2PrismaticJoint* m_joint2;
		b2PrismaticJoint* m_joint4;
		b2RevoluteJointDef revoluteJointDef2;
		b2RevoluteJointDef revoluteJointDef;
  
    dominos_t(){
		b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  b2FixtureDef fixtureDef;
  fixtureDef.density = 1;
  
 // b2BodyDef bodyDef1;
  //bodyDef.type = b2_staticBody;
  //b2FixtureDef fixtureDef1;
  //fixtureDef1.density = 1;
  b2PolygonShape polygonshape; 
 // shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
 //shape definition
    b2PolygonShape polygonShape;
    polygonShape.SetAsBox(1, 1); //a 2x2 rectangle
  b2BodyDef myBodyDef;
    myBodyDef.type = b2_dynamicBody;
   myBodyDef.type = b2_staticBody;
    myBodyDef.position.Set(0, 0);
    b2Body* staticBody = m_world->CreateBody(&myBodyDef);
    
    //fixture definition
    b2FixtureDef myFixtureDef;
    myFixtureDef.shape = &polygonShape;
    myFixtureDef.density = 1;
   polygonShape.SetAsBox( 200, 13, b2Vec2(0, 0), 0);//ground
    staticBody->CreateFixture(&myFixtureDef);
 
  //two shapes
  b2PolygonShape boxShape;
  boxShape.SetAsBox(10,5);
  b2CircleShape circleShape1;
  b2CircleShape circleShape2;
  circleShape1.m_radius = 2; 
  circleShape2.m_radius = 2;     
  
  //make box a little to the left
  bodyDef.position.Set(4.5, 20);
  fixtureDef.shape = &boxShape;
  m_bodyA = m_world->CreateBody(&bodyDef);
  m_bodyA->CreateFixture( &fixtureDef );
  
  //and circle a little to the right
  
  bodyDef.position.Set( 3, 10);
  fixtureDef.shape = &circleShape1;
  m_bodyB = m_world->CreateBody( &bodyDef );
  m_bodyB->CreateFixture( &fixtureDef );
  
  bodyDef.position.Set( 6, 10);
  fixtureDef.shape = &circleShape2;
  m_bodyC = m_world->CreateBody( &bodyDef );
  m_bodyC->CreateFixture( &fixtureDef );
  //defining joints

  revoluteJointDef.bodyA = m_bodyA;
  revoluteJointDef.bodyB = m_bodyB;
  revoluteJointDef.collideConnected = false;
  revoluteJointDef.localAnchorA.Set(10,-5);//the top right corner of the box
  revoluteJointDef.localAnchorB.Set(0,0);//center of the circle
  
   m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );
  //place the bodyB anchor at the edge of the circle 
  //revoluteJointDef.localAnchorB.Set(-10,0);
  
  //place the bodyA anchor outside the fixture
  //revoluteJointDef.localAnchorA.Set(30,10);
  /*
  revoluteJointDef.enableLimit = false;
  revoluteJointDef.lowerAngle = -30 * DEGTORAD;
  revoluteJointDef.upperAngle =  30 * DEGTORAD;
  revoluteJointDef.enableMotor = true;
  revoluteJointDef.maxMotorTorque = 15;
  revoluteJointDef.motorSpeed = 90 * DEGTORAD;//90 degrees per second
  
  */
  
  // m_bodyB->SetAngularVelocity(-100);
   //m_bodyC->SetAngularVelocity(-100);
  
  
  
  revoluteJointDef2.bodyA = m_bodyA;
  revoluteJointDef2.bodyB = m_bodyC;
  revoluteJointDef2.collideConnected = false;
  revoluteJointDef2.localAnchorA.Set(-10,-5);//the top right corner of the box
  revoluteJointDef2.localAnchorB.Set(0,0);//center of the circle
  
 m_joint3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef2 );
  //place the bodyB anchor at the edge of /*the circle 
  
  b2PrismaticJointDef prismaticJointDef;
  prismaticJointDef.bodyA = m_bodyA;
  prismaticJointDef.bodyB = m_bodyB;
  prismaticJointDef.collideConnected = false;
 //prismaticJointDef.localAxis1.Set(-18.5,-5);
	}
	
	

	void keyboard(unsigned char key)
  {
    switch (key)
    {
      case 'q': //move left
        m_bodyB->SetAngularVelocity(-100* DEGTORAD);
        break;
      case 'w': //stop
        m_bodyC->SetAngularVelocity(100* DEGTORAD);
        break;
      case 'e': //move right
        m_bodyB->SetAngularVelocity(0);
   m_bodyC->SetAngularVelocity(0);
        break;
        case 'j': //jump
  {
    b2Vec2 vel = m_bodyA->GetLinearVelocity();
    vel.y = 10;//upwards - don't change x velocity
    m_bodyA->SetLinearVelocity( vel );
  break;
}
      default:
        //run default behaviour
        cs296::base_sim_t::keyboard(key);
    }
    
  }
  

    
    static cs296::base_sim_t* create()
    {
      return new dominos_t;
    }
   

  
};
namespace cs296{ 
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}

  
#endif
