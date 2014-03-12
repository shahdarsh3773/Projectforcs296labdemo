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
#include <iostream>
//#include <dos.h>
#include <stdio.h>
//#include <conio.h>

  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  
 
  
  class dominos_t : public  cs296::base_sim_t
  {
  public:
   
   
  //b2Body* body;
        b2Body* m_bodyA;
		b2Body* m_bodyB;
		b2Body* m_bodyC;
		b2Body* m_bodyD;
		b2Body* m_bodyE;
		b2Body* m_bodyF;
		b2Body* m_bodyG;
		b2Body* m_leg11;
		b2Body* m_leg12;
		b2Body* m_leg21;
		b2Body* m_leg22;
		b2Body* m_leg31;
		b2Body* m_leg32;
		b2Body* m_leg41;
		b2Body* m_leg42;
		
		b2RevoluteJoint* m_joint;
		b2RevoluteJoint* m_joint3;
		b2RevoluteJoint* m_joint2;
		b2RevoluteJoint* m_joint4;
		b2RevoluteJoint* m_joint5;
		b2RevoluteJoint* m_joint6;
		b2RevoluteJoint* m_joint7;
		
		b2RevoluteJointDef revoluteJointDef7;
		b2RevoluteJointDef revoluteJointDef6;
		b2RevoluteJointDef revoluteJointDef5;
		b2RevoluteJointDef revoluteJointDef4;
		b2RevoluteJointDef revoluteJointDef3;
		b2RevoluteJointDef revoluteJointDef2;
		b2RevoluteJointDef revoluteJointDef;
  
    dominos_t(){
	
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  b2FixtureDef fixtureDef;
  fixtureDef.density = 1;
  
  
 b2BodyDef bodyDef1;
  bodyDef1.type = b2_dynamicBody;
  b2FixtureDef fixtureDef1;
 fixtureDef1.density = 0;
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
  /**
  boxShape.SetAsBox(10,5);
  
  b2PolygonShape seat;
  seat.SetAsBox(4,2);
  
  b2PolygonShape man;
  man.SetAsBox(3,4);
  
  b2PolygonShape head;
  head.SetAsBox(2,2);
  
  b2PolygonShape hand;
  hand.SetAsBox(2,1);
  
  b2CircleShape circleShape1;
  b2CircleShape circleShape2;
  circleShape1.m_radius = 2; 
  circleShape2.m_radius = 2;     
  
  //make box a little to the left
  bodyDef.position.Set(4.5, 20);
  fixtureDef.shape = &boxShape;
  m_bodyA = m_world->CreateBody(&bodyDef);
  m_bodyA->CreateFixture( &fixtureDef );
  
  //for seat of man
  bodyDef.position.Set(14.5,18);
  fixtureDef.shape=&seat;
  m_bodyD=m_world->CreateBody(&bodyDef);
  m_bodyD->CreateFixture(&fixtureDef);
  
  //A box mimicing man
  bodyDef1.position.Set(18,22);
  fixtureDef1.shape=&man;
  m_bodyE=m_world->CreateBody(&bodyDef1);
  m_bodyE->CreateFixture(&fixtureDef1);
  
  //head of man
  bodyDef1.position.Set(18,30);
  fixtureDef1.shape=&head;
  m_bodyF=m_world->CreateBody(&bodyDef1);
  m_bodyF->CreateFixture(&fixtureDef1);
  
  //hand of man
  bodyDef1.position.Set(26,23);
  fixtureDef1.shape=&hand;
  m_bodyG=m_world->CreateBody(&bodyDef1);
  m_bodyG->CreateFixture(&fixtureDef1);
 // m_bodyG->SetAngularVelocity(10*DEGTORAD);
  m_bodyG->SetTransform(m_bodyG->GetPosition(),-30*DEGTORAD);
  
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
  
  /**
  
  revoluteJointDef2.bodyA = m_bodyA;
  revoluteJointDef2.bodyB = m_bodyC;
  revoluteJointDef2.collideConnected = false;
  revoluteJointDef2.localAnchorA.Set(-10,-5);//the top right corner of the box
  revoluteJointDef2.localAnchorB.Set(0,0);//center of the circle
  
 m_joint3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef2 );
  //place the bodyB anchor at the edge of /*the circle 
  
  revoluteJointDef3.bodyA = m_bodyA;
  revoluteJointDef3.bodyB = m_bodyD;
  revoluteJointDef3.collideConnected = false;
  revoluteJointDef3.localAnchorA.Set(14,0);//the top right corner of the box
  revoluteJointDef3.localAnchorB.Set(0,0);//center of the circle
  
 m_joint4 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef3 );
  
  revoluteJointDef4.bodyA = m_bodyE;
  revoluteJointDef4.bodyB = m_bodyF;
  revoluteJointDef4.collideConnected = false;
  revoluteJointDef4.localAnchorA.Set(0,6);//the top right corner of the box
  revoluteJointDef4.localAnchorB.Set(0,0);//center of the circle
  
   m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef4);
   
   
  revoluteJointDef5.bodyA = m_bodyE;
  revoluteJointDef5.bodyB = m_bodyG;
  revoluteJointDef5.collideConnected = false;
  revoluteJointDef5.localAnchorA.Set(4,0);//the top right corner of the box
  revoluteJointDef5.localAnchorB.Set(0,0);//center of the circle
  revoluteJointDef5.enableLimit = true;
  revoluteJointDef5.lowerAngle = -10* DEGTORAD;
  revoluteJointDef5.upperAngle =  10* DEGTORAD;
  
  revoluteJointDef5.enableMotor = true;
  revoluteJointDef5.maxMotorTorque = 5;
  revoluteJointDef5.motorSpeed = 90 * DEGTORAD;//90 degrees per second
  
   m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef5);
   **/
   b2Vec2 vertices[8];
  vertices[0].Set(3,  0);
  vertices[1].Set(10,  1);
  vertices[2].Set( 12, 5);
  vertices[3].Set( 11,  7);
  vertices[4].Set( 10,  4.5);
  vertices[5].Set(3,4.5);
  vertices[6].Set(1,3);
	 b2PolygonShape polygonShape1;
   polygonShape1.Set(vertices, 7); //pass array to the shape
  
  myFixtureDef.shape = &polygonShape1; //change the shape of the fixture
  myBodyDef.position.Set(0, 20); //in the middle
  b2Body* dynamicBody2 = m_world->CreateBody(&myBodyDef);
  dynamicBody2->CreateFixture(&myFixtureDef);
 
  b2Vec2 horse_head[5];
  horse_head[0].Set(11,7);
  horse_head[1].Set(12,5);
  horse_head[2].Set(13,6);
  horse_head[3].Set(16,6);
  horse_head[4].Set(13,8);
  
  
  //horse_head[5].Set(13,8);
  
  b2PolygonShape polygonShape2;
   polygonShape2.Set(horse_head,5); //pass array to the shape
  
  myFixtureDef.shape = &polygonShape2; //change the shape of the fixture
  myBodyDef.position.Set(0, 20); //in the middle
  b2Body* dynamicBody3 = m_world->CreateBody(&myBodyDef);
  dynamicBody3->CreateFixture(&myFixtureDef);
  
  
  
  b2PolygonShape leg11;
  leg11.SetAsBox(2,1);
  b2PolygonShape leg12;
  leg12.SetAsBox(2,1);
  b2PolygonShape leg21;
  leg21.SetAsBox(2,1);
  b2PolygonShape leg22;
  leg22.SetAsBox(2,1);
  b2PolygonShape leg31;
  leg31.SetAsBox(2,1);
  b2PolygonShape leg32;
  leg32.SetAsBox(2,1);
  b2PolygonShape leg41;
  leg41.SetAsBox(2,1);
  b2PolygonShape leg42;
  leg42.SetAsBox(2,1);
  
 bodyDef.position.Set(8.5, 20);
  fixtureDef.shape = &leg11;
  m_leg11 = m_world->CreateBody(&bodyDef);
  m_leg11->CreateFixture( &fixtureDef );
  m_leg11->SetTransform(m_leg11->GetPosition(),-45*DEGTORAD);
  
   bodyDef.position.Set(12,17);
  fixtureDef.shape = &leg12;
  m_leg12 = m_world->CreateBody(&bodyDef);
  m_leg12->CreateFixture( &fixtureDef );
  m_leg12->SetTransform(m_leg12->GetPosition(),-60*DEGTORAD);
  
  //ask sir//
  revoluteJointDef6.bodyA = dynamicBody2;
  revoluteJointDef6.bodyB = m_leg11;
  revoluteJointDef6.collideConnected = false;
  revoluteJointDef6.localAnchorA.Set(22,-10);//the top right corner of the box
  revoluteJointDef6.localAnchorB.Set(10,10);//center of the circle
  
   m_joint6 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef6);
   
   
  revoluteJointDef7.bodyA = m_leg11;
  revoluteJointDef7.bodyB = m_leg12;
  revoluteJointDef7.collideConnected = false;
  revoluteJointDef7.localAnchorA.Set(3,0);//the top right corner of the box
  revoluteJointDef7.localAnchorB.Set(0,0);//center of the circle
  
   m_joint7 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7);
  //place the bodyB anchor at the edge of the circle 
  //revoluteJointDef.localAnchorB.Set(-10,0);
  //place the bodyB anchor at the edge of the circle 
  //revoluteJointDef.localAnchorB.Set(-10,0);
  
  /*
   bodyDef.position.Set(2.5, 20);
  fixtureDef.shape = &leg21;
  m_leg21 = m_world->CreateBody(&bodyDef);
  m_leg21->CreateFixture( &fixtureDef );
  
   bodyDef.position.Set(2.5, 18);
  fixtureDef.shape = &leg22;
  m_leg22 = m_world->CreateBody(&bodyDef);
  m_leg22->CreateFixture( &fixtureDef );
  
   bodyDef.position.Set(-0.5, 20);
  fixtureDef.shape = &leg31;
  m_leg31 = m_world->CreateBody(&bodyDef);
  m_leg31->CreateFixture( &fixtureDef );
  
   bodyDef.position.Set(-0.5, 18);
  fixtureDef.shape = &leg32;
  m_leg32 = m_world->CreateBody(&bodyDef);
  m_leg32->CreateFixture( &fixtureDef );
  
   bodyDef.position.Set(-4.5, 20);
  fixtureDef.shape = &leg41;
  m_leg41 = m_world->CreateBody(&bodyDef);
  m_leg41->CreateFixture( &fixtureDef );
  
   bodyDef.position.Set(-4.5, 18);
  fixtureDef.shape = &leg42;
  m_leg42 = m_world->CreateBody(&bodyDef);
  m_leg42->CreateFixture( &fixtureDef );
  **/
	}

	void keyboard(unsigned char key)
  {
    switch (key)
    {
      case 'q': //move left
        m_bodyB->SetAngularVelocity(-100* DEGTORAD);
        m_bodyG->SetTransform(m_bodyG->GetPosition(),+30*DEGTORAD);
        //sleep(1);
       // m_bodyG->SetTransform(m_bodyG->GetPosition(),-30*DEGTORAD);
        //sleep(1);
       //wait::wait(1);
        //m_bodyG->SetTransform(m_bodyG->GetPosition(),-30*DEGTORAD);
        break;
      case 'w': //stop
        m_bodyC->SetAngularVelocity(100* DEGTORAD);
        m_bodyG->SetTransform(m_bodyG->GetPosition(),30*DEGTORAD);
        break;
      case 'e': //move right
   m_bodyB->SetAngularVelocity(0);
   m_bodyC->SetAngularVelocity(0);
   m_bodyG->SetTransform(m_bodyG->GetPosition(),-30*DEGTORAD);
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
