	#ifndef _DOMINOS_HPP_
	#define _DOMINOS_HPP_
	#include "dominos.hpp"
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
	        b2Body* m_bodyA;//Cart Box
			b2Body* m_bodyB;//Forward Tyre 
			b2Body* m_bodyC;//Back tyre
			b2Body* m_bodyD;//Seat of man
			b2Body* m_bodyE;//Man's Body
			b2Body* m_bodyF;//Man's head
			b2Body* m_bodyG;//Man's arm
			b2Body* m_bodyH;//Rod connected to the horse
			b2Body* m_bodyI;//rod2
			b2Body* m_bodyJ;//the whip
			b2Body* m_bodyG1;//mass of the horse
			b2Body* m_bodyG2;//thigh1
			b2Body* m_bodyG3;//calf1
			b2Body* m_bodyG4;//thigh2
			b2Body* m_bodyG5;//calf2
			b2Body* m_bodyGA2;//thigh1
			b2Body* m_bodyGA3;//calf1
			b2Body* m_bodyGA4;//thigh2
			b2Body* m_bodyGA5;
			b2Body* m_bodyK;
			b2RevoluteJoint* m_joint;//Cart and forward tyre
			b2RevoluteJoint* m_joint3;//Cart and back tyre
			b2RevoluteJoint* m_joint2;//Man and seat
			b2RevoluteJoint* m_joint4;//Man and head
			b2RevoluteJoint* m_joint5;//Man and arm
			b2RevoluteJoint* m_joint6;//Front tyre and Rod
			b2RevoluteJoint* m_joint7;
			b2RevoluteJoint* m_joint71;
			b2RevoluteJoint* m_joint72;
			b2RevoluteJoint* m_joint73;
			b2RevoluteJoint* m_joint74;
			b2RevoluteJoint* m_joint75;
			b2RevoluteJoint* m_joint7A2;
			b2RevoluteJoint* m_joint7A3;
			b2RevoluteJoint* m_joint7A4;
			b2RevoluteJoint* m_joint7A5;
			b2RevoluteJoint* m_joint8;
			b2RevoluteJoint* m_joint9;
			b2RevoluteJoint* m_joint10;
			b2RevoluteJointDef revoluteJointDef10;
			b2RevoluteJointDef revoluteJointDef9;
			b2RevoluteJointDef revoluteJointDef8;
			b2RevoluteJointDef revoluteJointDef75;
			b2RevoluteJointDef revoluteJointDef74;
			b2RevoluteJointDef revoluteJointDef73;
			b2RevoluteJointDef revoluteJointDef72;
			b2RevoluteJointDef revoluteJointDef7A5;
			b2RevoluteJointDef revoluteJointDef7A4;
			b2RevoluteJointDef revoluteJointDef7A3;
			b2RevoluteJointDef revoluteJointDef7A2;
			b2RevoluteJointDef revoluteJointDef71;
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
	  fixtureDef.density = 100;
	  
	  b2BodyDef bodyDef2;
	  bodyDef2.type = b2_dynamicBody;
	  b2FixtureDef fixtureDef2;
	  fixtureDef2.density = 5;
	  
	  
	 b2BodyDef bodyDef1;
	  bodyDef1.type = b2_dynamicBody;
	  b2FixtureDef fixtureDef1;
	 fixtureDef1.density = 10;
	  b2PolygonShape polygonshape; 
	 
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
	 
	  //defining shapes
	  b2PolygonShape boxShape;
	  boxShape.SetAsBox(10,5);
	  
	  b2PolygonShape seat;
	  seat.SetAsBox(4,1.5);
	  
	  b2PolygonShape man;
	  man.SetAsBox(3,4);
	  
	  b2PolygonShape head;
	  head.SetAsBox(2,2);
	  
	  b2PolygonShape head1;
	  head1.SetAsBox(3,1.2);
	  
	  b2PolygonShape hand;
	  hand.SetAsBox(2,1);
	  
	  b2PolygonShape Rod;
	  Rod.SetAsBox(12.3,0.1);
	  
	  b2PolygonShape HB;
	  HB.SetAsBox(0.1,3.5);
	  
	  b2PolygonShape stomach;
	  stomach.SetAsBox(8,2);
	  
	  b2PolygonShape thigh1;
	  thigh1.SetAsBox(0.4,2.3);
	  
	  b2PolygonShape irene;
	  irene.SetAsBox(7-0.5,0.1);
	  
	 /* b2PolygonShape thigh1;
	  thigh1.SetAsBox(0.5,5);
	  
	  b2PolygonShape thigh1;
	  thigh1.SetAsBox(0.5,5);
	  
	  b2PolygonShape calf1;
	  thigh1.SetAsBox(0.5,5);
	  
	  b2PolygonShape thigh2;
	  thigh1.SetAsBox(0.5,5);
	  
	  b2PolygonShape calf2;
	  thigh1.SetAsBox(0.5,5);*/
	  
	  b2CircleShape circleShape1;
	  b2CircleShape circleShape2;
	  circleShape1.m_radius = 2; 
	  circleShape2.m_radius = 2;     
	  
	  //make box a little to the left
	  bodyDef.position.Set(4.5-15, 20);
	  fixtureDef.shape = &boxShape;
	  m_bodyA = m_world->CreateBody(&bodyDef);
	  m_bodyA->CreateFixture( &fixtureDef );
	  
	  //for seat of man
	  bodyDef.position.Set(14.5-15,18+1);
	  fixtureDef.shape=&seat;
	  m_bodyD=m_world->CreateBody(&bodyDef);
	  m_bodyD->CreateFixture(&fixtureDef);
	  
	  //A box mimicing man
	  bodyDef1.position.Set(18-15,22);
	  fixtureDef1.shape=&man;
	  m_bodyE=m_world->CreateBody(&bodyDef1);
	  m_bodyE->CreateFixture(&fixtureDef1);
	  
	  //head of man
	  bodyDef1.position.Set(18-15,30);
	  fixtureDef1.shape=&head;
	  m_bodyF=m_world->CreateBody(&bodyDef1);
	  m_bodyF->CreateFixture(&fixtureDef1);
	  
	  //hand of man
	  bodyDef1.position.Set(26-15,23);
	  fixtureDef1.shape=&hand;
	  m_bodyG=m_world->CreateBody(&bodyDef1);
	  m_bodyG->CreateFixture(&fixtureDef1);
	  m_bodyG->SetTransform(m_bodyG->GetPosition(),15*DEGTORAD);
	  
	  bodyDef1.position.Set(31-15,23);
	  fixtureDef1.shape=&irene;
	  m_bodyJ=m_world->CreateBody( &bodyDef1 );
	  m_bodyJ->CreateFixture( &fixtureDef1 );
	  m_bodyJ->SetTransform(m_bodyG->GetPosition(),15*DEGTORAD);
	 
	  //and circle a little to the right
	  
	  bodyDef.position.Set( 3-15, 10);
	  fixtureDef.shape = &circleShape1;
	  m_bodyB = m_world->CreateBody( &bodyDef );
	  m_bodyB->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 6-15, 10);
	  fixtureDef.shape = &circleShape2;
	  m_bodyC = m_world->CreateBody( &bodyDef );
	  m_bodyC->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 16+2-15, 10+3);
	  fixtureDef.shape= &Rod;
	  m_bodyH = m_world->CreateBody( &bodyDef );
	  m_bodyH->CreateFixture( &fixtureDef );
	  
	  /*bodyDef.position.Set( 26, 13.5);
	  fixtureDef.shape= &HB;
	  m_bodyI = m_world->CreateBody( &bodyDef );
	  m_bodyI->CreateFixture( &fixtureDef );*/
	  
	  bodyDef.position.Set( 26+4-15, 16);
	  fixtureDef.shape= &stomach;
	  m_bodyG1 = m_world->CreateBody( &bodyDef );
	  m_bodyG1->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 24.5-15, 14.75);
	  fixtureDef.shape= &thigh1;
	  m_bodyG2 = m_world->CreateBody( &bodyDef );
	  m_bodyG2->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 24.5-15+1, 14.75);
	  fixtureDef.shape= &thigh1;
	  m_bodyGA2 = m_world->CreateBody( &bodyDef );
	  m_bodyGA2->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 27.5+8-15, 14.75);
	  fixtureDef.shape= &thigh1;
	  m_bodyG3 = m_world->CreateBody( &bodyDef );
	  m_bodyG3->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 27.5+8-15-1, 14.75);
	  fixtureDef.shape= &thigh1;
	  m_bodyGA3 = m_world->CreateBody( &bodyDef );
	  m_bodyGA3->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 24.5-15, 10.25);
	  fixtureDef.shape= &thigh1;
	  m_bodyG4 = m_world->CreateBody( &bodyDef);
	  m_bodyG4->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 24.5-15+1, 10.25);
	  fixtureDef.shape= &thigh1;
	  m_bodyGA4 = m_world->CreateBody( &bodyDef);
	  m_bodyGA4->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 27.5+8-15, 10.25);
	  fixtureDef.shape= &thigh1;
	  m_bodyG5 = m_world->CreateBody( &bodyDef );
	  m_bodyG5->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 27.5+8-15-1, 10.25);
	  fixtureDef.shape= &thigh1;
	  m_bodyGA5 = m_world->CreateBody( &bodyDef );
	  m_bodyGA5->CreateFixture( &fixtureDef );
	  
	  bodyDef.position.Set( 30-15+8, 19);
	  fixtureDef.shape= &head1;
	  m_bodyK = m_world->CreateBody( &bodyDef );
	  m_bodyK->CreateFixture( &fixtureDef );
	
	  revoluteJointDef.bodyA = m_bodyA;
	  revoluteJointDef.bodyB = m_bodyB;
	  revoluteJointDef.collideConnected = false;
	  revoluteJointDef.localAnchorA.Set(10,-5);//the top right corner of the box
	  revoluteJointDef.localAnchorB.Set(0,0);//center of the circle
	  
	   m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );
	 
	  
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
	  revoluteJointDef3.localAnchorA.Set(18,0);//the top right corner of the box
	  revoluteJointDef3.localAnchorB.Set(4,0);//center of the circle
	  revoluteJointDef3.enableLimit = false;
	   m_joint4 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef3 ); 
	  /*revoluteJointDef10.bodyA = m_bodyA;
	  revoluteJointDef10.bodyB = m_bodyD;
	  revoluteJointDef10.collideConnected = false;
	  revoluteJointDef10.localAnchorA.Set(-10,0);//the top right corner of the box
	  revoluteJointDef10.localAnchorB.Set(0,0);//center of the circle
	  revoluteJointDef10.enableLimit = false;
	  m_joint10 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef10 );*/
	 
	  
	  revoluteJointDef4.bodyA = m_bodyE;
	  revoluteJointDef4.bodyB = m_bodyF;
	  revoluteJointDef4.collideConnected = false;
	  revoluteJointDef4.localAnchorA.Set(0,6);//the top right corner of the box
	  revoluteJointDef4.localAnchorB.Set(0,0);//center of the circle
	  revoluteJointDef4.enableLimit = false;
	  
	   m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef4);
	   
	   
	  revoluteJointDef5.bodyA = m_bodyE;
	  revoluteJointDef5.bodyB = m_bodyG;
	  revoluteJointDef5.collideConnected = false;
	  revoluteJointDef5.localAnchorA.Set(4,0);//the top right corner of the box
	  revoluteJointDef5.localAnchorB.Set(0,0);//center of the circle
	  revoluteJointDef5.enableLimit = true;
	  revoluteJointDef5.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef5.upperAngle = -10* DEGTORAD;
	   
	   m_joint5 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef5);
	   
	  revoluteJointDef6.bodyA = m_bodyB;
	  revoluteJointDef6.bodyB = m_bodyH;
	  revoluteJointDef6.collideConnected = false;
	  revoluteJointDef6.localAnchorA.Set(0,0);
	  revoluteJointDef6.localAnchorB.Set(-12.3,0.0);
	  //revoluteJointDef6.enableLimit = true;
	  revoluteJointDef6.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef6.upperAngle = -10* DEGTORAD;
	   
	   m_joint6 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef6);
	   
	  /*revoluteJointDef7.bodyA = m_bodyI;
	  revoluteJointDef7.bodyB = m_bodyH;
	  revoluteJointDef7.collideConnected = false;
	  revoluteJointDef7.localAnchorA.Set(0,-3.5);
	  revoluteJointDef7.localAnchorB.Set(12,0.0);
	  revoluteJointDef7.enableLimit = true;
	  revoluteJointDef7.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7.upperAngle = -10* DEGTORAD;*/
	  
	  revoluteJointDef7.bodyA = m_bodyG1;
	  revoluteJointDef7.bodyB = m_bodyH;
	  revoluteJointDef7.collideConnected = false;
	  revoluteJointDef7.localAnchorA.Set(0,0);
	  revoluteJointDef7.localAnchorB.Set(12.3,0.0);
	  revoluteJointDef7.enableLimit = true;
	  revoluteJointDef7.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7.upperAngle = -10* DEGTORAD;
	   
	   m_joint7 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7);
	   
	  /*revoluteJointDef71.bodyA = m_bodyI;
	  revoluteJointDef71.bodyB = m_bodyG1;
	  revoluteJointDef71.collideConnected = false;
	  revoluteJointDef71.localAnchorA.Set(0,3.5);
	  revoluteJointDef71.localAnchorB.Set(0,0.0);
	  revoluteJointDef71.enableLimit = true;
	  revoluteJointDef71.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef71.upperAngle = -10* DEGTORAD;
	   
	   m_joint71 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef71);*/
	   
	  revoluteJointDef72.bodyA = m_bodyG1;
	  revoluteJointDef72.bodyB = m_bodyG2;
	  revoluteJointDef72.collideConnected = false;
	  revoluteJointDef72.localAnchorA.Set(-6,0);
	  revoluteJointDef72.localAnchorB.Set(0,2.25);
	  revoluteJointDef72.enableLimit = true;
	  revoluteJointDef72.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef72.upperAngle = -10* DEGTORAD;
	   
	   m_joint72 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef72);
	   
	  revoluteJointDef73.bodyA = m_bodyG1;
	  revoluteJointDef73.bodyB = m_bodyG3;
	  revoluteJointDef73.collideConnected = false;
	  revoluteJointDef73.localAnchorA.Set(6,0);
	  revoluteJointDef73.localAnchorB.Set(0,2.25);
	  revoluteJointDef73.enableLimit = true;
	  revoluteJointDef73.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef73.upperAngle = -10* DEGTORAD;
	   
	   m_joint73 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef73);
	   
	  revoluteJointDef74.bodyA = m_bodyG2;
	  revoluteJointDef74.bodyB = m_bodyG4;
	  revoluteJointDef74.collideConnected = false;
	  revoluteJointDef74.localAnchorA.Set(0,-2.25);
	  revoluteJointDef74.localAnchorB.Set(0,2.25);
	  revoluteJointDef74.enableLimit = true;
	  revoluteJointDef74.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef74.upperAngle = -10* DEGTORAD;
	   
	   m_joint74 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef74);
	   
	  revoluteJointDef75.bodyA = m_bodyG3;
	  revoluteJointDef75.bodyB = m_bodyG5;
	  revoluteJointDef75.collideConnected = false;
	  revoluteJointDef75.localAnchorA.Set(0,-2.25);
	  revoluteJointDef75.localAnchorB.Set(0,2.25);
	  revoluteJointDef75.enableLimit = true;
	  revoluteJointDef75.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef75.upperAngle = -10* DEGTORAD;
	   
	   m_joint75 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef75);
	   
	  revoluteJointDef7A2.bodyA = m_bodyG1;
	  revoluteJointDef7A2.bodyB = m_bodyGA2;
	  revoluteJointDef7A2.collideConnected = false;
	  revoluteJointDef7A2.localAnchorA.Set(-6,0);
	  revoluteJointDef7A2.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A2.enableLimit = true;
	  revoluteJointDef7A2.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7A2.upperAngle = -10* DEGTORAD;
	   
	   m_joint7A2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A2);
	   
	  revoluteJointDef7A3.bodyA = m_bodyG1;
	  revoluteJointDef7A3.bodyB = m_bodyGA3;
	  revoluteJointDef7A3.collideConnected = false;
	  revoluteJointDef7A3.localAnchorA.Set(6,0);
	  revoluteJointDef7A3.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A3.enableLimit = true;
	  revoluteJointDef7A3.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7A3.upperAngle = -10* DEGTORAD;
	   
	   m_joint7A3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A3);
	   
	  revoluteJointDef7A4.bodyA = m_bodyGA2;
	  revoluteJointDef7A4.bodyB = m_bodyGA4;
	  revoluteJointDef7A4.collideConnected = false;
	  revoluteJointDef7A4.localAnchorA.Set(0,-2.25);
	  revoluteJointDef7A4.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A4.enableLimit = true;
	  revoluteJointDef7A4.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7A4.upperAngle = -10* DEGTORAD;
	   
	   m_joint7A4 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A4);
	   
	  revoluteJointDef7A5.bodyA = m_bodyGA3;
	  revoluteJointDef7A5.bodyB = m_bodyGA5;
	  revoluteJointDef7A5.collideConnected = false;
	  revoluteJointDef7A5.localAnchorA.Set(0,-2.25);
	  revoluteJointDef7A5.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A5.enableLimit = true;
	  revoluteJointDef7A5.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7A5.upperAngle = -10* DEGTORAD;
	   
	   m_joint7A5 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A5);
	   
	  revoluteJointDef8.bodyA = m_bodyG;
	  revoluteJointDef8.bodyB = m_bodyJ;
	  revoluteJointDef8.collideConnected = false;
	  revoluteJointDef8.localAnchorA.Set(1,0);
	  revoluteJointDef8.localAnchorB.Set(-7+0.5,0);
	  revoluteJointDef8.enableLimit = true;
	  revoluteJointDef8.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef8.upperAngle = -10* DEGTORAD;
	   
	   m_joint8 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef8);
	   
	  revoluteJointDef9.bodyA = m_bodyG1;
	  revoluteJointDef9.bodyB = m_bodyK;
	  revoluteJointDef9.collideConnected = false;
	  revoluteJointDef9.localAnchorA.Set(8,2);
	  revoluteJointDef9.localAnchorB.Set(0,0);
	  revoluteJointDef9.enableLimit = true;
	  revoluteJointDef9.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef9.upperAngle = -10* DEGTORAD;
	   
	   m_joint9 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef9);
	   
	 
		}
		
		
	
		void keyboard(unsigned char key)
	  {
	    switch (key)
	    {
	      case 'q': //move left
	        m_bodyB->SetAngularVelocity(100* DEGTORAD);
	        m_bodyG->SetTransform(m_bodyG->GetPosition(),+15*DEGTORAD);
	        m_bodyJ->SetTransform(m_bodyG->GetPosition(),+15*DEGTORAD);
	        m_bodyB->SetAngularVelocity(0);
	        m_bodyC->SetAngularVelocity(0);
	        break;
	      case 'w': //move right
	        m_bodyC->SetAngularVelocity(-100* DEGTORAD);
	        m_bodyG->SetTransform(m_bodyG->GetPosition(),15*DEGTORAD);
	        m_bodyJ->SetTransform(m_bodyJ->GetPosition(),15*DEGTORAD);
	        m_bodyB->SetAngularVelocity(0);
	        m_bodyC->SetAngularVelocity(0);
	        m_bodyG1->SetLinearVelocity( b2Vec2(0,0) );
	        m_bodyC->SetLinearVelocity( b2Vec2(0,0) );
	        m_bodyB->SetLinearVelocity( b2Vec2(0,0) );
	        break;
	      case 'e': //stop
	   m_bodyG->SetTransform(m_bodyG->GetPosition(),-65*DEGTORAD);
	   m_bodyJ->SetTransform(m_bodyJ->GetPosition(),-65*DEGTORAD);
	   //m_bodyG1->SetLinearVelocity( b2Vec2(0,4) );
	   //for (int i=0; i<1000000; i++);
	   //m_bodyG1->SetLinearVelocity( b2Vec2(20,-2) );
	   //for (int j=0; j<100; j++)
	   //{
		//for(int i=0; i<10; i++)
		//{
		/*m_bodyK->SetAngularVelocity(-100);
		m_bodyG2->SetAngularVelocity(40);
	    m_bodyG3->SetAngularVelocity(40);
	    m_bodyG4->SetAngularVelocity(-200);
	    m_bodyG5->SetAngularVelocity(-200);*/
	    /*for ( int i=0; i<100; i++)
	    {m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (0.45*i)*DEGTORAD );
	    m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (0.45*i)*DEGTORAD );}*/
	    for(int i=1; i<100000; i++){
		/*m_bodyG2->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	    m_bodyG3->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	    m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );*/
	    m_bodyG2->SetTransform( m_bodyG2->GetPosition(), -(0.00045*i)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), -(0.00125*i)*DEGTORAD );
	    m_bodyG3->SetTransform( m_bodyG3->GetPosition(), -(0.00045*i)*DEGTORAD );
	    m_bodyG5->SetTransform( m_bodyG5->GetPosition(), -(0.00125*i)*DEGTORAD );
	    m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), -(0.00045*i)*DEGTORAD );
	    m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), -(0.00125*i)*DEGTORAD );
	    m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), -(0.00045*i)*DEGTORAD );
	    m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), -(0.00125*i)*DEGTORAD );
	    }
	    m_bodyG1->SetLinearVelocity( b2Vec2(20,0) );
	    for(int k=0; k<1000000; k++){
	    for(int j=0; j<1000000; j++)
	    for(int i=0; i<1000000; i++);}
	   /* for(int i=1; i<100000; i++){
		m_bodyG2->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	    m_bodyG3->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	    m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );*/
	   /* m_bodyG2->SetTransform( m_bodyG4->GetPosition(), (0.00045*i)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (0.00125*i)*DEGTORAD );
	    m_bodyG3->SetTransform( m_bodyG5->GetPosition(), (0.00045*i)*DEGTORAD );
	    m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (0.00125*i)*DEGTORAD );}*/
	    /*m_bodyG2->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	    m_bodyG3->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	    m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );*/
	    //}*/
	    //for (int i=0; i<1000000; i++);
	    //m_bodyG4->SetAngularVelocity(-4);
	    //m_bodyG5->SetAngularVelocity(-4);
	   //}
	    //m_bodyC->SetAngularVelocity(10);
	    //m_bodyB->SetAngularVelocity(10);
	    //m_bodyB->SetLinearVelocity( b2Vec2(20,0) );
	    //m_bodyC->SetLinearVelocity( b2Vec2(20,0) );
	   //m_bodyG1->ApplyForce( b2Vec2(1000,0), m_bodyG1->GetWorldCenter(),true );
	        break;
	       /* case 'j': //jump
	  {
	    b2Vec2 vel = m_bodyA->GetLinearVelocity();
	    vel.y = 10;//upwards - don't change x velocity
	    m_bodyA->SetLinearVelocity( vel );
	  break;
	}*/
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
	
