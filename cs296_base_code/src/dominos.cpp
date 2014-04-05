	#ifndef _DOMINOS_HPP_
	#define _DOMINOS_HPP_
	#include "dominos.hpp"
	#include "callbacks.hpp"
	#include "cs296_base.hpp"
	#include <iostream>
	#include <unistd.h>
	//#include <dos.h>
	#include <stdio.h>
	//#include <conio.h>
	
	  //! This is the class that sets up the Box2D simulation world
	  //! Notice the public inheritance - why do we inherit the base_sim_t class?
	  
	 
	  
	  class dominos_t : public  cs296::base_sim_t
	  {
	  public:
	  int state;
		int vel;
	   
	  //b2Body* body;
	        b2Body* m_bodyA;//Cart Box
			b2Body* m_bodyB;//Forward Tyre 
			b2Body* m_bodyC;//Back tyre
			b2Body* m_bodyD;//Seat of man
			b2Body* m_bodyE;//Man's Body
			b2Body* m_bodyE1;
			b2Body* m_bodyF;//Man's neck
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
			b2WeldJoint* m_joint6;//Front tyre and Rod
			b2WeldJoint* m_joint7;
			b2RevoluteJoint* m_jointd;
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
			b2RevoluteJoint* m_joint41;
			b2RevoluteJoint* m_jointd1;
			 b2WeldJoint* m_jointnew;
			 b2WeldJoint* m_joinseat;
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
			b2WeldJointDef revoluteJointDef7;
			b2WeldJointDef revoluteJointDef6;
			b2RevoluteJointDef revoluteJointDef5;
			b2RevoluteJointDef revoluteJointDef4;
			b2RevoluteJointDef revoluteJointDef3;
			b2RevoluteJointDef revoluteJointDef2;
			b2RevoluteJointDef revoluteJointDef;
			b2RevoluteJointDef revoluteJointDef31;
			b2RevoluteJointDef revoluteJointDef81;
			 b2WeldJointDef	weldJointDef;
			 b2WeldJointDef jointDef;
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
	  
	  
	  b2FixtureDef fix1;
	  fix1.density=5;
	 // b2PolygonShape polygonshape; 
	 
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
	  boxShape.SetAsBox(10,4);//CHANGED
	  
	  b2PolygonShape seat;
	  seat.SetAsBox(4,1.5);
	  
	  b2PolygonShape man;
	  man.SetAsBox(3,4);
	  
	  b2PolygonShape head;
	  head.SetAsBox(0.7,1);
	  
	  b2PolygonShape head1;
	  	b2Vec2 vertices3[5];
		vertices3[0].Set(0,0);
		vertices3[1].Set(0,2.25);
		vertices3[2].Set(3,2.25);
		vertices3[3].Set(1.5,0);
		vertices3[4].Set(5,-1);
     	head1.Set(vertices3,5);
	  
	  b2PolygonShape hand;
	  hand.SetAsBox(2,1);

	 b2PolygonShape arjun;
	 arjun.SetAsBox(3,4);
	 
	 b2PolygonShape arjunhead;
	 arjunhead.SetAsBox(2,2);
	 
     b2PolygonShape arjunarm;
     arjunarm.SetAsBox(2,1);
     
     b2PolygonShape spear;
     spear.SetAsBox(6,1);
	  
	  
	  
	  b2PolygonShape Rod;
	  Rod.SetAsBox(8.3,0.1);
	  
	  b2PolygonShape HB;
	  HB.SetAsBox(0.1,3.5);
	  
	  b2PolygonShape stomach;
		b2Vec2 vertices1[6];
		vertices1[0].Set(-8,-2.5);
		vertices1[1].Set(-10,1.5);
		vertices1[2].Set(-8,2.5);
		vertices1[3].Set(6,3.5);
		vertices1[4].Set(8,2.5);
		vertices1[5].Set(7,-2.3);
		stomach.Set(vertices1,7);
	  
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
	  circleShape1.m_radius = 3; 
	  circleShape2.m_radius = 3;     
	  
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
	  
	  /*bodyDef1.position.Set(18-25,20);
	  fixtureDef1.shape=&arjun;
	  m_bodyE1=m_world->CreateBody(&bodyDef1);
	  m_bodyE1->CreateFixture(&fixtureDef1);*/
	  
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
	  
	  bodyDef.position.Set(0,10);
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
	  
	    b2PolygonShape shape;
			shape.SetAsBox(0.8f, 0.225f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			fd.friction = 0.2f;
			const float32 y = 15.0f;
			b2Body* prevBody = m_bodyG1;
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.5f , y);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			b2RevoluteJointDef jd;
			jd.bodyA = m_bodyG1;
			jd.bodyB = body;
			jd.collideConnected = false;
			jd.localAnchorA.Set(-8,0);//the top right corner of the box
			jd.localAnchorB.Set(0,0);
			b2RevoluteJoint* fic;
			fic = (b2RevoluteJoint*)m_world->CreateJoint( &jd);
			prevBody=body;
			for (int32 i = 1; i < 6; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.5f + i, y);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
				b2Vec2 anchor(float32(i), y);
				jd.Initialize(prevBody, body,anchor);
				m_world->CreateJoint(&jd);
				prevBody = body;
			}
	  
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
	  m_bodyK->SetTransform(m_bodyK->GetPosition(),60*DEGTORAD);
	
	  revoluteJointDef.bodyA = m_bodyA;
	  revoluteJointDef.bodyB = m_bodyB;
	  revoluteJointDef.collideConnected = false;
	  revoluteJointDef.localAnchorA.Set(10,-4);//the top right corner of the box CHANGED
	  revoluteJointDef.localAnchorB.Set(0,0);//center of the circle
	  
	   m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );
	 
	      
           jointDef.bodyA = m_bodyD;
           jointDef.bodyB = m_bodyE;
           jointDef.collideConnected = false;
           jointDef.localAnchorA.Set(0,5.5);//the top right corner of the box
	       jointDef.localAnchorB.Set(0,0);
          
           m_jointnew=(b2WeldJoint*)m_world->CreateJoint( &jointDef );
          
           //jointDef.localAnchorA.set(jointDef.bodyA.getLocalPoint(worldCoordsAnchorPoint));
           //jointDef.localAnchorB.set(jointDef.bodyB.getLocalPoint(worldCoordsAnchorPoint));
           //jointDef.referenceAngle = jointDef.bodyB.getAngle() - jointDef.bodyA.getAngle();

	 /*
	  revoluteJointDef8.bodyA = m_bodyD;
	  revoluteJointDef8.bodyB = m_bodyE;
	  revoluteJointDef8.collideConnected = false;
	  revoluteJointDef8.localAnchorA.Set(0,1.5);//the top right corner of the box
	  revoluteJointDef8.localAnchorB.Set(0,-4);//center of the circle
	  
	   m_jointd = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef8);
	   
	   revoluteJointDef81.bodyA = m_bodyD;
	  revoluteJointDef81.bodyB = m_bodyE;
	  revoluteJointDef81.collideConnected = false;
	  revoluteJointDef81.localAnchorA.Set(0,1.5);//the top right corner of the box
	  revoluteJointDef81.localAnchorB.Set(0,-4);//center of the circle
	  
	   m_jointd1 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef81);
	   * */
	  
	  revoluteJointDef2.bodyA = m_bodyA;
	  revoluteJointDef2.bodyB = m_bodyC;
	  revoluteJointDef2.collideConnected = false;
	  revoluteJointDef2.localAnchorA.Set(-10,-4);//the top right corner of the box CHANGED
	  revoluteJointDef2.localAnchorB.Set(0,0);//center of the circle
	  
	 m_joint3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef2 );
	  //place the bodyB anchor at the edge of /*the circle 
	  
	 
	  weldJointDef.bodyA = m_bodyA;
	  weldJointDef.bodyB = m_bodyD;
	  weldJointDef.collideConnected = false;
	  weldJointDef.localAnchorA.Set(10,2);//the top right corner of the box
	  weldJointDef.localAnchorB.Set(-4,0);//center of the circle
	  //weldJointDef.enableLimit = false;
	   m_joinseat = (b2WeldJoint*)m_world->CreateJoint( &weldJointDef ); 
	   
	   
	   
	 
	  revoluteJointDef4.bodyA = m_bodyE;
	  revoluteJointDef4.bodyB = m_bodyF;
	  revoluteJointDef4.collideConnected = false;
	  revoluteJointDef4.localAnchorA.Set(0,5);//the top right corner of the box
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
	   
	  revoluteJointDef6.bodyA = m_bodyD;
	  revoluteJointDef6.bodyB = m_bodyH;
	  revoluteJointDef6.collideConnected = false;
	  revoluteJointDef6.localAnchorA.Set(0,0);
	  revoluteJointDef6.localAnchorB.Set(-8.3,0.0);
	  
	   
	   m_joint6 = (b2WeldJoint*)m_world->CreateJoint( &revoluteJointDef6);
	   
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
	  revoluteJointDef7.localAnchorB.Set(8.3,0.0);
	 
	   
	   m_joint7 = (b2WeldJoint*)m_world->CreateJoint( &revoluteJointDef7);
	   
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
	  revoluteJointDef72.enableMotor = true;
	  revoluteJointDef72.maxMotorTorque = 1000;
	  //revoluteJointDef72.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef72.upperAngle = -5* DEGTORAD;
	   
	   m_joint72 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef72);
	   
	  revoluteJointDef73.bodyA = m_bodyG1;
	  revoluteJointDef73.bodyB = m_bodyG3;
	  revoluteJointDef73.collideConnected = false;
	  revoluteJointDef73.localAnchorA.Set(6,0);
	  revoluteJointDef73.localAnchorB.Set(0,2.25);
	  revoluteJointDef73.enableLimit = true;
	  revoluteJointDef73.enableLimit = true;
	  revoluteJointDef73.enableMotor = true;
	  revoluteJointDef73.maxMotorTorque = 1000;
	  
	  //revoluteJointDef73.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef73.upperAngle = -5* DEGTORAD;
	   
	   m_joint73 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef73);
	   
	  revoluteJointDef74.bodyA = m_bodyG2;
	  revoluteJointDef74.bodyB = m_bodyG4;
	  revoluteJointDef74.collideConnected = false;
	  revoluteJointDef74.localAnchorA.Set(0,-2.25);
	  revoluteJointDef74.localAnchorB.Set(0,2.25);
	  revoluteJointDef74.enableLimit = true;
	  revoluteJointDef74.enableLimit = true;
	  revoluteJointDef74.enableMotor = true;
	  revoluteJointDef74.maxMotorTorque = 1000;
	  //revoluteJointDef74.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef74.upperAngle = -5* DEGTORAD;
	   
	   m_joint74 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef74);
	   
	  revoluteJointDef75.bodyA = m_bodyG3;
	  revoluteJointDef75.bodyB = m_bodyG5;
	  revoluteJointDef75.collideConnected = false;
	  revoluteJointDef75.localAnchorA.Set(0,-2.25);
	  revoluteJointDef75.localAnchorB.Set(0,2.25);
	  revoluteJointDef75.enableLimit = true;
	  revoluteJointDef75.enableLimit = true;
	  revoluteJointDef75.enableMotor = true;
	  revoluteJointDef75.maxMotorTorque = 1000;
	  //revoluteJointDef75.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef75.upperAngle = -5* DEGTORAD;
	   
	   m_joint75 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef75);
	   
	  revoluteJointDef7A2.bodyA = m_bodyG1;
	  revoluteJointDef7A2.bodyB = m_bodyGA2;
	  revoluteJointDef7A2.collideConnected = false;
	  revoluteJointDef7A2.localAnchorA.Set(-6,0);
	  revoluteJointDef7A2.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A2.enableLimit = true;
	  revoluteJointDef7A2.enableLimit = true;
	  revoluteJointDef7A2.enableMotor = true;
	  revoluteJointDef7A2.maxMotorTorque = 1000;
	  //revoluteJointDef7A2.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef7A2.upperAngle = -5* DEGTORAD;
	   
	   m_joint7A2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A2);
	   
	  revoluteJointDef7A3.bodyA = m_bodyG1;
	  revoluteJointDef7A3.bodyB = m_bodyGA3;
	  revoluteJointDef7A3.collideConnected = false;
	  revoluteJointDef7A3.localAnchorA.Set(6,0);
	  revoluteJointDef7A3.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A3.enableLimit = true;
	  revoluteJointDef7A3.enableLimit = true;
	  revoluteJointDef7A3.enableMotor = true;
	  revoluteJointDef7A3.maxMotorTorque = 1000;
	  //revoluteJointDef7A3.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef7A3.upperAngle = -5* DEGTORAD;
	   
	   m_joint7A3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A3);
	   
	  revoluteJointDef7A4.bodyA = m_bodyGA2;
	  revoluteJointDef7A4.bodyB = m_bodyGA4;
	  revoluteJointDef7A4.collideConnected = false;
	  revoluteJointDef7A4.localAnchorA.Set(0,-2.25);
	  revoluteJointDef7A4.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A4.enableLimit = true;
	  revoluteJointDef7A4.enableLimit = true;
	  revoluteJointDef7A4.enableMotor = true;
	  revoluteJointDef7A4.maxMotorTorque = 1000;
	  //revoluteJointDef7A4.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef7A4.upperAngle = -5* DEGTORAD;
	   
	   m_joint7A4 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A4);
	   
	  revoluteJointDef7A5.bodyA = m_bodyGA3;
	  revoluteJointDef7A5.bodyB = m_bodyGA5;
	  revoluteJointDef7A5.collideConnected = false;
	  revoluteJointDef7A5.localAnchorA.Set(0,-2.25);
	  revoluteJointDef7A5.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A5.enableLimit = true;
	  revoluteJointDef7A5.enableLimit = true;
	  revoluteJointDef7A5.enableMotor = true;
	  revoluteJointDef7A5.maxMotorTorque = 1000;
	  //revoluteJointDef7A5.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef7A5.upperAngle = -5* DEGTORAD;
	   
	   m_joint7A5 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A5);
	   
	  revoluteJointDef8.bodyA = m_bodyG;
	  revoluteJointDef8.bodyB = m_bodyJ;
	  revoluteJointDef8.collideConnected = false;
	  revoluteJointDef8.localAnchorA.Set(1,0);
	  revoluteJointDef8.localAnchorB.Set(-7+0.5,0);
	  revoluteJointDef8.enableLimit = true;
	  //revoluteJointDef8.lowerAngle = 5* DEGTORAD;
	  //revoluteJointDef8.upperAngle = -5* DEGTORAD;
	   
	   m_joint8 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef8);
	   
	  revoluteJointDef9.bodyA = m_bodyG1;
	  revoluteJointDef9.bodyB = m_bodyK;
	  revoluteJointDef9.collideConnected = false;
	  revoluteJointDef9.localAnchorA.Set(8,2);
	  revoluteJointDef9.localAnchorB.Set(0,1);
	  revoluteJointDef9.enableLimit = true;
	  revoluteJointDef9.lowerAngle = 5* DEGTORAD;
	  revoluteJointDef9.upperAngle = -5* DEGTORAD;
	   
	   m_joint9 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef9);
	   state=0;
	   vel=0;
	   
	   	   //man2's body 
	   b2Body *man1;
	   b2BodyDef manDef1;
		b2FixtureDef manFix1;
		
		b2PolygonShape manPoly1;;
	 manPoly1.SetAsBox(3,4);
		manFix1.friction=0;
		manFix1.restitution=0;
		manFix1.shape = &manPoly1;
		
		manDef1.position.Set(-12, 29);
		manDef1.type = b2_dynamicBody;
		man1 = m_world->CreateBody(&manDef1);
		man1->CreateFixture(&manFix1);
	  
	  //man2's neck
	  	   b2Body *manh;
	   b2BodyDef manhDef2;
		b2FixtureDef manhFix2;
		
		b2PolygonShape manhPoly2;;
	 manhPoly2.SetAsBox(0.7,1);
		
		manhFix2.shape = &manhPoly2;
		manhFix2.restitution = 0;
		manhDef2.position.Set(-12, 34);
		manhDef2.type = b2_dynamicBody;
		manh = m_world->CreateBody(&manhDef2);
		manh->CreateFixture(&manhFix2);
		
		//joint between man2's body and his	head
          b2RevoluteJoint* revoluteJoint42;
          b2RevoluteJointDef revoluteJointDef42;
 	  revoluteJointDef42.bodyA = man1;
	  revoluteJointDef42.bodyB = manh;
	  revoluteJointDef42.collideConnected = false;
	  revoluteJointDef42.localAnchorA.Set(0,4);//the top right corner of the box
	  revoluteJointDef42.localAnchorB.Set(0,-1);//center of the circle
	  revoluteJointDef42.enableLimit = false;  
	   
	    revoluteJoint42 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef42);
        
        //joint between man2's body and cart
         b2WeldJoint* joint22;
           b2WeldJointDef jointDef22;
           jointDef22.bodyA = man1;
           jointDef22.bodyB = m_bodyA;
           jointDef22.collideConnected = true;
           jointDef22.localAnchorA.Set(0,-4);
	       jointDef22.localAnchorB.Set(0,4);
          
           joint22=(b2WeldJoint*)m_world->CreateJoint( &jointDef22 );   
      //man2's hand
       b2Body* manhand;
	   b2BodyDef manhandDef;
		b2FixtureDef manhandFix;     
      b2PolygonShape manhandPoly;;
	 manhandPoly.SetAsBox(2,1);
	  manhandFix.shape=&manhandPoly;
	  manhandDef.type = b2_dynamicBody;
	  	     manhandDef.position.Set(-9,28);
	  manhand=m_world->CreateBody(&manhandDef);
	  manhand->CreateFixture(&manhandFix);
	  manhand->SetTransform(manhand->GetPosition(),30*DEGTORAD);
	   
	  //joint between man2 and his hand
	  b2RevoluteJoint* revoluteJoint52;
          b2RevoluteJointDef revoluteJointDef52; 
	  revoluteJointDef52.bodyA = man1;
	  revoluteJointDef52.bodyB = manhand;
	  revoluteJointDef52.collideConnected = false;
	  revoluteJointDef52.localAnchorA.Set(4,0);//the top right corner of the box
	  revoluteJointDef52.localAnchorB.Set(0,0);//center of the circle
	  revoluteJointDef52.enableLimit = true;
	  revoluteJointDef52.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef52.upperAngle = -10* DEGTORAD;  
	  revoluteJoint52 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef52);
	 
	 //spear's rod
	 b2Body* weapon;
	   b2BodyDef weaponDef;
		b2FixtureDef weaponFix;     
      b2PolygonShape weaponPoly;;
	 weaponPoly.SetAsBox(6,0.1f);
	  weaponFix.shape=&weaponPoly;
	  weaponFix.restitution=0;
	  weaponDef.type = b2_dynamicBody;
	  weaponDef.position.Set(-2.5f,32);
	  weapon=m_world->CreateBody(&weaponDef);
	  weapon->CreateFixture(&weaponFix);
	  weapon->SetTransform(weapon->GetPosition(),60*DEGTORAD);
	  
	  
	   b2RevoluteJoint* revoluteJoint82;
          b2RevoluteJointDef revoluteJointDef82; 
	  revoluteJointDef82.bodyA = manhand;
	  revoluteJointDef82.bodyB = weapon;
	  revoluteJointDef82.collideConnected = false;
	  revoluteJointDef82.localAnchorA.Set(1,0);
	  revoluteJointDef82.localAnchorB.Set(-7+0.5,0);
	  revoluteJointDef82.enableLimit = true;
	  revoluteJointDef82.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef82.upperAngle = -10* DEGTORAD;
	 
	 revoluteJoint82 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef82);
	 
	 
	 //front man's head
	  	   b2Body *manhead;
	   b2BodyDef manheadDef;
		b2FixtureDef manheadFix;
		
		b2PolygonShape manheadPoly;;
	 manheadPoly.SetAsBox(2,2);
		
		manheadFix.shape = &manheadPoly;
		manheadFix.restitution = 0;
		manheadDef.position.Set(-12, 34);
		manheadDef.type = b2_dynamicBody;
		manhead = m_world->CreateBody(&manheadDef);
		manhead->CreateFixture(&manheadFix);
		
		
		b2WeldJoint* joint3;
           b2WeldJointDef jointDef3;
           jointDef3.bodyA = m_bodyF;
           jointDef3.bodyB = manhead;
           jointDef3.collideConnected = true;
           jointDef3.localAnchorA.Set(0,1);
	       jointDef3.localAnchorB.Set(0,-2);
	       
	       joint3= (b2WeldJoint*)m_world->CreateJoint( &jointDef3 ); 
	       
	       
	   	 //rear man's head
	  	   b2Body *manh1;
	   b2BodyDef manh1Def;
		b2FixtureDef manh1Fix;
		
		b2PolygonShape manh1Poly;
	 manh1Poly.SetAsBox(2,2);
		
		manh1Fix.shape = &manh1Poly;
		manh1Def.position.Set(-12,35) ;
		manh1Def.type = b2_dynamicBody;
		manh1 = m_world->CreateBody(&manh1Def);
		manh1->CreateFixture(&manh1Fix);
		
	
		   b2RevoluteJoint* revoluteJoint0;
          b2RevoluteJointDef revoluteJointDef0; 
	  revoluteJointDef0.bodyA = manh;
	  revoluteJointDef0.bodyB = manh1;
	  revoluteJointDef0.collideConnected = false;
	  revoluteJointDef0.localAnchorA.Set(0,1);
	  revoluteJointDef0.localAnchorB.Set(0,-2);
	  revoluteJointDef0.enableLimit = false;
	  revoluteJointDef0.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef0.upperAngle = -10* DEGTORAD;
	 
	 revoluteJoint0 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef0);
		
	/*	b2WeldJoint* joint0;
           b2WeldJointDef jointDef0;
           jointDef0.bodyA = manh1;
           jointDef0.bodyB = manh;
           jointDef0.collideConnected = true;
           jointDef0.localAnchorA.Set(0,-2);
	       jointDef0.localAnchorB.Set(0,1);
	       joint0= (b2WeldJoint*)m_world->CreateJoint( &jointD); */
		
		     /* b2RevoluteJoint* revoluteJoint421;
          b2RevoluteJointDef revoluteJointDef421;
 	  revoluteJointDef421.bodyA = manhead;
	  revoluteJointDef421.bodyB = m_bodyF;
	  revoluteJointDef421.collideConnected = false;
	  revoluteJointDef421.localAnchorA.Set(0,-2);//the top right corner of the box
	  revoluteJointDef421.localAnchorB.Set(0,1);//center of the circle
	  revoluteJointDef421.enableLimit = false;  
	  revoluteJoint421 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef421);*/
	 
	 //spear's corner tringle
	    b2Body *tri;
		b2BodyDef triDef;
		b2FixtureDef triFix;
		
		b2PolygonShape triPoly;
		b2Vec2 vertices2[3];
		vertices2[0].Set(-2,1.0f);
    
		vertices2[1].Set(0,0.5f);
	   vertices2[2].Set(-2,0);
		triPoly.Set(vertices2, 3);
		
		triFix.shape = &triPoly;
		
		triDef.position.Set(2,38.2f);
		triDef.type = b2_dynamicBody;
		tri = m_world->CreateBody(&triDef);
		tri->CreateFixture(&triFix);
		tri->SetTransform(tri->GetPosition(),60*DEGTORAD);
		
	   b2RevoluteJoint* revoluteJoint821;
          b2RevoluteJointDef revoluteJointDef821; 
	  revoluteJointDef821.bodyA = weapon;
	  revoluteJointDef821.bodyB = tri;
	  revoluteJointDef821.collideConnected = false;
	  revoluteJointDef821.localAnchorA.Set(6,0);
	  revoluteJointDef821.localAnchorB.Set(-2,0.5f);
	  revoluteJointDef821.enableLimit = false;
	  revoluteJointDef821.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef821.upperAngle = -10* DEGTORAD;
	 
	 revoluteJoint821 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef821);
	 
	 
	   
	 
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
	        m_bodyG1->SetLinearVelocity( b2Vec2(10,0) );
	      case 'e': //stop
	      cout<<m_bodyG2->GetPosition().x<<"\n";
	      //state=0;
	      for (int i=1; i<=250; i++)
	      {
	        //for(int i=0; i<=500000000; i++);
	        usleep(1000);
	        m_bodyG->SetTransform(m_bodyG->GetPosition(),-0.26*i*DEGTORAD);
	        m_bodyJ->SetTransform(m_bodyJ->GetPosition(),-0.26*i*DEGTORAD);
	      }
	      //m_bodyG->SetAngularVelocity(-10);
	      //m_bodyJ->SetAngularVelocity(-10);
			  cout<<"Entered state e";
	  if(state==0){
		  cout<<"0";
		 for(int i=1;i<250; i++)
		 {
			 usleep(1000);
	   m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-150*0.004*i)*DEGTORAD );
	   m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (150*0.004*i)*DEGTORAD );
	   m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-180*0.004*i)*DEGTORAD );
	   
	   m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (160*0.004*i)*DEGTORAD );
	    m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (0*i)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (160*0.004*i)*DEGTORAD );
	    m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (150*0.004*i)*DEGTORAD );
	    m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (0*i)*DEGTORAD );
	    }
	    m_bodyG3->SetLinearVelocity( b2Vec2(40*vel,0) );
	    m_bodyGA3->SetLinearVelocity( b2Vec2(40*vel,0) );
	    m_bodyG2->SetLinearVelocity( b2Vec2(40*vel,0) );
	    m_bodyGA2->SetLinearVelocity( b2Vec2(40*vel,0) );
	    cout<<m_bodyG2->GetPosition().x<<"\n";
	    while(m_bodyG2->GetPosition().x<12.6);
	    if(vel<=5)
	    vel++;
	    state=1;
	    break;
	}
	 if(state==1){
		 cout<<"1";
		 for(int i=1;i<250; i++)
		 {
			 usleep(1000);
	   m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (10*0.004*i)*DEGTORAD );
	   m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-10*0.004*i)*DEGTORAD );
	   m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-160*0.004*i)*DEGTORAD );
	   
	   m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (180*i*0.004)*DEGTORAD );
	    m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (-150*0.004*i)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (180*0.004*i)*DEGTORAD );
	    m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-150*0.004*i)*DEGTORAD );
	    m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (-160*i*0.004)*DEGTORAD );
	    }
	    m_bodyG3->SetLinearVelocity( b2Vec2(40*vel,0) );
	    m_bodyGA3->SetLinearVelocity( b2Vec2(40*vel,0) );
	    m_bodyG2->SetLinearVelocity( b2Vec2(40*vel,0) );
	    m_bodyGA2->SetLinearVelocity( b2Vec2(40*vel,0) );
	    state=0;
	    if(vel<=5)
	    vel++;
	    break;
	}
	if(state==2){
		cout<<"2";
		for(int i=0; i<=500000000; i++);
	  m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (30)*DEGTORAD );
	   m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-30)*DEGTORAD );
	   m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-90)*DEGTORAD );
	   
	   m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (90)*DEGTORAD );
	    m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (0)*DEGTORAD );
	    m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (10)*DEGTORAD );
	    m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-90)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-90)*DEGTORAD );
	    state=3;
	     m_bodyG1->SetLinearVelocity( b2Vec2(10,0) );
	     //usleep(100000);
	}
	if(state==3){
		cout<<"3";
	  m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (-70)*DEGTORAD );
	   m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-90)*DEGTORAD );
	   m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (0)*DEGTORAD );
	   
	   m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (60)*DEGTORAD );
	    m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (0)*DEGTORAD );
	    m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (0)*DEGTORAD );
	    m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-70)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-40)*DEGTORAD );
	    state=4;
	     m_bodyG1->SetLinearVelocity( b2Vec2(10,0) );
	    // usleep(100000);
	    for(int i=0; i<=500000000; i++);
	}
	if(state==4){
		cout<<"4";
		m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (-90)*DEGTORAD );
	   m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-90)*DEGTORAD );
	   m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-50)*DEGTORAD );
	   
	   m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (40)*DEGTORAD );
	    m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (-10)*DEGTORAD );
	    m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (10)*DEGTORAD );
	    m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-60)*DEGTORAD );
	    m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-10)*DEGTORAD );
	    state=5;
	     m_bodyG1->SetLinearVelocity( b2Vec2(10,0) );
	     //usleep(100000);
	     for(int i=0; i<=500000000; i++);
	}

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
	
