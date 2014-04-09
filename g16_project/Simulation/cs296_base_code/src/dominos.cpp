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
	  int state;//!for showing diffrent states of horse legs
		int vel;//!For velocity of m_bodyG1
		bool dummy_f;//!variable used in step function
		bool dummy1;//!variables used in step function
	    float x1;//!a variable to store distance
	        b2Body* m_bodyA;//! Cart Box
			b2Body* m_bodyB;//! Forward Tyre of cart 
			b2Body* m_bodyC;//! Back tyre of cart
			b2Body* m_bodyD;//! Seat of front man
			b2Body* m_bodyE;//! front Man's Body
			//b2Body* m_bodyE1;
			b2Body* m_bodyF;//! front Man's neck
			b2Body* m_bodyG;//! front Man's arm from shoulder to elbow
			b2Body* m_bodyH;//! Rod connected to the horse
			//b2Body* m_bodyI;//! rod2
			b2Body* m_bodyJ;//! the whip in the front man's hand
			b2Body* m_bodyG1;//! body of the horse
			b2Body* m_bodyG2;//!front right thigh of harse
			b2Body* m_bodyG3;//!calf1
			b2Body* m_bodyG4;//!thigh2
			b2Body* m_bodyG5;//!calf2
			b2Body* m_bodyGA2;//!thigh1
			b2Body* m_bodyGA3;//!calf1
			b2Body* m_bodyGA4;//!thigh2
			b2Body* m_bodyGA5;
			b2Body* m_bodyK;//! head of horse
			b2Body* m_bodyG6;//!front man's hand from elbow to palm
			b2Body* weapon;//!spear's rod part
			b2Body *tri;//! corner tringle of spear
			b2RevoluteJoint* m_joint;//!joint between Cart and forward tyre
			b2RevoluteJoint* m_joint3;//! joint between Cart and back tyre
			b2RevoluteJoint* m_joint2;//! joint between front Man and seat
			//b2RevoluteJoint* m_joint4;//!Man and head
			b2WeldJoint* m_joint5;//!joint between front Man and his arm
			b2WeldJoint* m_joint6;//! joint between Front tyre and Rod
			b2WeldJoint* m_joint7;//!joint between rod and horse body
			//b2RevoluteJoint* m_jointd;
			//b2RevoluteJoint* m_joint71;
			b2RevoluteJoint* m_joint72;//! joint connecting body to thigh
			b2RevoluteJoint* m_joint73;//!joint between front right thigh of horse to body 
			b2RevoluteJoint* m_joint74;//!joint between the right back thigh and calf 
			b2RevoluteJoint* m_joint75;//!joint between right front thigh and calf
			b2RevoluteJoint* m_joint7A2;//! joint between left back thigh and body
			b2RevoluteJoint* m_joint7A3; //!joint between left front thigh and body
			b2RevoluteJoint* m_joint7A4;//! joint between left back thigh and calf
			b2RevoluteJoint* m_joint7A5;//!joint between left front thigh and calf
			b2RevoluteJoint* m_joint8;//!joint between front man's hand and whip
			b2RevoluteJoint* m_joint9;//!joint between rod and horse body
			//b2RevoluteJoint* m_joint10;
			//b2RevoluteJoint* m_joint41;
			//b2RevoluteJoint* m_jointd1;
			b2RevoluteJoint* m_jointG6;//!elbow joint of charioter
			b2WeldJoint* m_jointnew;//! joint connecting front man to his seat  
			b2WeldJoint* m_joinseat;//! joint connecting seat to cart	 
			b2WeldJoint* joint3;//!joint between front man's neck and head	 
			b2RevoluteJoint* revoluteJoint82;//! joint between rear man's hand and spear
			b2RevoluteJointDef revoluteJointDefG6;//! JointDef for elbow joint of charioter
			b2RevoluteJointDef revoluteJointDef82; //! JoitnDef for joint between rear man's hand and spear
			//b2RevoluteJointDef revoluteJointDef10;
			b2RevoluteJointDef revoluteJointDef9;//! JointDef for joint between rod and horse body 
			b2RevoluteJointDef revoluteJointDef8;//! JointDef for joint between front man's hand and whip
			b2RevoluteJointDef revoluteJointDef75;//! JointDef for joint between right front thigh and calf
			b2RevoluteJointDef revoluteJointDef74;//! JointDef for joint between the right back thigh and calf 
			b2RevoluteJointDef revoluteJointDef73;//! JointDef for joint between front right thigh of horse to body
			b2RevoluteJointDef revoluteJointDef72;//! JointDef for joints connecting body to thigh 
			b2RevoluteJointDef revoluteJointDef7A5;//! JointDef for joint between left front thigh and calf
			b2RevoluteJointDef revoluteJointDef7A4;//! JointDef for joint between left back thigh and calf
			b2RevoluteJointDef revoluteJointDef7A3;//! JointDef for joint between left front thigh and body
			b2RevoluteJointDef revoluteJointDef7A2;//! JointDef for joint between left back thigh and body
			//b2RevoluteJointDef revoluteJointDef71;
			b2WeldJointDef revoluteJointDef7;//!JointDef for joint connecting rod to horse body
			b2WeldJointDef revoluteJointDef6;//!JointDef for joint connecting seat and rod 
			b2WeldJointDef revoluteJointDef5;//! JointDef for joint connecting man to his arm
			b2RevoluteJointDef revoluteJointDef4;//! JointDef for Joint connecting man to his neck 
			//b2RevoluteJointDef revoluteJointDef3;
			b2RevoluteJointDef revoluteJointDef2;//! JointDef for joint connecting cart to rear wheel  
			b2RevoluteJointDef revoluteJointDef;//! JointDef for joint connecting cart and front wheel
			//b2RevoluteJointDef revoluteJointDef31;
			//b2RevoluteJointDef revoluteJointDef81;
			b2WeldJointDef	weldJointDef;//! JointDef for joint connecting seat to cart	 
			b2WeldJointDef jointDef;//!JointDef for joint connecting man to his seat  
			b2Body *mobject;//!Moving Object in the simulation
			b2RevoluteJoint* fic;//!Joint used for horse tail
			b2RevoluteJoint* revoluteJoint42;//! Joint between rear man's body and his neck
			b2WeldJoint* joint22;//! joint between rear man's body and cart
			b2BodyDef mobjectDef;//!BodyDef for mobject	
			b2FixtureDef mobjectFix;//!FixtureDef for mobject
			b2RevoluteJoint* revoluteJoint52; //! joint between rear man and his hand
			b2RevoluteJoint* revoluteJoint522;//! elbow joint of rear man 
			b2RevoluteJoint* revoluteJoint0;//!joit between rear man's head and his neck
			b2RevoluteJoint* revoluteJoint821;//! joint between spear rod and tringle
			b2Body* m_bodyGA6;
	    dominos_t(){
			x1=0;
			state=-1;
			dummy_f=false;//! variables used in step function
			dummy1=false;
	  b2BodyDef bodyDef;//! Body definition for man cart etc.
	  bodyDef.type = b2_dynamicBody;//! Declaring it to be dynamic
	  b2FixtureDef fixtureDef;//! defined fixture  for objects
	  fixtureDef.density = 100;//!defined 100 desity of object
	  fixtureDef.filter.groupIndex=-1;//! To make legs collision resistant with each other
	  
	  b2BodyDef bodyDef2; //! Another body definition
	  bodyDef2.type = b2_dynamicBody;//! Body type changed to dynamicbody
	  b2FixtureDef fixtureDef2;//! another fixture definition
	  fixtureDef2.density = 5;//! defined 5 density of object 
	  	  
	  b2BodyDef bodyDef1; //! Another definition
	  bodyDef1.type = b2_dynamicBody;//! declaring body to dynamic
	  b2FixtureDef fixtureDef1;//! another fixture density
	  fixtureDef1.density = 10;//!defining density to 10
	  b2PolygonShape polygonshape;//!shape for bodyDef1 
	  b2FixtureDef fix1;//!Fixture definition
	  fix1.density=5;//! declared density to 5
	 
	 //! Body definition for ground
	  b2PolygonShape polygonShape;//!shape for ground 
	  polygonShape.SetAsBox(1, 1); //! a 2x2 rectangle
	  b2BodyDef myBodyDef;//!BodyDef for ground
	  myBodyDef.type = b2_staticBody;//!declaring ground to static
	  myBodyDef.position.Set(0, 0);//!setting center of ground at (0,0)
	  b2Body* staticBody = m_world->CreateBody(&myBodyDef);//! creating ground object
	    
	  //! fixture definition for ground
	  b2FixtureDef myFixtureDef;//!FixtureDef for Ground
	  myFixtureDef.shape = &polygonShape;//! shape of ground
	  myFixtureDef.density = 1;//!defining ground density to 1
	  myFixtureDef.friction=0.99;//! defining ground friction to 0.99
	  polygonShape.SetAsBox( 200, 13, b2Vec2(0, 0), 0);//! making  ground size to 200*13 rectangle 
	  staticBody->CreateFixture(&myFixtureDef);//!creating fixture of ground
	    
	  
		
	 
	 //! defining shapes
	  b2PolygonShape boxShape;//! The box representing cart
	  boxShape.SetAsBox(10,4);//! Dimensions of 20x8
	  
	  b2PolygonShape seat;//! The seat of man in cart
	  seat.SetAsBox(4,1.5);//! Dimensions of 8x3
	  
	  b2PolygonShape man;//! The body of man
	  man.SetAsBox(3,4);//! Dimensions of 6x8
	  
	  b2PolygonShape head;//! Neck of man
	  head.SetAsBox(0.7,1);//! diminsion of hand is 0.7*1 rectangle
	  
	  b2PolygonShape head1;//! Head of horse
	  	b2Vec2 vertices3[5];
		vertices3[0].Set(0,0);
		vertices3[1].Set(0,2.25);
		vertices3[2].Set(3,2.25);
		vertices3[3].Set(1.5,0);
		vertices3[4].Set(5,-1);
     	head1.Set(vertices3,5);
	  
	  b2PolygonShape hand;//! hand of man till elbow
	  hand.SetAsBox(2,1);//! diminsion of hand is 2*1 rectangle
 
      b2PolygonShape spear;//! Long spear in hand of man(warrior) on cart
      spear.SetAsBox(6,1);//! diminsion of spear is 6*1 rectangle
	
	  b2PolygonShape hand6;//!The arm of man from elbow to palm
	  hand6.SetAsBox(2.5,0.8);//! diminsion of hand6 is 2.5*0.8 rectangle
	   
	  b2PolygonShape Rod;//! Rod joining horse to cart
	  Rod.SetAsBox(8.3,0.1);//! diminsion of rod is 8.3*0.1 rectangle
	  
	  
	  b2PolygonShape stomach;//! The body of horse
	    b2Vec2 vertices1[6];
		vertices1[0].Set(-8,-2.5);
		vertices1[1].Set(-10,1.5);
		vertices1[2].Set(-8,2.5);
		vertices1[3].Set(6,3.5);
		vertices1[4].Set(8,2.5);
		vertices1[5].Set(7,-2.3);
		stomach.Set(vertices1,7);
	  
	  b2PolygonShape thigh1;//! calfs of horse
	  thigh1.SetAsBox(0.4,2.3);//! diminsion of thigh1 is 0.4*2.3 rectangle
	  
	  b2PolygonShape thigh2;//! Thighs of horse
	  thigh2.SetAsBox(0.6,2.4);//! diminsion of hand is 0.6*2.4 rectangle
	  
	  b2PolygonShape irene;//! whip in hand of man
	  irene.SetAsBox(6-0.5,0.1);//! diminsion of hand is 5.50.11 rectangle
	  
	  //! Wheels
	  b2CircleShape circleShape1;//! back wheel of cart 
	  b2CircleShape circleShape2;//! front wheel of cart
	  circleShape1.m_radius = 3; //! declaring back wheel radius to 3  
	  circleShape2.m_radius = 3; //! Declaring front wheel radius to 3   
	  
	  //! using m_bodyA for man
	  bodyDef.position.Set(4.5-15, 20);//! Setting position of m_bodyA to (-10.5,20)
	  fixtureDef.shape = &boxShape;//!passing shape boxshape to FixtureDef of m_bodyA
	  m_bodyA = m_world->CreateBody(&bodyDef);//! Creating m_bodyA 
	  m_bodyA->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyA
	  m_bodyA->SetAngularDamping(100);//!Declaring AngularDamping of m_bodyA to 100
	  
	  //! using m_bodyD for seat man
	  bodyDef.position.Set(14.5-15,18+1);//! Setting position of m_bodyD to (-0.5,19)
	  fixtureDef.shape=&seat;//!passing shape seat to FixtureDef of m_bodyD
	  m_bodyD=m_world->CreateBody(&bodyDef);//! Creating m_bodyD 
	  m_bodyD->CreateFixture(&fixtureDef);//!passing fixtureDef to m_bodyD
	  
	  //! using m_bodyE for man
	  bodyDef1.position.Set(18-15,22);//! Setting position of m_bodyE to (3,22)
	  fixtureDef1.shape=&man;//!passing shape man to FixtureDef of m_bodyE
	  m_bodyE=m_world->CreateBody(&bodyDef1);//! Creating m_bodyE
	  m_bodyE->CreateFixture(&fixtureDef1);//!passing fixtureDef1 to m_bodyE
	  
	  //! using m_bodyF for head of man
	  bodyDef1.position.Set(18-15,30);//! Setting position of m_bodyF to (3,30)
	  fixtureDef1.shape=&head;//!passing shape head to FixtureDef of m_bodyF
	  m_bodyF=m_world->CreateBody(&bodyDef1);//! Creating m_bodyF
	  m_bodyF->CreateFixture(&fixtureDef1);//!passing fixtureDef1 to m_bodyF
	  
	  //! using m_bodyG for man
	  bodyDef1.position.Set(26-15,23);//! Setting position of m_bodyG to (11,23)
	  fixtureDef1.shape=&hand;//!passing shape hand to FixtureDef of m_bodyG
	  m_bodyG=m_world->CreateBody(&bodyDef1);//! Creating m_bodyG
	  m_bodyG->CreateFixture(&fixtureDef1);//!passing fixtureDef1 to m_bodyG
	  m_bodyG->SetTransform(m_bodyG->GetPosition(),15*DEGTORAD);//!setting angle of front man's hand to 15 degree
	  
	  //! using m_bodyG6 for arm of man from elbow to palm
	  bodyDef1.position.Set(21+0.8-15,22.9);//! Setting position of m_bodyG6 to (6.8,22.9)
	  fixtureDef1.shape=&hand6;//!passing shape hand6 to FixtureDef of m_bodyG6
	  m_bodyG6=m_world->CreateBody(&bodyDef1);//! Creating m_bodyG6
	  m_bodyG6->CreateFixture(&fixtureDef1);//!passing fixtureDef1 to m_bodyG6
	  m_bodyG6->SetTransform(m_bodyG6->GetPosition(),5*DEGTORAD);//! setting angle of m_bodyG6 to 5 degree
	  
	  //! using m_bodyJ for whip
	  bodyDef1.position.Set(31-15,23);//! Setting position of m_bodyJ to (16,23)
	  fixtureDef1.shape=&irene;//!passing shape irene to FixtureDef of m_bodyJ
	  m_bodyJ=m_world->CreateBody( &bodyDef1 );//! Creating m_bodyJ
	  m_bodyJ->CreateFixture( &fixtureDef1 );//!passing fixtureDef1 to m_bodyJ
	  m_bodyJ->SetTransform(m_bodyG->GetPosition(),15*DEGTORAD);//! setting angle of m_bodyJ to 15 degree
	 
      //! using m_bodyB for front wheel
	  bodyDef.position.Set( 3-15, 10);//! Setting position of m_bodyC to (-12,10)
	  fixtureDef.shape = &circleShape1;//!passing shape circleShape1 to FixtureDef of m_bodyB
	  m_bodyB = m_world->CreateBody( &bodyDef );//! Creating m_bodyB
	  m_bodyB->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyB
	  m_bodyA->SetAngularDamping(10);//!Declaring AngularDamping of m_bodyB to 10
	  
	  //!using m_bodyC for rear wheel 
	  bodyDef.position.Set( 6-15, 10);//! Setting position of m_bodyC to (-9,10)
	  fixtureDef.shape = &circleShape2;//!passing shape circleShape2 to FixtureDef of m_bodyC
	  m_bodyC = m_world->CreateBody( &bodyDef );//! Creating m_bodyC
	  m_bodyC->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyC
	  m_bodyA->SetAngularDamping(10);//!Declaring AngularDamping of m_bodyC to 10
	  
	  //! using m_bodyH for rod connecting horse to cart
	  bodyDef.position.Set(0,10);//! Setting position of m_bodyH to (0,10)
	  fixtureDef.shape= &Rod;//!passing shape Rod to FixtureDef of m_bodyH
	  m_bodyH = m_world->CreateBody( &bodyDef );//! Creating m_bodyH
	  m_bodyH->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyH
	  
	  //!using m_bodyG1 for horse body
	  bodyDef.position.Set( 26+4-15, 16);//! Setting position of m_bodyG1 to (15,16)
	  fixtureDef.shape= &stomach;//!passing shape stomach to FixtureDef of m_bodyG1
	  m_bodyG1 = m_world->CreateBody( &bodyDef );//! Creating m_bodyG1
	  m_bodyG1->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyG1
	  
	  //! Tiny boxes used to define tail of horse
	  b2PolygonShape shape;
	  shape.SetAsBox(0.8f, 0.225f);//! Boxes are of dimension 0.8*0.225
	
	  //! fixture of tail of horse		
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
	  jd.localAnchorA.Set(-8,0);
	  jd.localAnchorB.Set(0,0);
	  
	  fic = (b2RevoluteJoint*)m_world->CreateJoint( &jd);
	  prevBody=body;
	  
	  //!Using several small boxes for tail of horse
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
	  
	  //! using m_bodyG2 for back thigh of horse facing us
	  bodyDef.position.Set( 24.5-15, 14.75);//! Setting position of m_bodyG1 to (9.5,14.75)
	  fixtureDef.shape= &thigh2;//!passing shape thigh2 to FixtureDef of m_bodyG2
	  m_bodyG2 = m_world->CreateBody( &bodyDef );//! Creating m_bodyG2
	  m_bodyG2->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyG2
	  
	  //! using m_bodyGA2 for back thigh of horse facing screen
	  bodyDef.position.Set( 24.5-15+1, 14.75);//! Setting position of m_bodyGA2 to (10.5,14.75)
	  fixtureDef.shape= &thigh2;//!passing shape thigh2 to FixtureDef of m_bodyGA2
	  m_bodyGA2 = m_world->CreateBody( &bodyDef );//! Creating m_bodyGA1
	  m_bodyGA2->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyGA2
	  
	  //! using m_bodyG3 for front thigh of horse facing us
	  bodyDef.position.Set( 27.5+8-15, 14.75);//! Setting position of m_bodyG3 to (20.5,14.75)
	  fixtureDef.shape= &thigh2;//!passing shape thigh2 to FixtureDef of m_bodyG3
	  m_bodyG3 = m_world->CreateBody( &bodyDef );//! Creating m_bodyG3
	  m_bodyG3->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyG3
	  
	  //! using m_bodyGA3 for front thigh of horse facing screen
	  bodyDef.position.Set( 27.5+8-15-1, 14.75);//! Setting position of m_bodyGA3 to (19.5,14.75)
	  fixtureDef.shape= &thigh2;//!passing shape thigh2 to FixtureDef of m_bodyGA3
	  m_bodyGA3 = m_world->CreateBody( &bodyDef );//! Creating m_bodyGA3
	  m_bodyGA3->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyGA3
	  
	  //! using m_bodyG4 for back calf  of horse facing us
	  bodyDef.position.Set( 24.5-15, 10.25);//! Setting position of m_bodyG4 to (9.5,10.25)
	  fixtureDef.shape= &thigh1;//!passing shape thigh1 to FixtureDef of m_bodyG4
	  m_bodyG4 = m_world->CreateBody( &bodyDef);//! Creating m_bodyG4
	  m_bodyG4->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyG4
	  
	  //! using m_bodyGA4 for back calf of horse facing screen
	  bodyDef.position.Set( 24.5-15+1, 10.25);//! Setting position of m_bodyGA4 to (10.5,10.25)
	  fixtureDef.shape= &thigh1;//!passing shape thigh1 to FixtureDef of m_bodyGA4
	  m_bodyGA4 = m_world->CreateBody( &bodyDef);//! Creating m_bodyGA4
	  m_bodyGA4->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyGA4
	  
	  //! using m_bodyG5 for front calf of horse facing us
	  bodyDef.position.Set( 27.5+8-15, 10.25);//! Setting position of m_bodyG5 to (20.5,10.25)
	  fixtureDef.shape= &thigh1;//!passing thigh1 stomach to FixtureDef of m_bodyG5
	  m_bodyG5 = m_world->CreateBody( &bodyDef );//! Creating m_bodyG5
	  m_bodyG5->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyG5
	  
	  //! using m_bodyGA5 for for front calf of horse
	  bodyDef.position.Set( 27.5+8-15-1, 10.25);//! Setting position of m_bodyGA5 to (19.5,16)
	  fixtureDef.shape= &thigh1;//!passing shape thigh1 to FixtureDef of m_bodyGA5
	  m_bodyGA5 = m_world->CreateBody( &bodyDef );//! Creating m_bodyGA5
	  m_bodyGA5->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyGA5
	  
	  //! using  m_bodyK for head of horse
	  bodyDef.position.Set( 30-15+8, 19);//! Setting position of m_bodyK to (23,19)
	  fixtureDef.shape= &head1;//!passing shape stomach to FixtureDef of m_bodyK
	  m_bodyK = m_world->CreateBody( &bodyDef );//! Creating m_bodyK
	  m_bodyK->CreateFixture( &fixtureDef );//!passing fixtureDef to m_bodyK
	  m_bodyK->SetTransform(m_bodyK->GetPosition(),60*DEGTORAD);//! setting angle of m_bodyJ to 60 degree
	  m_bodyK->SetAngularDamping(10);//!Declaring Angular Damping of m_bodyK to 10
	
	
	  	//!JOINTS
	  //! joint connecting cart and front wheel
	  revoluteJointDef.bodyA = m_bodyA;//!Body A of this joint is m_bodyA
	  revoluteJointDef.bodyB = m_bodyB;//!Body B of this joint is m_bodyB
	  revoluteJointDef.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef.localAnchorA.Set(10,-4);//!localAnchor of Body A is (10,-4)
	  revoluteJointDef.localAnchorB.Set(0,0);//!localAnchor of Body B is (0,0) 
	  m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );//! Creating the Joint m_joint
	 
	  //! joint connecting man to his seat  
      jointDef.bodyA = m_bodyD;//!Body A of this joint is m_bodyD
      jointDef.bodyB = m_bodyE;//!Body B of this joint is m_bodyE
      jointDef.collideConnected = false;//!set collideconnected to false
      jointDef.localAnchorA.Set(0,5.5);//!localAnchor of Body A is (0,5.5)
	  jointDef.localAnchorB.Set(0,0);   //!localAnchor of Body B is (0,0)
      m_jointnew=(b2WeldJoint*)m_world->CreateJoint( &jointDef );//! Creating the Joint m_jointnew
          
      //! joint connecting cart to rear wheel   
	  revoluteJointDef2.bodyA = m_bodyA;//!Body A of this joint is m_bodyA
	  revoluteJointDef2.bodyB = m_bodyC;//!Body B of this joint is m_bodyC
	  revoluteJointDef2.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef2.localAnchorA.Set(-10,-4);//!localAnchor of Body A is (10,-4)
	  revoluteJointDef2.localAnchorB.Set(0,0);//!localAnchor of Body B is (0,0) 
	  m_joint3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef2 );//! Creating the Joint m_joint3
	  
	  //! joint connecting seat to cart	 
	  weldJointDef.bodyA = m_bodyA;//!Body A of this joint is m_bodyA
	  weldJointDef.bodyB = m_bodyD;//!Body B of this joint is m_bodyD
	  weldJointDef.collideConnected = false;//!set collideconnected to false
	  weldJointDef.localAnchorA.Set(10,2);//!localAnchor of Body A is (10,2)
	  weldJointDef.localAnchorB.Set(-4,0);//!localAnchor of Body B is (-4,0)
	  m_joinseat = (b2WeldJoint*)m_world->CreateJoint( &weldJointDef ); //! Creating the Joint m_joinseat
	   
	  //!Joint connecting man to his neck 
	  revoluteJointDef4.bodyA = m_bodyE;//!Body A of this joint is m_bodyE
	  revoluteJointDef4.bodyB = m_bodyF;//!Body B of this joint is m_bodyF
	  revoluteJointDef4.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef4.localAnchorA.Set(0,5);//!localAnchor of Body A is (0,5)
	  revoluteJointDef4.localAnchorB.Set(0,0);//!localAnchor of Body B is (0,0)
	  revoluteJointDef4.enableLimit = false;//!set enablelimit to false
	  m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef4);//! Creating the Joint m_joint2
	   
	  //!joint connecting man to his arm
	  revoluteJointDef5.bodyA = m_bodyE;//!Body A of this joint is m_bodyE
	  revoluteJointDef5.bodyB = m_bodyG;//!Body B of this joint is m_bodyG
	  revoluteJointDef5.collideConnected = true;//!set collideconnected to true
	  revoluteJointDef5.localAnchorA.Set(4,0);//!localAnchor of Body A is (4,0)
	  revoluteJointDef5.localAnchorB.Set(0,0);//!localAnchor of Body B is (0,0)
	  m_joint5 = (b2WeldJoint*)m_world->CreateJoint( &revoluteJointDef5);//! Creating the Joint m_joint5
	  
	  //! joint connecting seat and rod 
	  revoluteJointDef6.bodyA = m_bodyD;//!Body A of this joint is m_bodyD
	  revoluteJointDef6.bodyB = m_bodyH;//!Body B of this joint is m_bodyH
	  revoluteJointDef6.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef6.localAnchorA.Set(0,0);//!localAnchor of Body A is (0,0)
	  revoluteJointDef6.localAnchorB.Set(-8.3,0.0);//!localAnchor of Body B is (-8.3,0)
	  m_joint6 = (b2WeldJoint*)m_world->CreateJoint( &revoluteJointDef6);//! Creating the Joint m_joint6
	   
	  //! joint connecting rod to horse body
	  revoluteJointDef7.bodyA = m_bodyG1;//!Body A of this joint is m_bodyG1
	  revoluteJointDef7.bodyB = m_bodyH;//!Body B of this joint is m_bodyH
	  revoluteJointDef7.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef7.localAnchorA.Set(0,0);//!localAnchor of Body A is (0,0)
	  revoluteJointDef7.localAnchorB.Set(8.3,0.0);//!localAnchor of Body B is (8.3,0)
	  m_joint7 = (b2WeldJoint*)m_world->CreateJoint( &revoluteJointDef7);//! Creating the Joint m_joint7
	   
	  //! joints connecting body to thigh 
	  revoluteJointDef72.bodyA = m_bodyG1;//!Body A of this joint is m_bodyG1
	  revoluteJointDef72.bodyB = m_bodyG2;//!Body B of this joint is m_bodyG2
	  revoluteJointDef72.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef72.localAnchorA.Set(-6,0);//!localAnchor of Body A is (-6,0)
	  revoluteJointDef72.localAnchorB.Set(0,2.25);//!localAnchor of Body B is (0,2.25)
	  revoluteJointDef72.enableLimit = true;//!Set enablelimit to true
	  revoluteJointDef72.lowerAngle = 5* DEGTORAD;//!declared lowerAngle of joint to 5 degree
	  revoluteJointDef72.upperAngle = -5* DEGTORAD;//!declared upperAngle of joint to -5 degree
	  m_joint72 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef72);//! Creating the Joint m_joint72
	  
      //!joint between front right thigh of horse to body 
	  revoluteJointDef73.bodyA = m_bodyG1;//!Body A of this joint is m_bodyA
	  revoluteJointDef73.bodyB = m_bodyG3;//!Body B of this joint is m_bodyB
	  revoluteJointDef73.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef73.localAnchorA.Set(6,0);//!localAnchor of Body A is (6,0)
	  revoluteJointDef73.localAnchorB.Set(0,2.25);//!localAnchor of Body B is (0,2.25)
	  revoluteJointDef73.enableLimit = true;//!set enable limit to true
	  revoluteJointDef73.lowerAngle = 5* DEGTORAD;//!declared lowerAngle of joint to 5 degree
	  revoluteJointDef73.upperAngle = -5* DEGTORAD;//!declared upperAngle of joint to -5 degree
	  m_joint73 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef73);//! Creating the Joint m_joint73
	   
	  //!joint between the right back thigh and calf 
	  revoluteJointDef74.bodyA = m_bodyG2;//!Body A of this joint is m_bodyA
	  revoluteJointDef74.bodyB = m_bodyG4;//!Body B of this joint is m_bodyB
	  revoluteJointDef74.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef74.localAnchorA.Set(0,-2.25);//!localAnchor of Body A is (0,-2.25)
	  revoluteJointDef74.localAnchorB.Set(0,2.25);//!localAnchor of Body B is (0,2.25)
	  revoluteJointDef74.enableLimit = true;//!set enable limit to true
	  revoluteJointDef74.lowerAngle = 5* DEGTORAD;//!declared lowerAngle of joint to 5 degree
	  revoluteJointDef74.upperAngle = -5* DEGTORAD;//!declared upparAngle of joint to -5 degree
	  m_joint74 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef74);//! Creating the Joint m_joint74
	   
	  //!joint between right front thigh and calf
	  revoluteJointDef75.bodyA = m_bodyG3;//!Body A of this joint is m_bodyA
	  revoluteJointDef75.bodyB = m_bodyG5;//!Body B of this joint is m_bodyB
	  revoluteJointDef75.collideConnected = false;//!set collideconnected to false
	  revoluteJointDef75.localAnchorA.Set(0,-2.25);//!localAnchor of Body A is (0,-2.25)
	  revoluteJointDef75.localAnchorB.Set(0,2.25);//!localAnchor of Body B is (0,2.25)
	  revoluteJointDef75.enableLimit = true;//!set enable limit to true
	  revoluteJointDef75.lowerAngle = 10* DEGTORAD;//!declared lowerAngle of joint to 5 degree
	  revoluteJointDef75.upperAngle = -10* DEGTORAD;//!declared upparAngle of joint to -5 degree
	  m_joint75 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef75);//! Creating the Joint m_joint75
	   
	  //! joint between left back thigh and body
	  revoluteJointDef7A2.bodyA = m_bodyG1;//! declare horse body to be bodyA
	  revoluteJointDef7A2.bodyB = m_bodyGA2;//! declare backward thigh facing screen to be bodyB
	  revoluteJointDef7A2.collideConnected = false;//! set collide connected to false
	  revoluteJointDef7A2.localAnchorA.Set(-4,0);//! set local anchor A at (-4.5,0) wrt its center
	  revoluteJointDef7A2.localAnchorB.Set(0,2.25);//! set local anchorB at (0,2.25) wrt its center
	  revoluteJointDef7A2.enableLimit = true;//! enable limit of joints
	  revoluteJointDef7A2.lowerAngle = 10* DEGTORAD;//! set lower angle at 10 degrees
	  revoluteJointDef7A2.upperAngle = -10* DEGTORAD;//!set upper angle at 10 degrees
	  m_joint7A2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A2);//! create a revolute joint
	   
	  //!joint between left front thigh and body
	  revoluteJointDef7A3.bodyA = m_bodyG1;//! declare horse body to be bodyA
	  revoluteJointDef7A3.bodyB = m_bodyGA3;//! declare forward thigh facing screen to be bodyB
	  revoluteJointDef7A3.collideConnected = false;//! set collide connected to false
	  revoluteJointDef7A3.localAnchorA.Set(4,0);//! set local anchor A at (4.5,0) wrt its center
	  revoluteJointDef7A3.localAnchorB.Set(0,2.25);//! set local anchor B at (0,2.25) wrt its center
	  revoluteJointDef7A3.enableLimit = true;//! set enable limit to true
	  revoluteJointDef7A3.lowerAngle = 10* DEGTORAD;//! set lower angle at 10 degrees
	  revoluteJointDef7A3.upperAngle = -10* DEGTORAD;//! set upper angle at 10 degrees
	  m_joint7A3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A3);//! creates joint
	   
	  //! joint between left back thigh and calf
	  revoluteJointDef7A4.bodyA = m_bodyGA2;//! declare backward thigh facing screen  to be bodyA
	  revoluteJointDef7A4.bodyB = m_bodyGA4;//! declare backward calf facing screen  to be bodyB
	  revoluteJointDef7A4.collideConnected = false;//! set collide connected to false
	  revoluteJointDef7A4.localAnchorA.Set(0,-2.25);//!set local anchor A at (0,-2.25) wrt its center
	  revoluteJointDef7A4.localAnchorB.Set(0,2.25);//! set local anchor B at (0,2.25) wrt its center
	  revoluteJointDef7A4.enableLimit = true;//! set enable limit to true
	  revoluteJointDef7A4.lowerAngle = 10* DEGTORAD;//! set lower angle to 10 degrees
	  revoluteJointDef7A4.upperAngle = -10* DEGTORAD;//! set upper angle to 10 degrees
	  m_joint7A4 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A4);//! creates a revolute joint
	   
	  //!joint between left front thigh and calf
	  revoluteJointDef7A5.bodyA = m_bodyGA3;//! declare forward thigh facing screen  to be bodyA
	  revoluteJointDef7A5.bodyB = m_bodyGA5;//! declare forward calf facing screen  to be bodyB
	  revoluteJointDef7A5.collideConnected = false;//! set collide connected to false
	  revoluteJointDef7A5.localAnchorA.Set(0,-2.25);//!set local anchor A at (0,-2.25) wrt its center
	  revoluteJointDef7A5.localAnchorB.Set(0,2.25);//! set local anchor B at (0,2.25) wrt its center
	  revoluteJointDef7A5.enableLimit = true;//! set enable limit to true
	  revoluteJointDef7A5.lowerAngle = 10* DEGTORAD;//! set lower angle to 10 degrees
	  revoluteJointDef7A5.upperAngle = -10* DEGTORAD;//! set upper angle to 10 degrees
	  m_joint7A5 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A5);  //! creates a revolute joint
	  
	  //! joint between man's hand and his whip
	  revoluteJointDef8.bodyA = m_bodyG6;//! set hand to be bodyA
	  revoluteJointDef8.bodyB = m_bodyJ;//!set whip to be body B
	  revoluteJointDef8.collideConnected = false;//! set collide connected to false
	  revoluteJointDef8.localAnchorA.Set(2,0);//! set local anchorA to (2,0)
	  revoluteJointDef8.localAnchorB.Set(-6+0.5,0);//! set local anchorB to(-5.5,0)
	  revoluteJointDef8.enableLimit = true;//! set enable limit to true
	  m_joint8 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef8);//! create a revolute joint
	   
	 //!elbow joint of charioter  
	  revoluteJointDefG6.bodyA = m_bodyG6;//!Body A of this joint is m_bodyG6
	  revoluteJointDefG6.bodyB = m_bodyG;//!Body B of this joint is m_bodyG
	  revoluteJointDefG6.collideConnected = false;//!set collideconnected to false
	  revoluteJointDefG6.localAnchorA.Set(-2.2,0);//!localAnchor of Body A is (-2.2,0)
	  revoluteJointDefG6.localAnchorB.Set(2.3,0);//!localAnchor of Body B is (2.3,0)
	  revoluteJointDefG6.enableLimit = true;//!set enablelimit to true
	  revoluteJointDefG6.referenceAngle = -25 * DEGTORAD;//!declared reference angle to 5 degree
	  m_jointG6 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDefG6);//! Creating the Joint m_jointG6
	  
	  //!joint between rod and horse body 
	  revoluteJointDef9.bodyA = m_bodyG1;//!Body A of this joint is m_bodyG1
	  revoluteJointDef9.bodyB = m_bodyK;//!Body B of this joint is m_bodyK
	  revoluteJointDef9.collideConnected = true;//!set collideconnected to true
	  revoluteJointDef9.localAnchorA.Set(7,3);//!localAnchor of Body A is (7,3)
	  revoluteJointDef9.localAnchorB.Set(0,1);//!localAnchor of Body B is (0,1)
	  revoluteJointDef9.enableLimit = true;//!set enablelimit to true
	  revoluteJointDef9.enableMotor = true;//!set enableMotor to true
	  revoluteJointDef9.lowerAngle = 5* DEGTORAD;//!declared lowerAngle of joint to 5 degree
	  revoluteJointDef9.upperAngle = -5* DEGTORAD;//!declared upperAngle of joint to 5 degree
	  m_joint9 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef9);//! Creating the Joint m_joint9
	  
	  state=0;
	  vel=0;
	   
	   //! rear man's body 
	   b2Body *man1;//!Body of rear man
	   b2BodyDef manDef1;//! BodyDef of man1
	   b2FixtureDef manFix1;//!FixtureDef for man1
	   b2PolygonShape manPoly1;;//!Defined manPloy for rear man's Body
	   manPoly1.SetAsBox(3,4);//!declared dimension of manPoly to 3*4
	   manFix1.shape = &manPoly1;//!passing shape manPoly to FixtureDef of man1
	   manDef1.position.Set(-12, 29);//!declared position of man 1 to (-12,19)
	   manDef1.type = b2_dynamicBody;//!declared type of body to dynamic 
	   man1 = m_world->CreateBody(&manDef1);//!creating body man1
	   man1->CreateFixture(&manFix1);//!passing manFix1 to man1
	  
	  //! rear man's neck
	   b2Body *manh;//! b2Body* of man's neck
	   b2BodyDef manhDef2;//! b2Body of man's neck
	   b2FixtureDef manhFix2;//! fixxture of man's neck
	   b2PolygonShape manhPoly2;//! The body of the man neck
	   manhPoly2.SetAsBox(0.7,1);//! Dimensions of man neck
	   manhFix2.shape = &manhPoly2;//! defining shae for bodyDef of man
	   manhFix2.restitution = 0;//! defining restitution of man to be zero
	   manhDef2.position.Set(-12, 34);//! Setting initial position of man at (-12,34)
	   manhDef2.type = b2_dynamicBody;//! declaring man to be a dynamic body
	   manh = m_world->CreateBody(&manhDef2);//! creating bodyDef of manh in m_world
	   manh->CreateFixture(&manhFix2);//! Creating manh's fixture
		
	   //! joint between rear man's body and his eck
       b2RevoluteJointDef revoluteJointDef42;//! joint between man's head and body
 	   revoluteJointDef42.bodyA = man1;//! setting man1 to be bodyA
	   revoluteJointDef42.bodyB = manh;//! setting manh to be bodyB
	   revoluteJointDef42.collideConnected = false;//! setting collide connected to false
	   revoluteJointDef42.localAnchorA.Set(0,4);//! setting local anchor of A wrt its center
	   revoluteJointDef42.localAnchorB.Set(0,-1);//! setting local anchor of B wrt its center
	   revoluteJointDef42.enableLimit = false;  //!disabling limit of joint rotation
	   revoluteJointDef42.enableMotor = false; //! disabling motor
	   revoluteJoint42 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef42);//! creating a revolute joint in m_world
        
        //! joint between rear man's body and cart
       b2WeldJointDef jointDef22;//! joint between man and cart
       jointDef22.bodyA = man1;//! setting man1 to be bodyA
       jointDef22.bodyB = m_bodyA;//! setting cart to be bodyB
       jointDef22.collideConnected = false;//! setting collide connected to true
       jointDef22.localAnchorA.Set(0,-4);//! setting local anchorA at (0,-4) wrt its center
	   jointDef22.localAnchorB.Set(0,4);//! setting local anchorB at (0,4) wrt its center
       joint22=(b2WeldJoint*)m_world->CreateJoint( &jointDef22 );   //! creating weld joint in m_world
      
      //! rear man's hand from elbow to palm
       b2Body* manhand;//! declaring b2Body* for elbow to palm of man
	   b2BodyDef manhandDef;//! declaring bodeDef for this part
	   b2FixtureDef manhandFix;//! declaring fixtureDef for manhand
       b2PolygonShape manhandPoly;//! declaring shape of manhand
	   manhandPoly.SetAsBox(2,1);//! giving it a dimenxion of 4x2
	   manhandFix.shape=&manhandPoly;//! giving shape as that of declared above
	   manhandDef.type = b2_dynamicBody;//! declaring hand to be a dynamic body
	   manhandDef.position.Set(-9,28);//! Setting its initial position at (-9,28)
	   manhand=m_world->CreateBody(&manhandDef);//! creating manhand  in m_world
	   manhand->CreateFixture(&manhandFix);//! creating fixture in m_world
	   manhand->SetTransform(manhand->GetPosition(),30*DEGTORAD);//! giving it an initial orientation of 30degrees
	  
	   //!rear man's hand till elbow
       b2BodyDef m_bodyGA6Def;//! declaring b2BodyDef for hand from shoulder to elbow
       b2FixtureDef m_bodyGA6Fix;//! declaring fixture for that part of hand
       b2PolygonShape m_bodyGA6Poly;//! declaring its shape to be a polygon
       m_bodyGA6Poly.SetAsBox(2.5,0.8);//! giving it a dimension of 5x1.6
       m_bodyGA6Fix.shape=&m_bodyGA6Poly;//! declaring shape of fixtureDef to be that of polygon
       m_bodyGA6Def.type = b2_dynamicBody;//! declaring it to be of type dynamicbody
       m_bodyGA6Def.position.Set(-30,28);//! setting its initial position at (-30,28)
       m_bodyGA6=m_world->CreateBody(&m_bodyGA6Def);//! creating body in world
       m_bodyGA6->CreateFixture(&m_bodyGA6Fix);//! setting its fixture in world
       m_bodyGA6->SetTransform(m_bodyGA6->GetPosition(),110*DEGTORAD);//! giving it an initial orietation of 110degrees
	   
	  //! joint between rear man and his hand
      b2RevoluteJointDef revoluteJointDef52;//! declaring a joint between man's body and hand
	  revoluteJointDef52.bodyA = man1;//! declaring man to be bodyA
	  revoluteJointDef52.bodyB = manhand;//! declaring hand to be bodyB
	  revoluteJointDef52.collideConnected = false;//! setting collideconnect to false
	  revoluteJointDef52.localAnchorA.Set(4,0);//! setting localanchor of body A at (4,0) wrt its center
	  revoluteJointDef52.localAnchorB.Set(0,0);//!setting local anchor ofbodyB at (0,0) wrt its center
	  revoluteJointDef52.enableLimit = true;//! enabling limit of rotation wrt joint
	  revoluteJointDef52.lowerAngle = 10* DEGTORAD;//! setting lower limit of 10degrees
	  revoluteJointDef52.upperAngle = -10* DEGTORAD;  //! setting upper limit of 10degrees
	  revoluteJoint52 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef52);//! creating a revolute joint in m_world
	  
	  //! elbow joint of rear man  
      b2RevoluteJointDef revoluteJointDef522; //! declaring a joint between at man's elbow
      revoluteJointDef522.bodyA = manhand;//! declatring sholder part to be bodyA
      revoluteJointDef522.bodyB = m_bodyGA6;//! declaring palm part to be bodyB
      revoluteJointDef522.collideConnected = false;//!setting collide connected to false
      revoluteJointDef522.localAnchorA.Set(2,0);//! setting localanchorA at (2,0) wrt its center
      revoluteJointDef522.localAnchorB.Set(-2.5,0);//!setting localanchorA at (-2.5,0) wrt its center
      revoluteJointDef522.enableLimit = false;//! setting enableLimit to false
      revoluteJoint522 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef522); //! creating revolute joint
	 
	 //! spear's rod
	  b2BodyDef weaponDef;//! declaring bodyDef for spear
	  b2FixtureDef weaponFix;     //! declaring fixtureDef for spear
      b2PolygonShape weaponPoly; //! declaring shape of weapon to be polygon
	  weaponPoly.SetAsBox(4,0.1f);//! giving it dimensions of 8x0.2
	  weaponFix.shape=&weaponPoly;//! giving it shape of polygon described above
	  weaponFix.restitution=0;//! giving it a restitution zero
	  weaponDef.type = b2_dynamicBody;//! setting it to be a dynamic body
	  weaponDef.position.Set(-2.5f,32);//! setting its initial position at (-2.5,32)
	  weapon=m_world->CreateBody(&weaponDef);//! creating weapon
      weapon->CreateFixture(&weaponFix);//! creating its fixture
	  weapon->SetTransform(weapon->GetPosition(),55*DEGTORAD);//! giving it an initial orientation of 55degrees
	  weapon->SetAngularDamping(1);//! setting an angular damping of 1
	  
	  //! joint between rear man's hand and spear
      revoluteJointDef82.bodyA = m_bodyGA6;//! declaring body A to be hand
      revoluteJointDef82.bodyB = weapon;//! declaring bodyB to be spear
      revoluteJointDef82.collideConnected = false;//! setting collide connected to false
      revoluteJointDef82.localAnchorA.Set(2,0);//! setting local anchor of A at (2,) wrt its center
      revoluteJointDef82.localAnchorB.Set(-3,0);//! setting local anchor of B (-3,0) wrt its center
      revoluteJointDef82.enableLimit = true;//! enabling limit to true
      revoluteJointDef82.lowerAngle = 10* DEGTORAD;//! setting lower angle of 10degrees
      revoluteJointDef82.upperAngle = -10* DEGTORAD; //! setting upper angle of 10degrees
      revoluteJoint82 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef82); //! creating a revolute joint
	 
	  //! front man's head
	  b2Body *manhead;//! creating a b2bodyDef* for man's head
	  b2BodyDef manheadDef;//! creating b2bodyDef
	  b2FixtureDef manheadFix;//! creating fixture for manhead
	  b2PolygonShape manheadPoly;//! creating shape for head
	  manheadPoly.SetAsBox(2,2);//! giving it a dimension of4x4
	  manheadFix.shape = &manheadPoly;//! giving it shape
	  manheadFix.restitution = 0;//! setting restitution to zero
	  manheadDef.position.Set(-12, 34);//! giving its initial position to (-12,34)
	  manheadDef.type = b2_dynamicBody;//! setting it as a dynamic body
	  manhead = m_world->CreateBody(&manheadDef);//! creating body
	  manhead->CreateFixture(&manheadFix);//! creating fixture
		
		
	  //!joint between front man's neck and head	 
      b2WeldJointDef jointDef3;//! declaring joint between neack and head
      jointDef3.bodyA = m_bodyF;//! declaring neck to be bodyA
      jointDef3.bodyB = manhead;//! declaring head to be bodyB
      jointDef3.collideConnected = true;//! setting collide connected to true
      jointDef3.localAnchorA.Set(0,1);//! setting localanchor A at (0,1) wrt its center
	  jointDef3.localAnchorB.Set(0,-2);//! setting local anchor B at (0,-2) wrt its center
	  joint3= (b2WeldJoint*)m_world->CreateJoint( &jointDef3 ); //! declaring weld joint
	       
	       
	   //! rear man's head
	  b2Body *manh1;//! declaring man's head b2body*
	  b2BodyDef manh1Def;//! declaring b2body for manhead
	  b2FixtureDef manh1Fix;//! declaring head's fixture
      b2PolygonShape manh1Poly;//! declaring head's shape
	  manh1Poly.SetAsBox(2,2);//! giving it dimensions of 4x4
	  manh1Fix.shape = &manh1Poly;//! setting shape to polygon described above
	  manh1Def.position.Set(-12,35) ;//! set its initial position at (-12,35)
	  manh1Def.type = b2_dynamicBody;//!setting it as adynamic body
	  manh1 = m_world->CreateBody(&manh1Def);//!creating body
	  manh1->CreateFixture(&manh1Fix);//!creating fixture
		
	 //!joint between rear man's head and his neck
      b2RevoluteJointDef revoluteJointDef0; //! declaring joint
	  revoluteJointDef0.bodyA = manh;//!setting head as bodyA
	  revoluteJointDef0.bodyB = manh1;//!setting neck as bodyB
	  revoluteJointDef0.collideConnected = false;//!setting collide connected to false
	  revoluteJointDef0.localAnchorA.Set(0,1);//!setting localAnchor A at (0,1) wrt its center
	  revoluteJointDef0.localAnchorB.Set(0,-2);//!settign localanchor of B at (0,-2) wrt its center
	  revoluteJointDef0.enableLimit = false;//!setting limit to false
	  revoluteJoint0 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef0);//!declaring revolute joint
		 
	 //! spear's corner tringle
	  b2BodyDef triDef;//! declaring bodyDef
	  b2FixtureDef triFix;//! declaring its fixture
	  b2PolygonShape triPoly;//! setting shape
	  b2Vec2 vertices2[3];//! giving an array of 3 points
	  vertices2[0].Set(-2,1.0f);//! setting first vertex at (-2,1)
	  vertices2[1].Set(0,0.5f);//! setting vertex 2 at (0,0.5)
	  vertices2[2].Set(-2,0);//! setting vertex3 at (-2,0)
	  triPoly.Set(vertices2, 3);//! declaring body shape to be that of triangle formed above
	  triFix.restitution=0;//! setting restitution to be zero
	  triFix.shape = &triPoly;//! setting shape to be triangle
	  triDef.position.Set(2,38.2f);//! setting its initial position at (2,38,2)
	  triDef.type = b2_dynamicBody;//! declaring object to be b2_dynamicBody
	  tri = m_world->CreateBody(&triDef);//! create body
	  tri->CreateFixture(&triFix);//! create fixture
	  tri->SetTransform(tri->GetPosition(),55*DEGTORAD);//! give it an initial rotation of 55degrees
	  tri->SetAngularDamping(1);//! set angular damping to be 1
	
	  //! joint between spear rod and tringle	
	 
      b2RevoluteJointDef revoluteJointDef821; //! declaring joint
	  revoluteJointDef821.bodyA = weapon;//! setting weapon as bodyA
	  revoluteJointDef821.bodyB = tri;//! setting triangle as bodyB
	  revoluteJointDef821.collideConnected = false;//! setting collideconnected to false
	  revoluteJointDef821.localAnchorA.Set(4,0);//! setting local anchor of A wrt its center at (4,0)
	  revoluteJointDef821.localAnchorB.Set(-2,0.5f);//! setting local anchor of B wrt its center at (-2,0.5)
	  revoluteJointDef821.enableLimit = false;//! setting limit to false	 
	  revoluteJoint821 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef821);	//! revolute joint declared
	  
	  //! moving box in simulation
	  float distance;//! declaring distance
	  distance=weapon->GetLinearVelocity().x;//! getting distance from velocity
	  b2PolygonShape mobjectPoly;//! setting object shape
	  mobjectPoly.SetAsBox(1.5,5);	//! giving it dimensions of 3x10
	  mobjectFix.shape = &mobjectPoly;//! setting its shape
	  mobjectFix.friction = 0;	//! setting friction to be zero
	  mobjectFix.restitution = 0;//! setting restitution to be zero
	  mobjectDef.position.Set(distance+40,2) ;//! setting its new position
	  mobjectDef.type = b2_dynamicBody;//! declaring it of type dynamic body
	  mobject = m_world->CreateBody(&mobjectDef);//! creating body
	  mobject->CreateFixture(&mobjectFix);//! creating fixture
	  mobject->SetLinearVelocity( b2Vec2(10,0) );//! giving it initial velocity of 10
}
		//! Keyboard function inherited and overriden from base_sim_t class
		void keyboard(unsigned char key)
	    { 
	    switch (key)
	    {
	      case 'e': //! man hits horse which compels it to move forward
	      dummy1=true;//! some variable used in step function
	      state=1;//! some variable used in step function
	      x1=m_bodyG1->GetPosition().x;//! setting x1 to G1's position
	      m_bodyK->SetTransform(m_bodyK->GetPosition(),-40*DEGTORAD);//! rotating head of horse
	      for(int i=1; i<=5; i++)
	      {
			  state=1;
		  }
	      vel++;//! increasing velocity after repeated beating
	      for (int i=1; i<=250; i++)
	          {
	            m_bodyG6->SetTransform(m_bodyG->GetPosition(),-0.16*i*DEGTORAD);//! arm of man moving step by step to make movement smooth
	            m_bodyJ->SetTransform(m_bodyJ->GetPosition(),-0.26*i*DEGTORAD);//! moving whip same as above
	          }
	          if(m_bodyG1->GetLinearVelocity().x>0)
	          {
	           m_bodyG1->SetLinearVelocity(b2Vec2(m_bodyG1->GetLinearVelocity().x+20,0));//! increasing its velocity slightly
			  }
			  else
			  {
				  m_bodyG1->SetLinearVelocity(b2Vec2(40,0));
			  }
	        break;		 
	      case 'j': //! throws the spear
				{
				m_world->DestroyJoint(revoluteJoint82);//! destroys joint between man and weapon
				weapon->SetAngularVelocity(-1.5);//! gives weapon velocity
				tri->SetAngularVelocity(-1.5);//! gives weapon velocity
				weapon->SetLinearVelocity( b2Vec2(40+m_bodyG1->GetLinearVelocity().x,25) );//!gets enough velocity to hit spear
				m_bodyGA6->SetTransform(m_bodyGA6->GetPosition(),40*DEGTORAD);//! hand of man moves to throw spear
		        break;
	           }
	      default:
	      //! run default behaviour
	      cs296::base_sim_t::keyboard(key);
	    } 
	  } 
	  //! step function inherited and overriden from base_sim_t class
      void step(cs296::settings_t* settings){//! we implement state machine of horse here
	  cs296::base_sim_t::step(settings);//! running the default step function
	  if(dummy1&&((int)(m_bodyG1->GetPosition().x-x1)/2%4==0)){//!when distance of body/4 is odd
		   revoluteJointDef73.lowerAngle=90*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=90*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef75.lowerAngle=90*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=90*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=-90*DEGTORAD;
	       if(m_bodyG1->GetLinearVelocity().x<50)
	       {//! goes to one of the states as given inour diagram
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (60)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-30)*DEGTORAD ); 
	       m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (0)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-30)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (0)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (60)*DEGTORAD );
			}
			else
			{//! goes to one of the states as given inour diagram
				m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-40)*DEGTORAD );
				m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (80)*DEGTORAD );
				m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-70)*DEGTORAD ); 
				m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (-10)*DEGTORAD );
				m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (-10)*DEGTORAD );
				m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-70)*DEGTORAD );
				m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-40)*DEGTORAD );
				m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (80)*DEGTORAD );
			}
	  }
	  if(dummy1&&((int)(m_bodyG1->GetPosition().x-x1)/2%4==1)){//!when distance of body/4 is odd
		   revoluteJointDef73.lowerAngle=90*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=90*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef75.lowerAngle=90*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=90*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=-90*DEGTORAD;
	       if(m_bodyG1->GetLinearVelocity().x<50)
	       {//! goes to one of the states as given inour diagram
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (90)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-30)*DEGTORAD ); 
	       m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (0)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-30)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (0)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (90)*DEGTORAD );
			}
			else
			{//! goes to one of the states as given inour diagram
				m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-40)*DEGTORAD );
				m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (80)*DEGTORAD );
				m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-70)*DEGTORAD ); 
				m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (-10)*DEGTORAD );
				m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (-10)*DEGTORAD );
				m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-70)*DEGTORAD );
				m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-40)*DEGTORAD );
				m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (80)*DEGTORAD );
			}
	  }
	  if(dummy1&&((int)(m_bodyG1->GetPosition().x-x1)/2%4==2)){//! if distance/4 is even
		   revoluteJointDef73.lowerAngle=90*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=90*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef75.lowerAngle=90*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=90*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=-90*DEGTORAD;
		   if(m_bodyG1->GetLinearVelocity().x<50)
		   {//! goes to one of the states as given inour diagram
	       m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (60)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-0)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-0)*DEGTORAD );
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-30)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (60)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-30)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	       }
	       else
	       {//! goes to one of the states as given inour diagram
		   m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (80)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-10)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-40)*DEGTORAD );
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-70)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (80)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (-10)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-70)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-40)*DEGTORAD );
			}
		}
		if(dummy1&&((int)(m_bodyG1->GetPosition().x-x1)/2%4==3)){//! if distance/4 is even
		   revoluteJointDef73.lowerAngle=90*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=90*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef75.lowerAngle=90*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-90*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=90*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=-90*DEGTORAD;
		   if(m_bodyG1->GetLinearVelocity().x<50)
		   {//! goes to one of the states as given inour diagram
	       m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (100)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-0)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-0)*DEGTORAD );
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-30)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (100)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-30)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	       }
	       else
	       {//! goes to one of the states as given inour diagram
		   m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (80)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-10)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-40)*DEGTORAD );
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-70)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (80)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (-10)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-70)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-40)*DEGTORAD );
			}
		}
	    if(mobject->GetPosition().x>=190)
	    mobject->SetLinearVelocity(b2Vec2(0,0));
	    if(m_bodyG1->GetPosition().x-mobject->GetPosition().x>-20)
	    {//! when moving  object is too near we stop horse
		  m_bodyG1->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG2->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG3->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG4->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG5->SetLinearVelocity(b2Vec2(0,0));
		}
	    float ax=mobject->GetPosition().x-tri->GetPosition().x;
	  float ay=mobject->GetPosition().y-tri->GetPosition().y;
	  if(ax<0)
	  ax*=-1;
	  if(ay<0)
	  ay*=-1;
	  if( ax<3 && ay<5.5)
	  {
		  mobject->SetTransform(mobject->GetPosition(), 90*DEGTORAD );
		  tri->SetTransform(mobject->GetPosition(), 0*DEGTORAD );
		  mobject->SetLinearVelocity(b2Vec2(0,0));
		  tri->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG1->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG2->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG3->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG4->SetLinearVelocity(b2Vec2(0,0));
		  m_bodyG5->SetLinearVelocity(b2Vec2(0,0));
	   }
	   if(tri->GetPosition().y<4)
	   tri->SetLinearVelocity(b2Vec2(0,0));
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
