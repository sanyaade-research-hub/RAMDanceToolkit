/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#pragma once

#include "ofMain.h"

//#include "GlutStuff.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btAlignedObjectArray.h"

class	btCollisionShape;
class	btDynamicsWorld;
class	btRigidBody;
class	btTypedConstraint;
class   btBroadphaseInterface;
class   btCollisionShape;
class   btOverlappingPairCache;
class   btCollisionDispatcher;
class   btConstraintSolver;
struct  btCollisionAlgorithmCreateFunc;
class   btDefaultCollisionConfiguration;
class   btCompoundShape;
class   btHingeConstraint;

class   GearShapeDrawer;

struct Gear {
    btCompoundShape     *shape;
    btRigidBody         *body;
    btHingeConstraint   *hinge;
};

class GearScene
{
public:        
    virtual void setup();
    virtual void update();
	virtual void draw();
    virtual void keyPressed(int key);
    
    void addGear(const btScalar radius,
                 const btScalar thickness,
                 const btVector3 &origin,
                 const btQuaternion &orientation,
                 const btVector3 &angularVelocity = btVector3(0.0f,2.0f,0.0f));
    
    inline btAlignedObjectArray<btCollisionShape *> getCollisionShapes() { return m_collisionShapes; }
    
    inline btAlignedObjectArray<Gear> getGears() { return m_gears; }
    
public:
	GearScene();
	virtual ~GearScene();
    
	inline btDynamicsWorld *getDynamicsWorld();
    
	inline btScalar getDeltaTimeMicroseconds();
    
protected:
    virtual void clientResetScene();
    
	btRigidBody* localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape);

    void initPhysics();
	void exitPhysics();
    
    void renderscene(int pass);
    
    void setupEmptyDynamicsWorld();
    
    btClock             m_clock;
    
	///this is the most important class
	btDynamicsWorld     *m_dynamicsWorld;
    
	GearShapeDrawer     *m_shapeDrawer;
	btVector3           m_sundirection;
	btScalar            m_defaultContactProcessingThreshold;
    
    
    //Ragdoll scene
    //keep track of variables to delete memory at the end
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btAlignedObjectArray<Gear> m_gears;
    
	class btBroadphaseInterface*	m_overlappingPairCache;
    
	class btCollisionDispatcher*	m_dispatcher;
    
	class btConstraintSolver*	m_constraintSolver;
    
	class btDefaultCollisionConfiguration* m_collisionConfiguration;
    
    class btRigidBody* m_groundBody;
    
    // for cone-twist motor driving
	float m_Time;
	class btConeTwistConstraint* m_ctc;

};

inline btDynamicsWorld *GearScene::getDynamicsWorld()
{
    return m_dynamicsWorld;
}

inline btScalar GearScene::getDeltaTimeMicroseconds()
{
    btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
    m_clock.reset();
    return dt;
}

