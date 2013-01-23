#pragma once

#include "ofMain.h"

#include "ramMain.h"

class testApp : public ramBaseApp
{
public:

	// of methods
	// ------------------------
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	
	// ram methods
	// ------------------------
	void drawFloor();
    void drawActor(ramActor &actor);

    
	// ...
	// ------------------------
	ramOscReceiver oscReceiver;
    
    btTypedConstraint*  m_pickConstraint;
    btVector3   m_handPos;
    btRigidBody *m_pickedBody;
    
};