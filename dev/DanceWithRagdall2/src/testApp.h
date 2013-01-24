#pragma once

#include "ofMain.h"

#include "ramMain.h"

#include "btPicker.h"

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
    
    enum { LA, RA, RL, COUNT };
    
    btTypedConstraint *m_pickConstraint[COUNT];
    btVector3   m_handPos[COUNT];
    btRigidBody *m_pickedBody[COUNT];
    
    btPicker m_picker;
};
