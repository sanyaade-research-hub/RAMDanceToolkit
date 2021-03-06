#include "testApp.h"


#include "CameraMover.h"
CameraMover mover;


#pragma mark - oF methods
//--------------------------------------------------------------
void testApp::setup()
{
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
	ofBackground(ramColor::WHITE);


	/// ram setup
	// ------------------
	ramInitialize(10000);

	mover.setup();
	ramGetGUI().addPanel(&mover);
}

//--------------------------------------------------------------
void testApp::update()
{
	mover.update();
}

//--------------------------------------------------------------
void testApp::draw()
{
	mover.draw();
}



#pragma mark - ram methods
//--------------------------------------------------------------
void testApp::drawActor(const ramActor &actor)
{
	ofSetColor(ramColor::GREEN_LIGHT);
	ramDrawBasicActor(actor);
}

//--------------------------------------------------------------
void testApp::drawRigid(const ramRigidBody &rigid)
{

}




#pragma mark - oF Events
//--------------------------------------------------------------
void testApp::keyPressed(int key)
{

}

//--------------------------------------------------------------
void testApp::keyReleased(int key)
{

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo)
{

}