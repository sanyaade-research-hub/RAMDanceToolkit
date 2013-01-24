#include "testApp.h"
#include "ofxAutoControlPanel.h"

#include "btBulletDynamicsCommon.h"

#include "GearScene.h"

ofxAutoControlPanel gui;

GearScene scene;

#pragma mark - oF methods
//--------------------------------------------------------------
void testApp::setup()
{
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
	ofBackground(0);
	oscReceiver.setup(10000);
	
	// enable ramBaseApp::setup, update, draw, exit
	ramEnableAllEvents();
	
	// gui setup
	ofxControlPanel::setTextColor(simpleColor(255,255,255,255));
	ofxControlPanel::setBackgroundColor(simpleColor(0,0,0,127));
    
    scene.setup();
    
    ofEasyCam *cam = (ofEasyCam *)&ramCameraManager::instance().getActiveCamera();
    cam->setDistance(500);
    
    
    if (scene.getGears().size()>1) {
        Gear gear = scene.getGears().at(1);
//        gear.body->setActivationState(DISABLE_DEACTIVATION);
//        gear.hinge->enableAngularMotor(true, -10.0f, 20.0f);
    }
}

//--------------------------------------------------------------
void testApp::update()
{
	oscReceiver.update();
    scene.update();
}

//--------------------------------------------------------------
void testApp::draw()
{
    
    //ofSetupScreen();
    ramPushAll();
    ramCameraBegin();
    scene.draw();
    ramCameraEnd();
    ramPopAll();

    ramPushAll();
    ramCameraBegin();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    {
    
    }
    glDisable(GL_DEPTH_TEST);
    ramCameraEnd();
    ramPopAll();
}




#pragma mark - ram methods
//--------------------------------------------------------------
void testApp::drawFloor()
{
//    ramBasicFloor(ramFloor::FLOOR_CHECKER_PATTERN);
}

//--------------------------------------------------------------
void testApp::drawActor(ramActor &actor)
{
	ramPushAll();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    for (int i=0; i<actor.getNumNode(); i++)
	{
		ramNode &node = actor.getNode(i);
		float jointSize = (i==ramActor::JOINT_HEAD) ? 6.0 : 3.0;
		
		node.transformBegin();
		ofSetColor( getRamColor(ramColor::BLUE_NORMAL) );
        ofNoFill();
		ofBox(jointSize);
		node.transformEnd();
		
		if (node.hasParent())
		{
			ofSetColor(getRamColor(ramColor::RED_NORMAL));
			ofLine(node, *node.getParent());
		}
	}
    
    ofVec3f pos = actor.getNode(ramActor::JOINT_LEFT_HAND).getPosition();
    
//    m_picker.updatePosition(btVector3(pos.x, pos.y, pos.z));
    
    glDisable(GL_DEPTH_TEST);
    ramPopAll();
}

#pragma mark - oF Events
//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
    scene.keyPressed(key);
}
