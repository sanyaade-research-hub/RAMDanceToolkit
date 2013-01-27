#include "testApp.h"
#include "ofxAutoControlPanel.h"

#include "btBulletDynamicsCommon.h"

#include "BaseConstrains.h"
#include "ConstrainsTestScene.h"

#include "ofxBtHelper.h"

ofxAutoControlPanel gui;

ConstrainsTestScene scene;

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
    
#if CHAIN_MODE
    m_picker0.setWorld(scene.getDynamicsWorld());
    m_picker1.setWorld(scene.getDynamicsWorld());
#endif
    
#if RAIL_MODE
    m_pickers.clear();
    for (int i=0; i<ramActor::NUM_JOINTS; i++) {
        btPicker picker;
        picker.setWorld(scene.getDynamicsWorld());
        m_pickers.push_back(picker);
    }
#endif
    
    const float lightPosition[] = { -1000.0f, 2000.0f, -1000.0f };
    gl::calcShadowMatrix(gl::kGroundPlaneYUp, lightPosition, m_shadowMat.getPtr());
    
    keyPressed(' ');
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
    ofBackgroundGradient(ofColor(64), ofColor(0));
    
    ramPushAll();
    ramCameraBegin();
    scene.draw();
    ramCameraEnd();
    ramPopAll();
}




#pragma mark - ram methods
//--------------------------------------------------------------
void testApp::drawFloor()
{
    //ramBasicFloor(ramFloor::FLOOR_CHECKER_PATTERN);
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
		ofSetColor(ramColor::BLUE_LIGHT);
        ofNoFill();
		ofBox(jointSize);
		node.transformEnd();
		
		if (node.hasParent())
		{
			ofSetColor(ramColor::RED_LIGHT);
			ofLine(node, *node.getParent());
		}
	}
    
    ramPushAll();
    {
        glTranslated(0.0f, 10.0f, 0.0f);
        
        ofMultMatrix(m_shadowMat);
        ofEnableAlphaBlending();
        
        ofSetColor(0, 120);
        
        for (int i=0; i<actor.getNumNode(); i++)
        {
            ramNode &node = actor.getNode(i);
            float jointSize = (i==ramActor::JOINT_HEAD) ? 6.0 : 3.0;
            
            node.transformBegin();
            ofNoFill();
            ofBox(jointSize);
            node.transformEnd();
            
            if (node.hasParent())
            {
                ofLine(node, *node.getParent());
            }
        }
    }
    ramPopAll();

#if CHAIN_MODE
    if (nameA=="") {
        nameA = actor.getName();
    }
    
    if (actor.getName()!=nameA && nameA!="" && nameB=="") {
        nameB = actor.getName();
    }
    

    //cout << nameA << "/" << nameB << endl;

    if (actor.getName()==nameA) {
        const ofVec3f lhp = actor.getNode(ramActor::JOINT_RIGHT_HAND).getPosition();
        m_picker0.updatePosition(btVector3(lhp.x, lhp.y, lhp.z));
    }
    
//    if (actor.getName()==nameB) {
//        const ofVec3f lhp = actor.getNode(ramActor::JOINT_LEFT_HAND).getPosition();
//        m_picker1.updatePosition(btVector3(lhp.x, lhp.y, lhp.z));
//    }
#endif
    
#if RAIL_MODE
    for (int i=0; i<actor.getNumNode(); i++) {
        const ofVec3f p = actor.getNode(i).getPosition();
        m_pickers.at(i).updatePosition(btVector3(p.x, p.y, p.z));
    }
#endif
    
    ramPopAll();
}

#pragma mark - oF Events
//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
    scene.keyPressed(key);
    
    if (key == ' ') {
        if (scene.getConstrains().size()) {
#if CHAIN_MODE
            btRigidBody *bd0 = scene.getConstrains().at(0)->m_bodies.at(0);
            m_picker0.attatchRigidBody(bd0);

            //const int s = scene.getConstrains().at(0)->m_bodies.size();
            //btRigidBody *bd1 = scene.getConstrains().at(0)->m_bodies.at(s-1);
            //m_picker1.attatchRigidBody(bd1);
#endif
            
#if RAIL_MODE
            for (int i=0; i<scene.getConstrains().at(0)->m_bodies.size(); i++) {
                btRigidBody *bd1 = scene.getConstrains().at(0)->m_bodies.at(i);
                m_pickers.at(i).attatchRigidBody(bd1);
            }
#endif
        }
    }
}











