#include "testApp.h"
#include "ofxAutoControlPanel.h"

#include "btBulletDynamicsCommon.h"

#include "RiggedBoxScene.h"
#include "RiggedBox.h"

#include "ofxBtHelper.h"

ofxAutoControlPanel gui;

RiggedBoxScene scene;

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
    
    m_picker0.setWorld(scene.getDynamicsWorld());
    m_picker1.setWorld(scene.getDynamicsWorld());
    
    if (scene.getRiggedBoxes().size()) {
        btRigidBody *bd0 = scene.getRiggedBoxes().at(0)->m_bodies.at(9);
        btRigidBody *bd1 = scene.getRiggedBoxes().at(0)->m_bodies.at(11);
        
        m_picker0.attatchRigidBody(bd0);
        m_picker1.attatchRigidBody(bd1);
    }

    const float lightPosition[] = { -1000.0f, 2000.0f, -1000.0f };
    gl::calcShadowMatrix(gl::kGroundPlaneYUp, lightPosition, m_shadowMat.getPtr());
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
    
    ramPushAll();
    ramCameraBegin();
    scene.draw();
    ramCameraEnd();
    ramPopAll();
    
	ramPushAll();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    
    ramCameraBegin();
    {
    }
    ramCameraEnd();
    
    ramPopAll();
}




#pragma mark - ram methods
//--------------------------------------------------------------
void testApp::drawFloor()
{

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
    
    glDisable(GL_DEPTH_TEST);
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

    
    
    
    
    const ofVec3f lhp = actor.getNode(ramActor::JOINT_LEFT_HAND).getPosition();
    const ofVec3f rhp = actor.getNode(ramActor::JOINT_RIGHT_HAND).getPosition();
    m_picker0.updatePosition(btVector3(lhp.x, lhp.y, lhp.z));
    m_picker1.updatePosition(btVector3(rhp.x, rhp.y, rhp.z));
    
    ramPopAll();
}

#pragma mark - oF Events
//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
    scene.keyPressed(key);    
}











