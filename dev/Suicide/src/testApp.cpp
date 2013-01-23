#include "testApp.h"
#include "ofxAutoControlPanel.h"

#include "btBulletDynamicsCommon.h"

#include "RagdollScene.h"
#include "RagDoll.h"

#include "ofxBtHelper.h"

ofxAutoControlPanel gui;

RagdollScene ragdollScene;

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
    
    ofEasyCam *cam = (ofEasyCam *)&ramCameraManager::instance().getActiveCamera();
    cam->setDistance(500);
    
    ragdollScene.setup();
    
    for (int i=0; i<4; i++) {
        keyPressed('e');
        keyPressed(' ');
    }
    keyPressed('e');
    keyPressed('e');
}

//--------------------------------------------------------------
void testApp::update()
{
	oscReceiver.update();
    ragdollScene.update();
}

//--------------------------------------------------------------
void testApp::draw()
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    ramCameraBegin();
    ragdollScene.draw();
    ramCameraEnd();
    glPopAttrib();
}




#pragma mark - ram methods
//--------------------------------------------------------------
void testApp::drawFloor()
{
}

//--------------------------------------------------------------
void testApp::drawActor(ramActor &actor)
{
}

#pragma mark - oF Events
//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
    ragdollScene.keyPressed(key);
    
    if (key == ' ') {
        
        btAlignedObjectArray<RagDoll *> &ragdolls = ragdollScene.getRagdolls();
        if (ragdolls.size() != 0) {
            
            for (int i=0; i<ragdolls.size(); i++) {
                RagDoll *dall = ragdolls.at(i);
                
                //enum
                //{
                //    BODYPART_PELVIS = 0,
                //    BODYPART_SPINE,
                //    BODYPART_HEAD,
                //    
                //    BODYPART_LEFT_UPPER_LEG,
                //    BODYPART_LEFT_LOWER_LEG,
                //    
                //    BODYPART_RIGHT_UPPER_LEG,
                //    BODYPART_RIGHT_LOWER_LEG,
                //    
                //    BODYPART_LEFT_UPPER_ARM,
                //    BODYPART_LEFT_LOWER_ARM,
                //    
                //    BODYPART_RIGHT_UPPER_ARM,
                //    BODYPART_RIGHT_LOWER_ARM,
                //    
                //    BODYPART_COUNT
                //};
                
                //int part = 0;
                int part = RagDoll::BODYPART_HEAD;
//                while ((part%2) == 0)
//                    part = ofRandom(0, RagDoll::BODYPART_COUNT);
                
                btRigidBody* body = dall->m_bodies[part];
                
                if (body) {
                    //other exclusions?
                    if (!(body->isStaticObject() || body->isKinematicObject())) {
                        body = body;
                        body->setActivationState(DISABLE_DEACTIVATION);
                        
                        
                        //btVector3 pickPos = rayCallback.m_hitPointWorld;
                        btVector3 pickPos = body->getCenterOfMassTransform().getOrigin();
                        
                        //printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
                        
                        btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
                        
                        btTransform tr;
                        tr.setIdentity();
                        tr.setOrigin(localPivot);
                        btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*body, tr,false);
                        dof6->setLinearLowerLimit(btVector3(0,0,0));
                        dof6->setLinearUpperLimit(btVector3(0,0,0));
                        dof6->setAngularLowerLimit(btVector3(0,0,0));
                        dof6->setAngularUpperLimit(btVector3(0,0,0));
                        
                        ragdollScene.getDynamicsWorld()->addConstraint(dof6,true);

                    }
                }
                
            }
        }
    }
}
