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
    
    ragdollScene.setup();
    
    ofEasyCam *cam = (ofEasyCam *)&ramCameraManager::instance().getActiveCamera();
    cam->setDistance(500);
    
    m_pickConstraint = NULL;
    
    keyPressed('e');
    keyPressed(' ');
}

//--------------------------------------------------------------
void testApp::update()
{
	oscReceiver.update();
    ragdollScene.update();
    
    if (m_pickConstraint) {
        
        if (m_pickConstraint->getConstraintType() == D6_CONSTRAINT_TYPE) {
            btGeneric6DofConstraint* pickCon = static_cast<btGeneric6DofConstraint*>(m_pickConstraint);
            if (pickCon) {
                pickCon->getFrameOffsetA().setOrigin(m_handPos);
            }
        }
    }
}

//--------------------------------------------------------------
void testApp::draw()
{
    
    //ofSetupScreen();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    ramCameraBegin();
    ragdollScene.draw();
    ramCameraEnd();
    glPopAttrib();
    
	glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glPushMatrix();
    ramCameraBegin();
    {
    }
    ramCameraEnd();
    glPopMatrix();
    glDisable(GL_DEPTH_TEST);
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
	glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glPushMatrix();
    ofPushStyle();
    
    ramBasicActor(actor);
    
    ofVec3f v = actor.getNode(ramActor::JOINT_LEFT_HAND).getPosition();
    m_handPos.setValue(v.x, v.y, v.z);
    
    ofPopStyle();
    glPopMatrix();
    glDisable(GL_DEPTH_TEST);
    glPopAttrib();
}

#pragma mark - oF Events
//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
    ragdollScene.keyPressed(key);
    
    if (key == ' ') {
        
            if (m_pickConstraint && ragdollScene.getDynamicsWorld())
            {
                ragdollScene.getDynamicsWorld()->removeConstraint(m_pickConstraint);
                delete m_pickConstraint;
                m_pickConstraint = NULL;
                //printf("removed constraint %i",gPickingConstraintId);
                m_pickedBody->forceActivationState(ACTIVE_TAG);
                m_pickedBody->setDeactivationTime( 0.f );
                m_pickedBody = NULL;
            }
        
        
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
                part = ofRandom(0, RagDoll::BODYPART_COUNT);
                
                m_pickedBody = dall->m_bodies[part];
                
                if (m_pickedBody) {
                    //other exclusions?
                    if (!(m_pickedBody->isStaticObject() || m_pickedBody->isKinematicObject())) {
                        btRigidBody *body = m_pickedBody;
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
                        m_pickConstraint = dof6;
                        
                        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,0);
                        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,1);
                        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,2);
                        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,3);
                        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,4);
                        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,5);
                        
                        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,0);
                        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,1);
                        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,2);
                        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,3);
                        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,4);
                        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,5);
                        
                        //                        PickingPos = rayTo;
                        //                        gHitPos = pickPos;
                        //
                        //                        gOldPickingDist  = (pickPos-rayFrom).length();
                    }
                }
                
            }
        }
    }
}
