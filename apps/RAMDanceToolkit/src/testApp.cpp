#include "testApp.h"

/*!
 Scenes
 */
#include "LineDrawing.h"
LineDrawing drawLines;

#include "BigBox.h"
BigBox bigbox;

#include "Future.h"
Future future;

#include "Donuts.h"
Donuts donuts;

#include "Stamp.h"
Stamp stamp;

#include "Expansion.h"
Expansion expansion;

#include "Particles.h"
Particles particles;

#include "Abacus.h"
Abacus abacus;

#include "SoundCube.h"
SoundCube soundcube;

#include "UpsideDown.h"
UpsideDown upsideDown;

#include "Kepler.h"
Kepler kepler;

#include "HastyChase.h"
HastyChase hastyChase;

#include "ColorGrid.h"
ColorGrid colorGrid;

#include "ThreePoints.h"
ThreePoints threePoints;

#include "FourPoints.h"
FourPoints fourPoints;

#include "Chain.h"
Chain chain;    

#include "Monster.h"
Monster monster;

#include "Laban.h"
Laban laban;

#include "Notation.h"
Notation notation;


#pragma mark - oF methods
//--------------------------------------------------------------
void testApp::setup()
{
	ofSetFrameRate(120);
	ofSetVerticalSync(true);
	
	/// ram setup
	// ------------------
	ramInitialize(10000);
	
	
	/// scenes setup
	// ------------------
	ramSceneManager& sceneManager = ramSceneManager::instance();
	sceneManager.addScene( drawLines.getPtr() );
	sceneManager.addScene( bigbox.getPtr() );
	sceneManager.addScene( future.getPtr() );
	sceneManager.addScene( donuts.getPtr() );
	sceneManager.addScene( stamp.getPtr() );
	sceneManager.addScene( expansion.getPtr() );
	sceneManager.addScene( particles.getPtr() );
	sceneManager.addScene( abacus.getPtr() );
	sceneManager.addScene( soundcube.getPtr() );
	sceneManager.addScene( upsideDown.getPtr() );
	sceneManager.addScene( kepler.getPtr() );
	sceneManager.addScene( hastyChase.getPtr() );
	sceneManager.addScene( colorGrid.getPtr() );
	sceneManager.addScene( threePoints.getPtr() );
	sceneManager.addScene( fourPoints.getPtr() );
	sceneManager.addScene( chain.getPtr() );
	sceneManager.addScene( monster.getPtr() );
	sceneManager.addScene( laban.getPtr() );
	sceneManager.addScene( notation.getPtr() );
}

//--------------------------------------------------------------
void testApp::update()
{

}

//--------------------------------------------------------------
void testApp::draw()
{
	
}

#pragma mark - ram methods
//--------------------------------------------------------------
void testApp::drawActor(const ramActor &actor)
{
	
}

//--------------------------------------------------------------
void testApp::drawRigid(const ramRigidBody &rigid)
{
	
}

#pragma mark - oF Events
//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
	
	if(key=='/')
	{
		ramLoadSettings("Settings/scene.xml");
	}
	
	if(key=='_')
	{
		ramSaveSettings("Settings/scene.xml");
	}
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
