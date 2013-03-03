#pragma once

#include "ramBaseScene.h"
#include "ramTSVCoder.h"
#include "ControlSegment.h"


/*
 almost all things of this class depends on ofxUI
*/
class ramActorsScene : public ramBaseScene
{
public:
	
	ramActorsScene();
	
	const string getName();
	void setupControlPanel();
	void setup();
	void update();
	void draw();
	void drawHUD();
	
	void onActorSetup(const ramActor &actor);
	void onRigidSetup(const ramRigidBody &rigid);
	void onActorExit(const ramActor &actor);
	void onRigidExit(const ramRigidBody &rigid);
	void onEnabled();
	
	void drawNodes(const ramNodeArray &NA); // experimental
	
private:
	
	void addControlSegment(const ramNodeArray &NA);
	void removeControlSegment(const ramNodeArray &NA);
	void rebuildControlPanel();

	
	/// ActorsPanel
	ofxUICanvasPlus *mLocalPanel;
	
	
	/// ControlSegment map
	map<string, ControlSegment*> mSegmentsMap;
	typedef map<string, ControlSegment*>::iterator SegmentsIter;
	
	
	/// to encode/decode to tsv file
	ramTSVCoder coder;
	
	
	/// fonts
	ofTrueTypeFont font;
	float fontSize;
	
	
	/// update flags
	bool mNeedUpdatePanel;
	bool bRecording;
	
	
	/// draw method flags
	ofLight light;
	bool bUseNewActor;
	bool bUseLight;
	
	
	/// to store actor color
	ofxXmlSettings XML;
	
};
