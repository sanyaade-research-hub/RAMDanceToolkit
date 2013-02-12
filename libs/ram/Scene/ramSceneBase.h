#pragma once
#include "ofMain.h"

#include "ramActorManager.h"
#include "ramCameraManager.h"

#include "ramControllable.h"

#include "ofxUI.h"


class ramControlPanel;

class ramSceneBase : public ramControllable
{
	friend class ramControlPanel;
	
public:
	ramSceneBase() : bEnabled(false) {}
	virtual ~ramSceneBase(){}
	
	virtual string getSceneName() { return "unnamed scene"; }

	inline ramActorManager& getActorManager() { return ramActorManager::instance(); }
	
	inline bool hasActor(const string &key) { return ramActorManager::instance().hasActor(key); }
	inline ramActor& getActor(string name) { return ramActorManager::instance().getActor(name); }
	
	inline size_t getNumActor() { return ramActorManager::instance().getNumActor(); }
	inline ramActor& getActor(int index) { return ramActorManager::instance().getActor(index); }

	inline bool hasRigidBody(const string &key) { return ramActorManager::instance().hasRigidBody(key); }
	inline ramRigidBody& getRigidBody(string name) { return ramActorManager::instance().getRigidBody(name); }
	
	inline size_t getNumRigidBody() { return ramActorManager::instance().getNumRigidBody(); }
	inline ramRigidBody& getRigidBody(int index) { return ramActorManager::instance().getRigidBody(index); }

	inline ofCamera& getActiveCamera() { return ramCameraManager::instance().getActiveCamera(); }
	
	virtual void setup() {}
	virtual void update() {}
	virtual void draw() {}
	virtual void drawActor(ramActor &actor) {}
	virtual void drawRigid(ramRigidBody &rigid) {}
	
	inline void enable(){ bEnabled = true; }
	inline void disable() { bEnabled = false; }
	inline void toggle() { bEnabled ^= true; }
	inline void setEnabled(bool b) { bEnabled = b; }
	inline bool isEnabled() { return bEnabled; }
	
	ramSceneBase* getPtr() { return this; }
	
protected:
	
	ramControlPanel& gui() { return *guiPtr; }
	
private:
	
	bool bEnabled;
	ramControlPanel *guiPtr;
	
};
