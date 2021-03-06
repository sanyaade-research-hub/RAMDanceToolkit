#pragma once

class MyScene2 : public ramBaseScene
{
private:

	ramGhost ghost;
	float fontColor;

public:
	
	const string getName() { return "MyScene2"; }

	MyScene2(): fontColor(100) {}

	void setupControlPanel()
	{
		ramControlPanel &gui = ramGetGUI();

		gui.addSlider("Font Color", 0.0, 255.0, &fontColor);

		ghost.setupControlPanel();
	}

	void setup()
	{

	}


	void update()
	{

	}

	void draw()
	{

	}

	void drawActor(const ramActor& actor )
	{
		ghost.update(actor);

		ofVec3f pos = ghost.get().getNode(ramActor::JOINT_HEAD);
		pos.y += 30.0;

		ofSetColor(0, fontColor, 0);
		ofDrawBitmapString( "I am " + getName(), pos );

		ramDrawBasicActor((ramActor&)ghost.get());
	}

	void drawRigidBody(const ramRigidBody &rigid)
	{
	}
	
};
