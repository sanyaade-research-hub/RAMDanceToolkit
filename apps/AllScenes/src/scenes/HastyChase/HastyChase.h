#pragma once

#include "ramMain.h"

class HastyChase : public ramBaseScene
{
public:
	
	map<string, ramTimeShifter> time_shifters;
	float buffer_time;
	float rate;
	bool draw_line;
	
	void setupControlPanel()
	{
		ramControlPanel &gui = ramGetGUI();
		
		buffer_time = 3600;
		rate = 1.5;
		
		gui.addSlider("buffer_time", 1, 10000, &buffer_time);
		gui.addSlider("rate", -2, 3, &rate);
		
		gui.addToggle("draw_line", &draw_line);
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
	
	void drawHUD()
	{

	}
	
	void drawActor(ramActor& actor)
	{
		ramTimeShifter &TS = time_shifters[actor.getName()];
		TS.setNumBufferFrame(buffer_time);
		TS.setRate(rate);

		const ramActor &chaser = TS.update(actor);
		ramDrawBasicActor(chaser);
		
		if (draw_line)
			ramDrawNodeCorresponds(actor, chaser);
		
		ofPushStyle();
		
		ofSetColor(255, 127);
		ofNoFill();
		
		for (int i = 0; i < chaser.getNumNode(); i++)
		{
			const ramNode &node = chaser.getNode(i);
			ramBox(node, node.getVelocity().length() * 2);
		}
		
		ofPopStyle();
	}
	
	void drawRigid(ramRigidBody &rigid)
	{
	}
	
	const string getName() { return "Hasty Chase"; }
};
