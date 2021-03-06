#pragma once

class BigBox : public ramBaseScene
{
	vector<float> mSizeArray;
	float mBoxLineWidth;
	float mMasterBoxSize;
	bool mUseSingleColor;
	ofFloatColor mLineColor;
	
public:

	BigBox() : mBoxLineWidth(2.0), mMasterBoxSize(300.0), mUseSingleColor(true), mLineColor(0.840, 1.000, 0.419)
	{
		mSizeArray.clear();
		mSizeArray.resize(ramActor::NUM_JOINTS);
		for (int i=0; i<mSizeArray.size(); i++)
			mSizeArray.at(i) = mMasterBoxSize;
	}

	void setupControlPanel()
	{
#ifdef RAM_GUI_SYSTEM_OFXUI
		
		ramGetGUI().addToggle("Use single color", &mUseSingleColor);
		ramGetGUI().addColorSelector("line color", &mLineColor);
		ramGetGUI().addSlider("Line width", 0.0, 10.0, &mBoxLineWidth);
		ramGetGUI().addSlider("Master box size", 0.0, 1000.0, &mMasterBoxSize);

		for (int i=0; i<ramActor::NUM_JOINTS; i++)
			ramGetGUI().addSlider(ramActor::getJointName(i), 0.0, 1000.0, &mSizeArray.at(i));

		ofAddListener(ramGetGUI().getCurrentUIContext()->newGUIEvent, this, &BigBox::onPanelChanged);
		
#endif
	}
	
	void draw()
	{
		ramBeginCamera();
		
		for(int i=0; i<getNumNodeArray(); i++)
		{
			const ramNodeArray &NA = getNodeArray(i);
			drawBigBox(NA);
		}
		
		ramEndCamera();
	}
	
	void drawBigBox(const ramNodeArray& NA)
	{
		for (int i=0; i<NA.getNumNode(); i++)
		{
			const int keyJoint = NA.isActor() ? ramActor::JOINT_HEAD : 0;
			
			const ramNode &node = NA.getNode(i);
			float boxSize = (i==keyJoint) ? 6 : 3;
			float bigBoxSize = mSizeArray.at(i);
			
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glPushMatrix();
			{
				ofPushStyle();
				ofNoFill();
				
				glEnable(GL_DEPTH_TEST);
				
				/*!
				 big box
				 */
				if (mUseSingleColor)
				{
					ofSetColor(mLineColor);
				}
				else
				{
					if (i%3 == 0)
					{
						ofSetColor( ramColor::BLUE_DEEP );
					}
					else if (i%3 == 1)
					{
						ofSetColor( ramColor::BLUE_NORMAL );
					}
					else
					{
						ofSetColor( ramColor::BLUE_LIGHT );
					}
				}
				
				ofSetLineWidth(mBoxLineWidth);
				node.beginTransform();
				ofBox(bigBoxSize);
				node.endTransform();
				
				ofPopStyle();
			}
			glPopMatrix();
			glPopAttrib();
		}
	}
	
	void onPanelChanged(ofxUIEventArgs& e)
	{
		string name = e.widget->getName();
		
		if (name == "Master box size")
		{
			for (int i=0; i<mSizeArray.size(); i++)
				mSizeArray.at(i) = mMasterBoxSize;
		}
	}
	
	const string getName() { return "Big Box"; }
};
