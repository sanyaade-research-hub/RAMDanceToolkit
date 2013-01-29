#pragma once
#include "ramMain.h"


class ramGhost
{
public:
	ramGhost() :
	historySize(10),
	distance(150),
	speed(27)
	{
		clear();
	}
	virtual ~ramGhost() {}
	
	void clear()
	{
		recordedActors.clear();
		recordedRigids.clear();
	}
	
	void update(ramActor &present)
	{
		
		
		if (present.getNumNode() != 0)
			recordedActors.push_back(present);
		
		if (recordedActors.size() > historySize)
			recordedActors.pop_front();
		
		ramNodeArray &presentArray = recordedActors.back();
		ramNodeArray &pastArray = recordedActors.front();
		
		for (int i = 0; i < presentArray.getNumNode(); i++)
		{
			
			const ofVec3f& p0 = presentArray.getNode(i).getPosition();
			const ofVec3f& p1 = pastArray.getNode(i).getPosition();
			
			// position
			ofVec3f d = (p0 - p1);
			d.normalize();
			d *= distance;
			ofVec3f v = p0 + d;
			
			ofVec3f vec = ghostActor.getNode(i).getPosition();
			vec += (v - vec) * speed * 0.001;
			
			// quaternion
			const ofQuaternion& quat = presentArray.getNode(i).getOrientationQuat();
			
			ramNode& node = ghostActor.getNode(i);
			node.setPosition(vec);
			node.setOrientation(quat);
			node.getAccerelometer().update(vec, quat);
		}
	}
	
	void update(ramRigidBody &present)
	{
		if (present.getNumNode() != 0)
			recordedRigids.push_back(present);
		
		if (recordedRigids.size() > historySize)
			recordedRigids.pop_front();
		
		ramNodeArray &presentArray = recordedRigids.back();
		ramNodeArray &pastArray = recordedRigids.front();
		
		for (int i = 0; i < presentArray.getNumNode(); i++)
		{
			const ofVec3f& p0 = presentArray.getNode(i).getPosition();
			const ofVec3f& p1 = pastArray.getNode(i).getPosition();
			
			// position
			ofVec3f d = (p0 - p1);
			d.normalize();
			d *= distance;
			ofVec3f v = p0 + d;
			
			ofVec3f vec = ghostRigidBody.getNode(i).getPosition();
			vec += (v - vec) * speed * 0.001;
			
			// quaternion
			const ofQuaternion& quat = presentArray.getNode(i).getOrientationQuat();
			
			ramNode& node = ghostRigidBody.getNode(i);
			node.setPosition(vec);
			node.setOrientation(quat);
			node.getAccerelometer().update(vec, quat);
		}
	}
	
	inline ramActor& getActor() { return ghostActor; }
	inline ramRigidBody& getRigidBody() { return ghostRigidBody; }
	
	inline void setDistance(const float d) { distance = d; }
	inline void setSpeed(const float s) { speed = s; }
	inline void setHistorySize(const unsigned int m) { historySize = m; }
	
	inline float getdistance() { return distance; }
	inline float getspeed() { return speed; }
	inline unsigned int getHistorySize() { return historySize; }
	
protected:
	deque<ramActor> recordedActors;
	deque<ramRigidBody> recordedRigids;
	ramActor ghostActor;
	ramRigidBody ghostRigidBody;
	
	unsigned int historySize;
	float distance, speed;
};