#pragma once

class ramLowPassFilter : public ramBaseFilter
{
public:

	ramLowPassFilter() : amount(0.1) {}

	const ramNodeArray& get(size_t index = 0) const { return copy; }
	size_t getSize() const { return 1; }

	void setupControlPanel()
	{
		ramGetGUI().addSlider("LowPass amount", 0.0, 1.0, &amount);
	}

	const string getName() { return "ramLowPassFilter"; }

#pragma mark -

	void setAmount(float a) { amount = a; }

	const ramNodeArray& filter(const ramNodeArray& src)
	{
		if (src.getNumNode() != copy.getNumNode()) copy = src;

		for (int i = 0; i < src.getNumNode(); i++)
		{
			ofVec3f input = src.getNode(i).getGlobalPosition();
			ofVec3f output = copy.getNode(i).getGlobalPosition();
			copy.getNode(i).setGlobalPosition((input * amount) + (output * (1 - amount)));
		}

		return copy;
	}

protected:

	ramNodeArray copy;
	float amount;
};