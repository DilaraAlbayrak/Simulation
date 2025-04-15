#pragma once
class Scenario
{
public:
	Scenario() {};
	~Scenario() = default;

	virtual void onLoad() = 0;
	virtual void onUnload() = 0;
	virtual void onUpdate() = 0;
};

