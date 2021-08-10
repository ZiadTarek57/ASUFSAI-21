#include<iostream>
#include <math.h>
using namespace std;

//https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
//Got the equations from this link

class Bicycle
{
public:
	double x;
	double y;
	double theta;
	float delta = 0;
	float beta = 0;
	int l = 5;
	float lr = 3;
	float time = 0.01;
public:
	void init() {
		x = 0;
		y = 0;
		theta = 0;
		beta = 0;
		delta = 0;
	}
	void move(float right, float left, float delta) {
		double r = atan(delta) / l;
		double v = r * (right + left) / 2;
		double xdot = v * cos(theta + beta);
		double ydot = v * sin(theta + beta);
		double thetadot = (v / l) * (cos(beta) * tan(delta));
		beta = atan(lr * tan(delta) / l);
		x += xdot * time;
		y += ydot * time;
		theta += thetadot * time;
		delta = delta;
	}
};
int main() {

	Bicycle b;
	b.x = 0;
	b.y = 0;
	b.theta = 0;
	for (int i = 0; i < 20; i++)
	{
		b.move(3, 3, 0.04);
		cout << "Current Position: X-axis direction  = " << b.x << " Y-axis direction  = " << b.y << " yaw angle = " << (float)b.theta;
		cout << endl;
	}
}