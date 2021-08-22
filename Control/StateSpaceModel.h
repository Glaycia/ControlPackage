#include <"LQR.h">

struct StateSpaceModel {
	int states;
	int inputs;
	int outputs;
	
	Matrix<states, states> a;
	Matrix<states, inputs> b;
	Matrix<outputs, states> c;
	Matrix<outputs, inputs> d;

	Vector<states> x;
	Vector<states> r;
	Vector<inputs> u;
	Vector<outputs> y;

	Vector<states> gains;
};

void update(Vector<states> newX);
void update(Vector<states> newX, Vector<states> newR);
