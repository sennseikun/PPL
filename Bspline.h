#pragma once
#define TESTDLLSORT_API __declspec(dllexport)
#include <vector>

using namespace std;

struct Knot {
	double x;
	double y;
	double z;
};

class Bspline{
private:
	vector<Knot> path;
	vector<Knot> interpolation;
	int size;
	int resolution;
	int dimensions;
	int order;

	void makeRandomPath();
	double calculateBasisFunction(double i, double k, double point);
	void findPoint(int t);
public:
	Bspline(int size,  int resolution, int order);
	int getPathSize() const;
	int getInterpolationSize() const;
	vector<Knot> getPath() const;
	vector<Knot> getInterpolation() const;
	void savePathToFile(string filename);
	void saveInterpolationToFile(string filename);
	void makeBspline();
};

ostream& operator<<(ostream& os, const vector<Knot>& rhs);
Knot operator*(const Knot& lhs, double rhs);
Knot& operator+=(Knot& lhs, const Knot& rhs);
extern "C" {
	TESTDLLSORT_API void bsplineArray(int knots, int resolution, int order, double* interpolationArray, int arraySize);
}