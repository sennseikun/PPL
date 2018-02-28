#include <iostream>
#include <cmath>

using std::cout;
using std::endl;

// Will populate argument "point" with n points defining the path of a helix
// argument "point" is expected to have length at least 3*n
// point[3*k+i] stores the i'th component of the k'th point
// so (f.ex. (point[3*2+0], point[3*2+1], point[3*2+2]) will be the coordinates of the third point
// Lines are implicitly defined by connecting consecutive points

// N > ~100 is probably needed to see the helix

void generateDummyLine_(double*point, int n) {
	cout << "Hello world!" << endl;
	double spins = 3;
	double radius = 1;
	double z_per_radian = 0.3;
	for (int i = 0; i < n; i++) {
		double angle = 1.*i*spins / (n - 1);
		point[i * 3] = cos(angle);
		point[i * 3 + 1] = sin(angle);
		point[i * 3 + 2] = z_per_radian * angle;
	}
}

extern "C" {
	__declspec(dllexport) void generateDummyLine(double*point, int n) {
		generateDummyLine_(point, n);
	}
}