#include "Bspline.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

#include "environment.hpp"

using namespace std;

int randomWithLimits(int lower, int upper) {
  return (rand() % (upper - lower + 1)) + lower;
}


Bspline::Bspline(int size, int resolution, int order){
  this->size = size+2;
  this->dimensions = 3;
  this->order = order;
  this->resolution = resolution;
  path.resize(size+2);
  interpolation.resize((size + 2)*resolution);
  //makeRandomPath();
  makePlannedPath();
}

void Bspline::makePlannedPath() {
  PathBase plan = generatePath(size);
  for (int i = 0; i < size; i++) {
    path[i].x = plan.knot[i].p[0];
    path[i].y = plan.knot[i].p[1];
    path[i].z = plan.knot[i].p[2];
  }
}

void Bspline::makeRandomPath() {
  int length = size - 2;
  for (int i = 0; i < length; i++) {
    path[i].x = randomWithLimits(i * 10, i * 10 + 10);
    path[i].y = randomWithLimits(i * 10, i * 10 + 10);
    path[i].z = randomWithLimits(i * 10, i * 10 + 10);
  }
  for (int i = 0; i < 2; i++) {
    path[i + length].x = path[length - 1].x;
    path[i + length].y = path[length - 1].y;
    path[i + length].z = path[length - 1].z;
  }
}
int Bspline::getPathSize() const {
  return path.size();
}
int Bspline::getInterpolationSize() const {
  return interpolation.size();
}
vector<Knot> Bspline::getPath() const {
  return path;
}
vector<Knot> Bspline::getInterpolation() const {
  return interpolation;
}
double Bspline::calculateBasisFunction(double i, double k, double point) {
  if (k == 0) {
    if (i <= point && point < i + 1)
      return 1;
    return 0;
  }
  double result = ((point - i) / k)*calculateBasisFunction(i, k - 1, point) + ((i + k + 1 - point) / k)*calculateBasisFunction(i + 1, k - 1, point);
  return result;
}
void Bspline::findPoint(int t) {
  for (int i = 0; i < size; i++) {
    double basisFunction = calculateBasisFunction(i, order, t/double(resolution));
    interpolation[t] += (path[i] * basisFunction);
  }
}
void Bspline::savePathToFile(string filename) {
  ofstream outfile(&filename[0]);

  if (!outfile) { // evt. if(inputFile.fail())
    cout << "Error opening file(s)\n";
    return;
  }
  outfile << this->getPath();

  outfile.close();
}
void Bspline::saveInterpolationToFile(string filename) {
  ofstream outfile(&filename[0]);

  if (!outfile) { // evt. if(inputFile.fail())
    cout << "Error opening file(s)\n";
    return;
  }
  outfile << this->getInterpolation();

  outfile.close();
}


void Bspline::makeBspline() {
  for (int t = 0; t < size*resolution; t++) {
    findPoint(t);
  }
}

ostream& operator<<(ostream& os, const vector<Knot>& rhs) {
  for (int i = 0; i < rhs.size(); i++) {
    Knot knot = rhs[i];
    os << knot.x << " " << knot.y << " " << knot.z << endl;
  }
  return os;
}
Knot operator*(const Knot &lhs, double rhs) {
  Knot result;
  result.x = lhs.x*rhs;
  result.y = lhs.y*rhs;
  result.z = lhs.z*rhs;
  return result;
}
Knot& operator+=(Knot& lhs, const Knot& rhs){
  lhs.x += rhs.x;
  lhs.y += rhs.y;
  lhs.z += rhs.z;
  return lhs;
}

void helperFunction(int knots, int resolution, int order, double* interpolationArray, int arraySize) {

}

extern "C" {
#ifndef __linux
  __declspec(dllexport)
#endif

  void bsplineArray(int knots, int resolution, int order, double* interpolationArray, int arraySize) {
    int dimensions = 3;
    Bspline bspline(knots, resolution, order);
    bspline.makeBspline();
    vector<Knot> interpolation = bspline.getInterpolation();

    int maxSize = min(int(interpolation.size()), arraySize/3);
    for (int i = 0; i < maxSize; i++) {
      interpolationArray[i*dimensions] = interpolation[i].x;
      interpolationArray[i*dimensions + 1] = interpolation[i].y;
      interpolationArray[i*dimensions + 2] = interpolation[i].z;
    }
  }
}
