#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>

const int RES = 64;
const int dilate_radius = 1;


#include "path_planning.hpp"
#include <stdio.h>
#include <string.h>
#include <map>
#include <time.h>

using namespace std;


int volume[RES][RES][RES];
float dist[RES*RES*RES];
int done[RES][RES][RES];


void rasterVolume(int volume[RES][RES][RES], float p[3][3]) {
  float(&a)[3] = p[0];
  float(&b)[3] = p[1];
  float(&c)[3] = p[2];
  int axis;
  {
    float ax = a[0]-c[0], ay = a[1]-c[1], az = a[2]-c[2];
    float bx = b[0]-c[0], by = b[1]-c[1], bz = b[2]-c[2];
    float nx = fabs(ay*bz-az*by), ny = fabs(az*bx-ax*bz), nz = fabs(ax*by-ay*bx);
    if (nx > ny && nx > nz) {
      axis = 0;
      swap(p[0][0], p[0][2]);
      swap(p[1][0], p[1][2]);
      swap(p[2][0], p[2][2]);
    } else if (ny > nz) {
      axis = 1;
      swap(p[0][1], p[0][2]);
      swap(p[1][1], p[1][2]);
      swap(p[2][1], p[2][2]);
    } else
      axis = 2;
  }
  int sx = max(int(ceil(min(min(p[0][0], p[1][0]), p[2][0]))), 0);
  int sy = max(int(ceil(min(min(p[0][1], p[1][1]), p[2][1]))), 0);
  int ex = min(int(floor(max(max(p[0][0], p[1][0]), p[2][0])))+1, RES);
  int ey = min(int(floor(max(max(p[0][1], p[1][1]), p[2][1])))+1, RES);

  float ax = b[1]-a[1];
  float ay = a[0]-b[0];
  float az = a[0]*b[1]-a[1]*b[0];
  float bx = c[1]-b[1];
  float by = b[0]-c[0];
  float bz = b[0]*c[1]-b[1]*c[0];
  float cx = a[1]-c[1];
  float cy = c[0]-a[0];
  float cz = c[0]*a[1]-c[1]*a[0];

  float nx, ny, nz;
  {
    float ax = a[0]-c[0], ay = a[1]-c[1], az = a[2]-c[2];
    float bx = b[0]-c[0], by = b[1]-c[1], bz = b[2]-c[2];
    float mcx = az*by-ay*bz, mcy = ax*bz-az*bx, cz = ax*by-ay*bx;
    float idet = 1.f/cz;
    nx = mcx*idet;
    ny = mcy*idet;
    nz = c[2]-c[0]*nx-c[1]*ny+0.5f;
  }
  for (int j = sy; j < ey; j++)
    for (int i = sx; i < ex; i++) {
      int la = ax*i+ay*j < az;
      int lb = bx*i+by*j < bz;
      int lc = cx*i+cy*j < cz;
      if (la == lb && la == lc) {
	float z = nx*i+ny*j+nz;
	int k0 = floor(z), k1 = ceil(z);
	if (k0 >= 0 && k1 < RES) {
	  if (axis == 0)
	    volume[i][j][k0] = 1, volume[i][j][k1] = 1;
	  else if (axis == 1)
	    volume[j][k0][i] = 1, volume[j][k1][i] = 1;
	  else
	    volume[k0][j][i] = 1, volume[k1][j][i] = 1;
	}
      }
    }
}



void distanceTransform2(float*dist2, int w, int stride) {
  static float cache[2000];
  static int v[2000];
  static float z[2000];
  for (int i = 0; i < w; i++) cache[i] = dist2[i*stride];
  int k = 0;
  v[0] = 0;
  z[0] =-1e9;
  z[1] = 1e9;
  for (int q = 1; q < w; q++) {
    float s = ((cache[q]+q*q)-(cache[v[k]]+v[k]*v[k]))*0.5f/(q-v[k]);
    if (s < z[k]) {
      k--;
      q--;
    } else {
      k++;
      v[k] = q;
      z[k] = s;
      z[k+1] = 1e9;
    }
  }
  k = 0;
  for (int q = 0; q < w; q++) {
    while (z[k+1] < q) k++;
    dist2[q*stride] = (q-v[k])*(q-v[k])+cache[v[k]];
    //cout << k << ' ' << v[k] << ' ' << dist2[q*stride] << endl;
  }
}


void distanceTransform(int volume[RES][RES][RES], float r) {
  for (int k = 0; k < RES; k++)
    for (int j = 0; j < RES; j++)
      for (int i = 0; i < RES; i++)
	dist[i+j*RES+k*RES*RES] = volume[k][j][i]==0 ? 1e9 : 0;
  for (int j = 0; j < RES; j++)
    for (int i = 0; i < RES; i++)
    distanceTransform2(dist+i+j*RES, RES, RES*RES);
  for (int j = 0; j < RES; j++)
    for (int i = 0; i < RES; i++)
    distanceTransform2(dist+i+j*RES*RES, RES, RES);
  for (int j = 0; j < RES; j++)
    for (int i = 0; i < RES; i++)
    distanceTransform2(dist+i*RES+j*RES*RES, RES, 1);
  for (int k = 0; k < RES; k++)
    for (int j = 0; j < RES; j++)
      for (int i = 0; i < RES; i++)
	volume[k][j][i] = dist[i+j*RES+k*RES*RES] <= r*r;
}

void fill(int(&volume)[RES][RES][RES]) {
  for (int i = 0; i < RES; i++)
    for (int j = 0; j < RES; j++)
      for (int k = 0; k < RES; k++)
	done[i][j][k] = -volume[i][j][k]*100;

  queue<Jpos> q;
  q.push(Jpos(0, 0, 0));
  int c = 0;
  while (q.size()) {
    int i = q.front().x;
    int j = q.front().y;
    int k = q.front().z;
    q.pop();
    done[i][j][k]++;
    if (k         && !done[i][j][k-1]++) q.push(Jpos(i, j, k-1));
    if (k < RES-1 && !done[i][j][k+1]++) q.push(Jpos(i, j, k+1));
    if (j         && !done[i][j-1][k]++) q.push(Jpos(i, j-1, k));
    if (j < RES-1 && !done[i][j+1][k]++) q.push(Jpos(i, j+1, k));
    if (i         && !done[i-1][j][k]++) q.push(Jpos(i-1, j, k));
    if (i < RES-1 && !done[i+1][j][k]++) q.push(Jpos(i+1, j, k));
  }
  for (int i = 0; i < RES; i++)
    for (int j = 0; j < RES; j++)
      for (int k = 0; k < RES; k++)
	volume[i][j][k] = done[i][j][k] <= 0;
}


struct Mesh {
  float (*vertex)[3];
  int (*triangle)[3];
  int vertices, triangles;
  Mesh() {
    vertex = new float[3][3];
    triangle = new int[1][3];
    float vert[] = {0, -1, 3,  1, 1, 3, -1, 1, 2};
    for (int i = 0; i < 3; i++) {
      triangle[0][i] = i;
      for (int j = 0; j < 3; j++)
	vertex[i][j] = vert[i*3+j];
    }
    vertices = 3;
    triangles = 1;
  }

  Mesh(const char*filename) {
    vertices = 0;
    triangles = 0;
    vertex = NULL;
    FILE*fp = fopen(filename, "r");
    if (!fp) {
      cout << "Could not open file " << filename << endl;
      return;
    }
    char str[100];
    int tmp = fscanf(fp, "%s", str);
    if (strcmp(str, "ply") != 0) {
      cout << "Can't open non-ply format" << endl;
      fclose(fp);
      return;
    }
    int properties = 0, vertexing = 0;
    while (1) {
      int tmp = fscanf(fp, "%s", str);
      if (strcmp(str, "vertex") == 0) {
	int tmp = fscanf(fp, "%d", &vertices);
	vertexing = 1;
      } else if (strcmp(str, "face") == 0) {
	int tmp = fscanf(fp, "%d", &triangles);
	vertexing = 0;
      } else if (strcmp(str, "end_header") == 0) {
	break;
      } else if (strcmp(str, "property") == 0 && vertexing) properties++;
    }
    vertex = new float[vertices][3];
    triangle = new int[triangles][3];
    float ftmp;
    for (int i = 0; i < vertices; i++) {
      int tmp = fscanf(fp, "%f%f%f", vertex[i], vertex[i]+1, vertex[i]+2);
      for (int j = 3; j < properties; j++) int tmp = fscanf(fp, "%f", &ftmp);
    }
    for (int i = 0; i < triangles; i++) {
      int corners;
      {int tmp = fscanf(fp, "%d", &corners);}
      if (corners != 3) {
	cout << "Can't open non-triangle " << corners << endl;
	fclose(fp);
	return;
	//continue;
      }
      int tmp = fscanf(fp, "%d%d%d", triangle[i], triangle[i]+1, triangle[i]+2);
    }
    fclose(fp);
  }
  void rescale(double W, double border = 0) {
    float minv[3] = {1e9, 1e9, 1e9};
    float maxv[3] = {-1e9, -1e9, -1e9};
    for (int i = 0; i < vertices; i++) {
      for (int j = 0; j < 3; j++) {
	minv[j] = min(minv[j], vertex[i][j]);
	maxv[j] = max(maxv[j], vertex[i][j]);
      }
    }
    cout << "Bounding box" << endl;
    for (int i = 0; i < 3; i++) cout << minv[i] << ' ' << maxv[i] << endl;
    float neww = W-border*2;
    float iw[3];
    for (int i = 0; i < 3; i++) iw[i] = neww/(maxv[i]-minv[i]);
    for (int i = 0; i < vertices; i++) {
      for (int k = 0; k < 3; k++)
	vertex[i][k] = (vertex[i][k]-minv[k])*iw[k]+border;
    }
  }
  void render(int volume[RES][RES][RES]) {
#pragma omp parallel for
    for (int i = 0; i < triangles; i++) {
      float p[3][3];
      for (int j = 0; j < 3; j++) {
	float*x = vertex[triangle[i][j]];
	for (int k = 0; k < 3; k++)
	  p[j][k] = x[k];
      }
      rasterVolume(volume, p);
    }
  }
  Mesh(int volume[RES][RES][RES]) {
    map<int, int> indmap;
    for (int l = 0; l < 2; l++) {
      vertices = 0;
      for (int i = 1; i < RES; i++)
	for (int j = 1; j < RES; j++)
	  for (int k = 1; k < RES; k++) {
	    int base = volume[i][j][k];
	    if (base != volume[i  ][j-1][k  ] ||
		base != volume[i  ][j  ][k-1] ||
		base != volume[i  ][j-1][k-1] ||
		base != volume[i-1][j  ][k  ] ||
		base != volume[i-1][j-1][k  ] ||
		base != volume[i-1][j  ][k-1] ||
		base != volume[i-1][j-1][k-1]) {
	      if (l) {
		vertex[vertices][0] = k-1;
		vertex[vertices][1] = j-1;
		vertex[vertices][2] = i-1;
		indmap[i*RES*RES+j*RES+k] = vertices;
	      }
	      vertices++;
	    }
	  }
      if (!l) vertex = new float[vertices][3];
    }
#define addSquare()			    \
    triangle[triangles][0] = a;		    \
    triangle[triangles][1] = b;		    \
    triangle[triangles][2] = c;		      \
    triangle[triangles+1][0] = d;	      \
    triangle[triangles+1][1] = c;	      \
    triangle[triangles+1][2] = b;

    for (int l = 0; l < 2; l++) {
      triangles = 0;
      for (int i = 1; i < RES; i++)
	for (int j = 1; j < RES; j++)
	  for (int k = 1; k < RES; k++) {
	    int base = volume[i][j][k];
	    int ind = i*RES*RES+j*RES+k;
	    if (base != volume[i-1][j][k]) {
	      if (l) {
		int a = indmap[ind], b = indmap[ind+1], c = indmap[ind+RES], d = indmap[ind+RES+1];
		addSquare();
	      }
	      triangles += 2;
	    }
	    if (base != volume[i][j-1][k]) {
	      if (l) {
		int a = indmap[ind], b = indmap[ind+1], c = indmap[ind+RES*RES], d = indmap[ind+RES*RES+1];
		addSquare();
	      }
	      triangles += 2;
	    }
	    if (base != volume[i][j][k-1]) {
	      if (l) {
		int a = indmap[ind], b = indmap[ind+RES], c = indmap[ind+RES*RES], d = indmap[ind+RES*RES+RES];
		addSquare();
	      }
	      triangles += 2;
	    }
	  }
      if (!l) triangle = new int[triangles][3];
    }

  }
  ~Mesh() {
    if (vertex) {
      delete[]vertex;
      delete[]triangle;
    }
  }
};


void init() {
  Mesh environment("terrain.ply");

  char s[100] = "Could not read";
  FILE*ifp = fopen("Johan_log.txt", "r");
  if (ifp) {
    fscanf(ifp, "%s", s);
    fclose(ifp);
  }

  FILE*fp = fopen("Johan_log.txt", "w");
  fprintf(fp, "%sFound file if vertices is large, vertices %d\n", s, environment.vertices);
  fclose(fp);

  //Mesh environment("Tree.ply");
  environment.rescale(RES, 5);

  printf("Environment has %d vertices and %d faces\n", environment.vertices, environment.triangles);

  environment.render(volume);

  //volume[RES/2][RES/2][RES/2] = 1;
  distanceTransform(volume, dilate_radius);
  fill(volume);
}

Jpos getPos(int(&volume)[RES][RES][RES], int border = 10) {
  Jpos p;
  do {
    p.x = rand()%(RES-border*2)+border;
    p.y = rand()%(RES-border*2)+border;
    p.z = rand()%(RES-border*2)+border;
  } while (volume[p.z][p.y][p.x]);
  return p;
}

int toggle_planner = 2; // K_5 for BFS, K_6 for Dijkstra, K_7 for A*

PathBase generatePath(int len) {
  srand(time(NULL));

  static int inited = 0;
  if (!inited++) init();

  PathBase path;
  Jpos startpos = getPos(volume);

  while (len >= int(path.knot.size())-3) {
    //path.knot.clear();
    int minlen = path.knot.size()+10;
    Jpos b;
    while (path.knot.size() < minlen) {
      b = getPos(volume);

      int size0 = path.knot.size();
      //timeval t1, t2;
      //gettimeofday(&t1, NULL);
      if (toggle_planner == 0)
	findPathBFS(volume, startpos, b, path);
      else if (toggle_planner == 1)
	findPathDijkstra(volume, startpos, b, path);
      else if (toggle_planner == 2)
	findPathAstar(volume, startpos, b, path);
      //gettimeofday(&t2, NULL);
      //double deltime = (t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*.000001)*1000;
      //cout << "Planned a path of length " << path.knot.size()-size0 << " in " << deltime << " ms" << endl;
      startpos = b;
    }
    //cout << path.knot.size() << endl;

    //gettimeofday(&ta, NULL);
    //deltime = 0.1;
  }
  //cout << path.knot.size() << endl;
  double border = 5;
  for (int i = 0; i < path.knot.size(); i++) {
    for (int j = 0; j < 3; j++) {
      path.knot[i].p[j] = (path.knot[i].p[j]-border)/(RES-border*2);
    }
  }

  return path;
}
