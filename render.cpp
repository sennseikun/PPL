//g++ render.cpp -lgdi32 -lwinmm -mwindows -O3 -fopenmp -march=native -g && a.exe
//g++ render.cpp -lX11 -O3 -fopenmp -march=native -g && ./a.out

#include "environment.hpp"

#include "bat/bat.hpp"
#include "bat/trackball.hpp"
#include <string.h>
#include <map>
#include <vector>
#include <queue>
#include <stack>


using namespace std;

int toggle_grid  = 0; // K_SPACE
int toggle_snake = 0; // K_TAB
int toggle_water = 1; // K_1
int toggle_SSAO  = 1; // K_2
int toggle_knots = 0; // K_3

void pixelShader(Surface&sf, float*&zb, int&i, int&j, float&nx, float&ny, float&nz, uint col) {
  float iz = nx*i+ny*j+nz;
  if (iz > zb[i+j*sf.w]) {
    zb[i+j*sf.w] = iz;
    sf.pixels[i+j*sf.w] = col;
  }
}

void drawTriangleSlow(Surface&sf, float*&zb, float p[3][3], uint&col) {
  if (p[0][2] < 0 || p[1][2] < 0 || p[2][2] < 0) return;
  int sx = max(int(ceil(min(min(p[0][0], p[1][0]), p[2][0]))), 0);
  int sy = max(int(ceil(min(min(p[0][1], p[1][1]), p[2][1]))), 0);
  int ex = min(int(floor(max(max(p[0][0], p[1][0]), p[2][0])))+1, sf.w);
  int ey = min(int(floor(max(max(p[0][1], p[1][1]), p[2][1])))+1, sf.h);

  float(&a)[3] = p[0];
  float(&b)[3] = p[1];
  float(&c)[3] = p[2];
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
    nz = c[2]-c[0]*nx-c[1]*ny;
  }
  for (int j = sy; j < ey; j++)
    for (int i = sx; i < ex; i++) {
      int la = ax*i+ay*j < az;
      int lb = bx*i+by*j < bz;
      int lc = cx*i+cy*j < cz;
      if (la == lb and la == lc)
	pixelShader(sf, zb, i, j, nx, ny, nz, col);
    }
}

void drawSortedTriangle(Surface&sf, float*&zb, float(&a)[3], float(&b)[3], float(&c)[3], uint&col, int flip) {
  float ax = a[0]-c[0], ay = a[1]-c[1], az = a[2]-c[2];
  float bx = b[0]-c[0], by = b[1]-c[1], bz = b[2]-c[2];
  float mcx = az*by-ay*bz, mcy = ax*bz-az*bx, cz = ax*by-ay*bx;
  float idet = 1.f/cz;
  float nx = mcx*idet;
  float ny = mcy*idet;
  float nz = c[2]-c[0]*nx-c[1]*ny;
  //if ((idet > 0) ^ flip) return;
  if (idet < 0) {
    int sy = max(int(ceil(a[1])), 0);
    int my = max(min(int(floor(b[1]))+1, sf.h), 0);
    int ey = min(int(floor(c[1]))+1, sf.h);
    //cout << "A " << sy << ' ' << my << ' ' << ey << endl;
    float sdx0 = (b[0]-a[0])/(b[1]-a[1]);
    float sdx1 = (c[0]-b[0])/(c[1]-b[1]);
    float edx = (c[0]-a[0])/(c[1]-a[1]);
    for (int j = sy; j < my; j++) {
      float sx = max(int(ceil(a[0]+(j-a[1])*sdx0)), 0);
      float ex = min(int(floor(a[0]+(j-a[1])*edx+1)), sf.w);
      for (int i = sx; i < ex; i++) {
	pixelShader(sf, zb, i, j, nx, ny, nz, col);
      }
    }
    for (int j = my; j < ey; j++) {
      float sx = max(int(ceil(b[0]+(j-b[1])*sdx1)), 0);
      float ex = min(int(floor(a[0]+(j-a[1])*edx+1)), sf.w);
      for (int i = sx; i < ex; i++) {
	pixelShader(sf, zb, i, j, nx, ny, nz, col);
      }
    }
  } else {
    int sy = max(int(ceil(a[1])), 0);
    int my = max(min(int(floor(b[1]))+1, sf.h), 0);
    int ey = min(int(floor(c[1]))+1, sf.h);
    //cout << "B " << sy << ' ' << my << ' ' << ey << endl;
    float sdx = (c[0]-a[0])/(c[1]-a[1]);
    float edx0 = (b[0]-a[0])/(b[1]-a[1]);
    float edx1 = (c[0]-b[0])/(c[1]-b[1]);
    for (int j = sy; j < my; j++) {
      int sx = max(int(ceil(a[0]+(j-a[1])*sdx)), 0);
      int ex = min(int(floor(a[0]+(j-a[1])*edx0))+1, sf.w);
      for (int i = sx; i < ex; i++) {
	pixelShader(sf, zb, i, j, nx, ny, nz, col);
      }
    }
    for (int j = my; j < ey; j++) {
      int sx = max(int(ceil(a[0]+(j-a[1])*sdx)), 0);
      int ex = min(int(floor(b[0]+(j-b[1])*edx1))+1, sf.w);
      for (int i = sx; i < ex; i++) {
	pixelShader(sf, zb, i, j, nx, ny, nz, col);
      }
    }
  }
}

void drawTriangle(Surface&sf, float*&zb, float p[3][3], uint&col) {
  if (p[0][2] < 0 || p[1][2] < 0 || p[2][2] < 0) return;
  /*cout << endl;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      cout << p[i][j] << ' ';
    cout << endl;
  }
  cout << endl;*/

  if (p[0][1] < p[1][1] and p[0][1] < p[2][1]) {
    if (p[1][1] < p[2][1])
      drawSortedTriangle(sf, zb, p[0], p[1], p[2], col, 0);
    else
      drawSortedTriangle(sf, zb, p[0], p[2], p[1], col, 1);
  } else if (p[1][1] < p[2][1]) {
    if (p[0][1] < p[2][1])
      drawSortedTriangle(sf, zb, p[1], p[0], p[2], col, 1);
    else
      drawSortedTriangle(sf, zb, p[1], p[2], p[0], col, 0);
  } else {
    if (p[0][1] < p[1][1])
      drawSortedTriangle(sf, zb, p[2], p[0], p[1], col, 0);
    else
      drawSortedTriangle(sf, zb, p[2], p[1], p[0], col, 1);
  }
}


void drawPoint(Surface&sf, float*&zbuf, float(&a)[3], uint col) {
  if (a[2] < 0) return;
  int x = a[0]+.5f, y = a[1]+0.5f;
  int sx = max(x-1, 0), sy = max(y-1, 0);
  int ex = min(x+1, sf.w), ey = min(y+1, sf.h);
  for (int j = sy; j < ey; j++)
    for (int i = sx; i < ex; i++) {
      if (a[2] > zbuf[i+j*sf.w]) {
	zbuf[i+j*sf.w] = a[2];
	sf.pixels[i+j*sf.w] = col;
      }
    }
}

void drawLine(Surface&sf, float*&zbuf, float(&a)[3], float(&b)[3], uint col) {
  if (a[0] == b[0] && a[1] == b[1]) return;
  if (!(a[2] > 0 || b[2] > 0)) return;
  if (a[0] < 2 && b[0] < 2) return;
  if (a[1] < 2 && b[1] < 2) return;
  if (a[0] > sf.w-2 && b[0] > sf.w-2) return;
  if (a[1] > sf.h-2 && b[1] > sf.h-2) return;
  if (a[2] < 0) swap(a, b);

  float dx = b[0]-a[0], dy = b[1]-a[1], dz = b[2]-a[2];
  if (dx == 0 || dy == 0) return;
  int flip = (b[2] < 0);
  if (flip) dx = -dx, dy = -dy, dz = -dz;
  float idx = 1.f/dx, idy = 1.f/dy, idz = 1.f/dz;

  float mint = 0, maxt = flip ? 1e9 : 1;

  float minz = min(fabs(dz*idx), fabs(dz*idy))*2;
  if (dx > 0) mint = max(mint, (2-a[0])*idx);
  if (dx < 0) mint = max(mint, (sf.w-2-a[0])*idx);
  if (dy > 0) mint = max(mint, (2-a[1])*idy);
  if (dy < 0) mint = max(mint, (sf.h-2-a[1])*idy);
  if (dz > 0) mint = max(mint, (minz-a[2])*idz);
  if (dx < 0) maxt = min(maxt, (2-a[0])*idx);
  if (dx > 0) maxt = min(maxt, (sf.w-2-a[0])*idx);
  if (dy < 0) maxt = min(maxt, (2-a[1])*idy);
  if (dy > 0) maxt = min(maxt, (sf.h-2-a[1])*idy);
  if (dz < 0) maxt = min(maxt, (minz-a[2])*idz);
  //cout << a[0] << ' ' << a[1] << ' ' << b[0] << ' ' << b[1] << endl;
  //cout << mint << ' ' << maxt << endl;
  maxt -= mint;
  if (maxt < 0 || maxt != maxt) return;
  a[0] = a[0]+mint*dx;
  a[1] = a[1]+mint*dy;
  a[2] = a[2]+mint*dz;
  b[0] = a[0]+maxt*dx;
  b[1] = a[1]+maxt*dy;
  b[2] = a[2]+maxt*dz;

  if (a[0] != a[0] || b[0] != b[0]) return;
  if (a[1] != a[1] || b[1] != b[1]) return;
  if (a[2] != a[2] || b[2] != b[2]) return;


  if (fabs(dx) > fabs(dy)) {
    float nx = dz*idx, nz = a[2]-nx*a[0];
    int sx = a[0], ex = b[0];
    if (sx > ex) swap(sx, ex);
    for (int i = sx; i <= ex; i++) {
      int j = a[1]+dy*idx*(i-a[0]);
      float iz = nx*i+nz;
      if (i < 1 || j < 1 || i >= sf.w-1 || j >= sf.h-1 || iz < 0) {
	//cout << "A " << i << ' ' << j << ' ' << iz << endl;
	//cout << a[0] << ' ' << a[1] << ' ' << b[0] << ' ' << b[1] << endl;
	//exit(0);
      } else
      if (0 || iz > zbuf[i+j*sf.w]) {
	int k = i+j*sf.w, &w = sf.w;
	sf.pixels[k] = col;
	sf.pixels[k+1] = col;
	sf.pixels[k-1] = col;
	sf.pixels[k+w] = col;
	sf.pixels[k-w] = col;
	sf.pixels[k+w+1] = col;
	sf.pixels[k+w-1] = col;
	sf.pixels[k-w+1] = col;
	sf.pixels[k-w-1] = col;
	zbuf[k] = iz;
	zbuf[k+1] = iz;
	zbuf[k-1] = iz;
	zbuf[k+w] = iz;
	zbuf[k-w] = iz;
	zbuf[k+w+1] = iz;
	zbuf[k+w-1] = iz;
	zbuf[k-w+1] = iz;
	zbuf[k-w-1] = iz;
      }
    }
  } else {
    float ny = dz*idy, nz = a[2]-ny*a[1];
    int sy = a[1], ey = b[1];
    if (sy > ey) swap(sy, ey);
    for (int j = sy; j <= ey; j++) {
      int i = a[0]+dx*idy*(j-a[1])+0.5f;
      float iz = ny*j+nz;
      if (i < 1 || j < 1 || i >= sf.w-1 || j >= sf.h-1 || iz < 0) {
	//cout << "B " << i << ' ' << j << ' ' << iz << endl;
	//cout << a[0] << ' ' << a[1] << ' ' << b[0] << ' ' << b[1] << endl;
	//exit(0);
      } else
	if (0 || iz > zbuf[i+j*sf.w]) {
	int k = i+j*sf.w, &w = sf.w;
	sf.pixels[k] = col;
	sf.pixels[k+1] = col;
	sf.pixels[k-1] = col;
	sf.pixels[k+w] = col;
	sf.pixels[k-w] = col;
	sf.pixels[k+w+1] = col;
	sf.pixels[k+w-1] = col;
	sf.pixels[k-w+1] = col;
	sf.pixels[k-w-1] = col;
	zbuf[k] = iz;
	zbuf[k+1] = iz;
	zbuf[k-1] = iz;
	zbuf[k+w] = iz;
	zbuf[k-w] = iz;
	zbuf[k+w+1] = iz;
	zbuf[k+w-1] = iz;
	zbuf[k-w+1] = iz;
	zbuf[k-w-1] = iz;
      }
    }
  }
}


struct Path : PathBase {
  Path(PathBase pb) {
    knot = pb.knot;
  }
  Path() {
    /*
    for (int i = 0; i < 500; i++) {
      JKnot p;
      for (int k = 0; k < 3; k++)
	p.p[k] = rand()%RES;
      knot.push_back(p);
      }*/
  }
  void renderKnots(Surface&sf, float*zbuf, float T[3][4]) {
    for (int i = 0; i < knot.size(); i++) {
      float p[3];
      float(&x)[3] = knot[i].p;
      for (int k = 0; k < 3; k++)
	p[k] = T[k][0]*x[0]+T[k][1]*x[1]+T[k][2]*x[2]+T[k][3];
      float iz = 1.f/p[2];
      p[0] *= iz;
      p[1] *= iz;
      p[2] = iz;
      drawPoint(sf, zbuf, p, 0xffff00);
    }
  }
  void render(Surface&sf, float*zbuf, float T[3][4], float start = 0, float time = 0) {
    vector<JKnot> path;
    for (int i = 100*start; i < 100*time; i++) {
      path.push_back(getPoint(i/100.f));
    }
    for (int i = 1; i < path.size(); i++) {
      float p[2][3];
      for (int j = 0; j < 2; j++) {
	float(&x)[3] = path[i+j-1].p;
	for (int k = 0; k < 3; k++)
	  p[j][k] = T[k][0]*x[0]+T[k][1]*x[1]+T[k][2]*x[2]+T[k][3];
	float iz = 1.f/p[j][2];
	p[j][0] *= iz;
	p[j][1] *= iz;
	p[j][2] = iz;
      }
      drawLine(sf, zbuf, p[0], p[1], 0x00ff00);
    }
  }
  JKnot getPoint(float t) {
    if (t >= int(knot.size())-3) t = int(knot.size())-3-1e-6;
    if (t < 0) t = 0;
    float u = t-floor(t), u1 = 1.f-u;
    float w[4] = {u1*u1*u1*(1.f/6.f),
		  ((3.f*u-6.f)*u*u+4.f)*(1.f/6.f),
		  (((-3.f*u+3.f)*u+3.f)*u+1.f)*(1.f/6.f),
		  u*u*u*(1.f/6.f)};
    int ti = int(t);
    JKnot r;
    r.p[0] = r.p[1] = r.p[2] = 0;
    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < 4; i++) {
	r.p[j] += w[i]*knot[ti+i].p[j];
      }
    }
    return r;
  }
  JKnot getVelocity(float t) {
    if (t >= int(knot.size())-3) t = int(knot.size())-3-1e-6;
    if (t < 0) t = 0;
    float u = t-floor(t), u1 = 1.f-u;
    float w[4] = {-u1*u1*0.5f,
		  ((3.f*u-4.f)*u)*0.5f,
		  ((-3.f*u+2.f)*u+1.f)*0.5f,
		  u*u*0.5f};
    int ti = int(t);
    JKnot r;
    r.p[0] = r.p[1] = r.p[2] = 0;
    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < 4; i++) {
	r.p[j] += w[i]*knot[ti+i].p[j];
      }
    }
    return r;
  }
  void getPose(vec3&up, vec3&viewdir, vec3&pos, float t) {
    if (t >= int(knot.size())-3) t = int(knot.size())-3-1e-6;
    JKnot p = getPoint(t);
    pos.x = p.p[0], pos.y = p.p[1], pos.z = p.p[2];
    JKnot v = getVelocity(t);
    viewdir.x = v.p[0], viewdir.y = v.p[1], viewdir.z = v.p[2];
    viewdir /= sqrt(viewdir*viewdir);
    vec3 side = vec3(0, 0, -1)^viewdir;
    up = viewdir^side;
    up /= sqrt(up*up);
  }
};

void distanceTransformSurface(Surface&sf) {
  static float dist[960*540];
  for (int i = 0; i < sf.w*sf.h; i++)
    dist[i] = sf.pixels[i]==0 ? 1e9 : 0;
  for (int i = 0; i < sf.h; i++)
    distanceTransform2(dist+i*sf.w, sf.w, 1);
  for (int i = 0; i < sf.w; i++)
    distanceTransform2(dist+i, sf.h, sf.w);
  for (int i = 0; i < sf.w*sf.h; i++) {
    sf.pixels[i] = int(min(sqrtf(dist[i]), 255.f))*0x10101;
  }
}




uint flat(float(&a)[3], float(&b)[3], float(&c)[3]) {

  float ax = a[0]-c[0], ay = a[1]-c[1], az = a[2]-c[2];
  float bx = b[0]-c[0], by = b[1]-c[1], bz = b[2]-c[2];
  float nx = ay*bz-az*by, ny = az*bx-ax*bz, nz = ax*by-ay*bx;
  return max(int(255*(fabsf(nx*.2)+fabs(ny*.3)+fabs(nz*.5))/sqrtf(nx*nx+ny*ny+nz*nz)), 1)*0x10101;
}
void renderMesh(Mesh&mesh, Surface&sf, float*zbuf, float T[3][4]) {
#pragma omp parallel for
  for (int i = 0; i < mesh.triangles; i++) {
    float p[3][3];
    uint col = 0xffffff;
    if (!toggle_SSAO)
      col = flat(mesh.vertex[mesh.triangle[i][0]], mesh.vertex[mesh.triangle[i][1]], mesh.vertex[mesh.triangle[i][2]]);
    for (int j = 0; j < 3; j++) {
      float*x = mesh.vertex[mesh.triangle[i][j]];
      for (int k = 0; k < 3; k++)
	p[j][k] = T[k][0]*x[0]+T[k][1]*x[1]+T[k][2]*x[2]+T[k][3];
      float iz = 1.f/p[j][2];
      p[j][0] *= iz;
      p[j][1] *= iz;
      p[j][2] = iz;
    }
    drawTriangle(sf, zbuf, p, col);
  }
}


const int w = 1000, h = 700;
const float focal = w, centerx = w/2, centery = h/2;

float myrandom() {
  return rand()*(1.f/RAND_MAX);
}

void SSAO(Surface&sf, float*zbuf) {
  const int spp = 8, blurw = 4;
  float r = 1./100;
  static int inited = 0;
  static float off[spp*blurw*blurw][3];
  if (!inited++) {
    for (int i = 0; i < spp*blurw*blurw; i++) {
      float x, y, z;
      do {
	x = myrandom()*2-1;
	y = myrandom()*2-1;
	z = myrandom()*2-1;
      } while (x*x+y*y+z*z > 1);
      float len = x*x+y*y+z*z;
      x *= len, y *= len, z *= len;
      off[i][0] = (x*focal+z*centerx)*r;
      off[i][1] = (y*focal+z*centery)*r;
      off[i][2] = z*r;
    }
  }
  timeval ta, tb;
  gettimeofday(&ta, NULL);

  static int sum[w*h];
  for (int j = 0; j < sf.h; j++) {
#pragma omp parallel for
    for (int i = 0; i < sf.w; i++) {
      float iz = zbuf[i+j*sf.w];

      int c = 0;
      if (iz) {
	float fi = i, fj = j;
	int base = (i%blurw+j%blurw*blurw)*spp;
	for (int k = 0; k < spp; k++) {
	  float(&d)[3] = off[base+k];
	  float rx = fi+d[0]*iz, ry = fj+d[1]*iz, rz = 1+d[2]*iz, riz = 1.f/rz;
	  float ix = rx*riz, iy = ry*riz;
	  c += (ix >= 0 && iy >= 0 && ix < sf.w && iy < sf.h && rz > 0 && zbuf[int(ix)+int(iy)*sf.w]*rz < iz);
	}
      }
      sum[i+j*sf.w] = c;
    }
  }

  for (int j = 0; j < sf.h; j++) {
    for (int i = 0; i < sf.w; i++) {
      int k = i+j*sf.w;
      sum[k] += (i?sum[k-1]:0)+(j?sum[k-sf.w]:0)-(i&&j?sum[k-sf.w-1]:0);
    }
  }
  for (int j = 0; j < sf.h; j++) {
    for (int i = 0; i < sf.w; i++) {
      int ex = min(i+blurw/2, sf.w-1), sx = max(i-blurw/2, 0);
      int ey = min(j+blurw/2, sf.h-1), sy = max(j-blurw/2, 0);
      int c = sum[ex+ey*sf.w]+sum[sx+sy*sf.w]-sum[sx+ey*sf.w]-sum[ex+sy*sf.w];
      sf.pixels[i+j*sf.w] = ((sf.pixels[i+j*sf.w]&0xff)*c>>7)*0x10101;
    }
  }
  gettimeofday(&tb, NULL);
  double deltime = (tb.tv_sec-ta.tv_sec+(tb.tv_usec-ta.tv_usec)*.000001)*1000;
  //cout << deltime << endl;
}

void addWater(Surface&sf, float*zbuf, float T[3][4]) {
  for (int j = 0; j < sf.h; j++) {
#pragma omp parallel for
    for (int i = 0; i < sf.w; i++) {
      float &px = T[0][3], &py = T[1][3], &pz = T[2][3];
      float dx = T[0][0]*i+T[0][1]*j+T[0][2], idx = 1.f/dx;
      float dy = T[1][0]*i+T[1][1]*j+T[1][2], idy = 1.f/dy;
      float dz = T[2][0]*i+T[2][1]*j+T[2][2], idz = 1.f/dz;
      float tx0 = -px*idx, tx1 = idx+tx0;
      float ty0 = -py*idy, ty1 = idy+ty0;
      float tz0 = -pz*idz, tz1 = idz+tz0;
      if (idx < 0) swap(tx0, tx1);
      if (idy < 0) swap(ty0, ty1);
      if (idz < 0) swap(tz0, tz1);
      float iz = zbuf[i+j*sf.w];
      float z = 1.f/iz;
      float in = max(max(tx0, ty0), max(tz0, 0.f)), out = min(min(tx1, ty1), min(tz1, z));
      float len = max(out-in, 0.f)+(out<z && in<out)*0.5;//*(RES/2);
      if (len==0) continue;
      float alpha = expf(-len*(1.5f));
      int r = sf.pixels[i+j*sf.w].r*alpha, g = sf.pixels[i+j*sf.w].g*alpha, b = sf.pixels[i+j*sf.w].b*alpha;
      int blue = (1-alpha)*255*.4;
      sf.pixels[i+j*sf.w] = Color(r+(blue>>2), g, b+blue);
    }
  }
}


int main() {
  srand(time(NULL));
  MyScreen screen(w, h);
  Surface sf(w, h);
  float zbuf[w*h];

  Clock myclock;
  MyTrackball track(screen);
  track.speed = 0.02;
  track.rspeed = .5;

  float P[4][4] = {{focal, 0, centerx, 0}, {0, focal, centery, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  float Pi[4][4] = {{1.f/focal, 0, -centerx/focal, 0}, {0, 1.f/focal, -centery/focal, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  float R[4][4], Ri[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      R[i][j] = Ri[i][j] = i==j;
  float T[3][4], Ti[3][4];

  track.pos.x =-0.5;
  track.pos.y = 0.5;
  track.pos.z = 0.5;
  viewdir = vec3(1, 0, 0);
  up = vec3(0, 0,-1);

  Mesh environment("terrain.ply");
  //Mesh environment("Tree.ply");
  environment.rescale(1);

  timeval ta, tb;
  gettimeofday(&ta, NULL);

  Jpos startpos = getPos(volume);

  float pointx = 0, pointy = 0, pointz = 0;

  Path path = generatePath(10000);
  Mesh grid(volume);
  grid.rescale(RES*1./(RES-10));//TODO
  printf("Grid mesh has %d vertices and %d faces\n", grid.vertices, grid.triangles);

  while (1) {
    gettimeofday(&tb, NULL);
    float mul = 5;
    double deltime = (tb.tv_sec-ta.tv_sec+(tb.tv_usec-ta.tv_usec)*.000001);

    if (toggle_snake)
      path.getPose(up, viewdir, track.pos, deltime*mul);

    vec3 side = up^viewdir;
    for (int i = 0; i < 3; i++) {
      R[0][i] = side[i];
      R[1][i] = up[i];
      R[2][i] = viewdir[i];
      R[i][3] = 0;
    }
    for (int i = 0; i < 3; i++)
      for (int j = 0;j  < 3; j++)
	R[i][3] -= R[i][j]*track.pos[j];

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 4; j++) {
	T[i][j] = 0;
	for (int k = 0; k < 4; k++)
	  T[i][j] += P[i][k]*R[k][j];
      }
    for (int i = 0; i < 3; i++) {
      Ri[i][0] = side[i];
      Ri[i][1] = up[i];
      Ri[i][2] = viewdir[i];
      Ri[i][3] = track.pos[i];
    }

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 4; j++) {
	Ti[i][j] = 0;
	for (int k = 0; k < 4; k++)
	  Ti[i][j] += Ri[i][k]*Pi[k][j];
      }



    while (screen.gotEvent()) {
      Event e = screen.getEvent();
      if (e.type == KeyPress) {
        if (e.key == K_ESCAPE) return 0;
	else if (e.key == K_SPACE) toggle_grid ^= 1;
	else if (e.key == K_TAB) toggle_snake ^= 1;
	else if (e.key == K_1) toggle_water ^= 1;
	else if (e.key == K_2) toggle_SSAO ^= 1;
	else if (e.key == K_3) toggle_knots ^= 1;
	else if (e.key == K_5) cout << "Switching to BFS" << endl, toggle_planner = 0;
	else if (e.key == K_6) cout << "Switching to Dijkstra" << endl, toggle_planner = 1;
	else if (e.key == K_7) cout << "Switching to A*" << endl, toggle_planner = 2;
      } else if (e.type == ButtonPress) {
	if (e.key == 3) {
	  int x, y;
	  screen.getCursorPos(x, y);
	  float iz = zbuf[x+y*sf.w];
	  if (iz == 0) cout << "Can't move to infinity" << endl;
	  else {
	    float z = 1.f/iz;
	    float &px = Ti[0][3], &py = Ti[1][3], &pz = Ti[2][3];
	    float dx = Ti[0][0]*x+Ti[0][1]*y+Ti[0][2];
	    float dy = Ti[1][0]*x+Ti[1][1]*y+Ti[1][2];
	    float dz = Ti[2][0]*x+Ti[2][1]*y+Ti[2][2];
	    px += dx*z, py += dy*z, pz += dz*z;
	    //cout << x << ' ' << y << endl;
	    if (px < 5 || py < 5 || pz < 0 || px > RES-5 || py > RES-5 || pz > RES-5) {
	      cout << "Pick a position closer to the center" << endl;
	    } else {
	      int ix = px+0.5f, iy = py+0.5f, iz = pz+0.5f;
	      //cout << ix << ' ' << iy << ' ' << iz << endl;
	      //cout << volume[iz][iy][ix] << endl;
	      //volume[iz][iy][ix] = 1;

	      JKnot knot;
	      knot.p[0] = ix;
	      knot.p[1] = iy;
	      knot.p[2] = iz;
	      path.knot.push_back(knot);
	    }
	  }
	}
      }
      track.handleEvent(e);
    }
    track.update();

    for (int i = 0; i < w*h; i++) sf.pixels[i] = 0, zbuf[i] = 0;
    if (toggle_grid)
      renderMesh(grid, sf, zbuf, T);
    else
      renderMesh(environment, sf, zbuf, T);

    if (toggle_SSAO)
      SSAO(sf, zbuf);

    path.render(sf, zbuf, T, deltime*mul+0.4, deltime*mul+10.4);
    if (toggle_knots)
      path.renderKnots(sf, zbuf, T);

    if (toggle_water)
      addWater(sf, zbuf, Ti);

    //distanceTransformSurface(sf);

    screen.putSurface(sf);

    myclock.tick(60);
    myclock.print();
  }
}
