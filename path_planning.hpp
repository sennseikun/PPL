#include <vector>
#include <queue>

using namespace std;

struct pos {
  int x, y, z;
  pos() {}
  pos(int x, int y, int z) : x(x), y(y), z(z) {}
};

struct Knot {
  float p[3];
};

struct PathBase {
  vector<Knot> knot;
};

void findPathBFS(int(&volume)[RES][RES][RES], pos start, pos end, PathBase&r) {
  static int dist[RES][RES][RES];
  for (int i = 0; i < RES; i++)
    for (int j = 0; j < RES; j++)
      for (int k = 0; k < RES; k++)
	dist[i][j][k] = volume[i][j][k] || i == 0 || j == 0 || k == 0 || i == RES-1 || j == RES-1 || k == RES-1 ? -1 : 1e9;

  int dx[26] = {};
  int dy[26] = {};
  int dz[26] = {};
  float len[26] = {};
  int c = 0;
  for (int i = -1; i < 2; i++)
    for (int j = -1; j < 2; j++)
      for (int k = -1; k < 2; k++) {
	if (i == 0 && j == 0 && k == 0) continue;
	dx[c] = i;
	dy[c] = j;
	dz[c] = k;
	c++;
      }
  queue<pos> q;
  dist[end.z][end.y][end.x] = 0;
  q.push(pos(end.z, end.y, end.x));

  int co = 0;
  while (q.size()) {
    int i = q.front().x;
    int j = q.front().y;
    int k = q.front().z;
    if (i == start.z && j == start.y && k == start.x) {
      //cout << "Found start!" << endl;
      break;
    }
    //if (co++%1000000 == 0) cout << co << endl;
    q.pop();
    int d = dist[i][j][k]+1;

    for (int l = 0; l < 26; l++) {
      if (dist[i+dx[l]][j+dy[l]][k+dz[l]] > d) {
	q.push(pos(i+dx[l], j+dy[l], k+dz[l]));
	dist[i+dx[l]][j+dy[l]][k+dz[l]] = d;
      }
    }
  }
  /*
    for (int i = 0; i < RES; i++)
    for (int j = 0; j < RES; j++)
    for (int k = 0; k < RES; k++)
    if (volume[i][j][k] == 0) {
    Knot knot;
    knot.p[0] = k-0.5f, knot.p[1] = j-0.5f, knot.p[2] = i-0.5f;
    r.knot.push_back(knot);
    }
  */
  pos p(start.z, start.y, start.x);
  Knot knot;
  for (int i = 0; i < 2; i++) {
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
  }
  while (1) {
    Knot knot;
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
    int&i = p.x;
    int&j = p.y;
    int&k = p.z;
    int d = dist[i][j][k];
    if (d == 1e9) {
      cout << "No path to victory!" << endl;
      exit(0);
    }
    //cout << i << ' ' << j << ' ' << k << ' ' << d << endl;

    if (d == 0) break;
    int mindist = 1e9, minl = 0;
    for (int l = 0; l < 26; l++) {
      float dd = dist[i+dx[l]][j+dy[l]][k+dz[l]];
      if (dd != -1 && dd < mindist) {
	mindist = dd;
	minl = l;
      }
    }
    i+=dx[minl], j+=dy[minl], k+=dz[minl];
    if (mindist == 1e9) {cout << "Something went horribly wrong when finding a path!" << endl; exit(0);}
  }
  for (int i = 0; i < 2; i++) {
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
  }
}



void findPathDijkstra(int(&volume)[RES][RES][RES], pos start, pos end, PathBase&r) {
  static float dist[RES][RES][RES];
  for (int i = 0; i < RES; i++)
    for (int j = 0; j < RES; j++)
      for (int k = 0; k < RES; k++)
	dist[i][j][k] = volume[i][j][k] || i == 0 || j == 0 || k == 0 || i == RES-1 || j == RES-1 || k == RES-1 ? -1 : 1e9;

  int dx[26] = {};
  int dy[26] = {};
  int dz[26] = {};
  float len[26] = {};
  int c = 0;
  for (int i = -1; i < 2; i++)
    for (int j = -1; j < 2; j++)
      for (int k = -1; k < 2; k++) {
	if (i == 0 && j == 0 && k == 0) continue;
	dx[c] = i;
	dy[c] = j;
	dz[c] = k;
	len[c] = sqrtf(i*i+j*j+k*k);
	c++;
      }
  priority_queue<pair<float, pos> > q;
  dist[end.z][end.y][end.x] = 0;
  q.push(make_pair(0, pos(end.z, end.y, end.x)));

  int co = 0;
  while (q.size()) {
    int i = q.top().second.x;
    int j = q.top().second.y;
    int k = q.top().second.z;
    float d = -q.top().first;
    if (i == start.z && j == start.y && k == start.x) {
      //cout << "Found start!" << endl;
      break;
    }
    //if (co++%1000000 == 0) cout << co << endl;
    q.pop();
    if (d > dist[i][j][k]) continue;

    for (int l = 0; l < 26; l++) {
      float dd = d+len[l];
      if (dist[i+dx[l]][j+dy[l]][k+dz[l]] > dd) {
	q.push(make_pair(-dd, pos(i+dx[l], j+dy[l], k+dz[l])));
	dist[i+dx[l]][j+dy[l]][k+dz[l]] = dd;
      }
    }
  }
  /*
    for (int i = 0; i < RES; i++)
    for (int j = 0; j < RES; j++)
    for (int k = 0; k < RES; k++)
    if (volume[i][j][k] == 0) {
    Knot knot;
    knot.p[0] = k-0.5f, knot.p[1] = j-0.5f, knot.p[2] = i-0.5f;
    r.knot.push_back(knot);
    }
  */
  pos p(start.z, start.y, start.x);
  Knot knot;
  for (int i = 0; i < 2; i++) {
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
  }
  while (1) {
    Knot knot;
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
    int&i = p.x;
    int&j = p.y;
    int&k = p.z;
    float d = dist[i][j][k];
    if (d == 1e9) {
      cout << "No path to victory!" << endl;
      exit(0);
    }
    //cout << i << ' ' << j << ' ' << k << ' ' << d << endl;

    if (d <= 1e-5) break;
    int mindist = 1e9, minl = 0;
    for (int l = 0; l < 26; l++) {
      float dd = dist[i+dx[l]][j+dy[l]][k+dz[l]]+len[l];
      if (dd != len[l]-1 && dd < mindist) {
	mindist = dd;
	minl = l;
      }
    }
    i+=dx[minl], j+=dy[minl], k+=dz[minl];
    if (mindist == 1e9) {cout << "Something went horribly wrong when finding a path!" << endl; exit(0);}
  }
  for (int i = 0; i < 2; i++) {
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
  }
}

void findPathAstar(int(&volume)[RES][RES][RES], pos start, pos end, PathBase&r) {
  static float dist[RES][RES][RES];
  for (int i = 0; i < RES; i++)
    for (int j = 0; j < RES; j++)
      for (int k = 0; k < RES; k++)
	dist[i][j][k] = volume[i][j][k] || i == 0 || j == 0 || k == 0 || i == RES-1 || j == RES-1 || k == RES-1 ? -1 : 1e9;

  int dx[26] = {};
  int dy[26] = {};
  int dz[26] = {};
  float len[26] = {};
  int c = 0;
  for (int i = -1; i < 2; i++)
    for (int j = -1; j < 2; j++)
      for (int k = -1; k < 2; k++) {
	if (i == 0 && j == 0 && k == 0) continue;
	dx[c] = i;
	dy[c] = j;
	dz[c] = k;
	len[c] = sqrtf(i*i+j*j+k*k);
	c++;
      }
  priority_queue<pair<pair<float, float>, pos> > q;
  dist[end.z][end.y][end.x] = 0;
  q.push(make_pair(make_pair(0,0), pos(end.z, end.y, end.x)));

  int co = 0;
  while (q.size()) {
    int i = q.top().second.x;
    int j = q.top().second.y;
    int k = q.top().second.z;
    float d = q.top().first.second;
    if (i == start.z && j == start.y && k == start.x) {
      //cout << "Found start!" << endl;
      break;
    }
    //if (co++%1000000 == 0) cout << co << endl;
    q.pop();
    if (d > dist[i][j][k]) continue;

    for (int l = 0; l < 26; l++) {
      float dd = d+len[l];
      if (dist[i+dx[l]][j+dy[l]][k+dz[l]] > dd) {
	float x = start.z-i-dx[l], y = start.y-j-dy[l], z = start.x-k-dz[l];
	float heuristic = sqrtf(x*x+y*y+z*z);
	q.push(make_pair(make_pair(-(dd+heuristic), dd), pos(i+dx[l], j+dy[l], k+dz[l])));
	dist[i+dx[l]][j+dy[l]][k+dz[l]] = dd;
      }
    }
  }
  /*
    for (int i = 0; i < RES; i++)
    for (int j = 0; j < RES; j++)
    for (int k = 0; k < RES; k++)
    if (volume[i][j][k] == 0) {
    Knot knot;
    knot.p[0] = k-0.5f, knot.p[1] = j-0.5f, knot.p[2] = i-0.5f;
    r.knot.push_back(knot);
    }
  */

  pos p(start.z, start.y, start.x);
  Knot knot;
  for (int i = 0; i < 2; i++) {
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
  }
  while (1) {
    Knot knot;
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
    int&i = p.x;
    int&j = p.y;
    int&k = p.z;
    float d = dist[i][j][k];
    if (d == 1e9) {
      cout << "No path to victory!" << endl;
      exit(0);
    }
    //cout << i << ' ' << j << ' ' << k << ' ' << d << endl;

    if (d <= 1e-5) break;
    int mindist = 1e9, minl = 0;
    for (int l = 0; l < 26; l++) {
      float dd = dist[i+dx[l]][j+dy[l]][k+dz[l]]+len[l];
      if (dd != len[l]-1 && dd < mindist) {
	mindist = dd;
	minl = l;
      }
    }
    i+=dx[minl], j+=dy[minl], k+=dz[minl];
    if (mindist == 1e9) {cout << "Something went horribly wrong when finding a path!" << endl; exit(0);}
  }
  for (int i = 0; i < 2; i++) {
    knot.p[0] = p.z-0.5f, knot.p[1] = p.y-0.5f, knot.p[2] = p.x-0.5f;
    r.knot.push_back(knot);
  }
}
