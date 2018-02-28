#include "vec3.hpp"

struct MyTrackball {
	MyScreen*screen;
	vec3 pos;
  int lx, ly;
  float dx, dy, dz, speed, rspeed;
	MyTrackball() {}
	MyTrackball(MyScreen&s) {
		speed = 0.03f;
		rspeed = .2f;
		dx = dy = dz = 0;
		lx = ly = -1;
		pos = vec3(0,0,0);
		screen = &s;
	}
	void handleEvent(Event e) {
		int key = e.key, type = e.type;
		if (e.type == KeyPress) {
			if (key == K_q) dy = 1;
			else if (key == K_e) dy =-1;
			else if (key == K_w) dz = 1;
			else if (key == K_s) dz =-1;
			else if (key == K_d) dx = 1;
			else if (key == K_a) dx =-1;
		} else if (type == KeyRelease) {
			if (key == K_e or key == K_q) dy = 0;
			if (key == K_w or key == K_s) dz = 0;
			if (key == K_a or key == K_d) dx = 0;
		} else if (type == ButtonPress && key == 1) {
			screen->getCursorPos(lx, ly);
		} else if (type == ButtonRelease && key == 1)
			lx = -1;
	}
	void update() {
		if (lx != -1) {
      int mx, my;
      screen->getCursorPos(mx, my);
      screen->setCursorPos(screen->w/2, screen->h/2);
      viewdir.rotatePlane(up, (mx-lx)*rspeed);

      vec3 side = up^viewdir;
      up.rotatePlane(side, (ly-my)*rspeed);
      viewdir.rotatePlane(side, (ly-my)*rspeed);
      lx = screen->w/2;
      ly = screen->h/2;
    }
    vec3 side = up^viewdir;

    pos += side*dx*speed;
    pos += up*dy*speed;
    pos += viewdir*dz*speed;
	}
};

MyTrackball t;
