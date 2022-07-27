/*
 * KeyboardListener.cpp
 *
 *  Created on: 26 Jul 2022
 *      Author: jochen
 */

#include "KeyboardListener.hpp"
#include "Utils.hpp"

#include <stdio.h>
#include <sys/ioctl.h> // for terminal access
#include <termios.h>
#include <stdbool.h>
#include <unistd.h>
using namespace std;

// ensure the singleton
static bool singletonCheck = false;

KeyboardListener::KeyboardListener()
{
	if (singletonCheck) {
		cerr << "KeyboardListsener should be instantiated only once. Always use KeyboardListener::getInstance()" << endl;
	}
}

KeyboardListener::~KeyboardListener()
{}


int KeyboardListener::keyPressed() {
    static bool initflag = false;
    static const int STDIN = 0;

    if (!initflag) {
        // Use termios to turn off line buffering
        struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initflag = true;
    }

    int nbbytes;
    ioctl(STDIN, FIONREAD, &nbbytes);  // 0 is STDIN
    return nbbytes;
}

void KeyboardListener::teardown() {
	stop = true;
}


void KeyboardListener::setup() {
	// setup can only be called on one instance
	singletonCheck = true;

	// start a thread that listens to the keyboard in the background
	keyboard_thread = new std::thread(&KeyboardListener::listener, this);  // spawn new thread that runs MPC in parallel


	const static int ALTCTRLSHIFT = 59;
	const static int ALT = 51;
	const static int CURSOR = 91;

	const static int CTRL = 53;
	const static int SHIFT = 50;
	const static int UP = 65;
	const static int DOWN = 66;
	const static int RIGHT = 67;
	const static int LEFT = 68;
	const static int PG = 126;
	const static int PGUP = 53;
	const static int PGDOWN = 54;


	specialkeys.insert (std::pair<int, vector<int>>(ARROW_UP, vector<int>{ESC,CURSOR,UP}));
	specialkeys.insert (std::pair<int, vector<int>>(ARROW_DOWN, vector<int>{ESC,CURSOR,DOWN}));
	specialkeys.insert (std::pair<int, vector<int>>(ARROW_RIGHT, vector<int>{ESC,CURSOR,RIGHT}));
	specialkeys.insert (std::pair<int, vector<int>>(ARROW_LEFT, vector<int>{ESC,CURSOR,LEFT}));

	specialkeys.insert (std::pair<int, vector<int>>(SHIFT_ARROW_UP, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,SHIFT,UP}));
	specialkeys.insert (std::pair<int, vector<int>>(SHIFT_ARROW_DOWN, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,SHIFT,DOWN}));
	specialkeys.insert (std::pair<int, vector<int>>(SHIFT_ARROW_RIGHT, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,SHIFT,RIGHT}));
	specialkeys.insert (std::pair<int, vector<int>>(SHIFT_ARROW_LEFT, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,SHIFT,LEFT}));

	specialkeys.insert (std::pair<int, vector<int>>(CTRL_ARROW_UP, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,CTRL,UP}));
	specialkeys.insert (std::pair<int, vector<int>>(CTRL_ARROW_DOWN, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,CTRL,DOWN}));
	specialkeys.insert (std::pair<int, vector<int>>(CTRL_ARROW_RIGHT, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,CTRL,RIGHT}));
	specialkeys.insert (std::pair<int, vector<int>>(CTRL_ARROW_LEFT, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,CTRL,LEFT}));

	specialkeys.insert (std::pair<int, vector<int>>(ALT_ARROW_UP, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,ALT,UP}));
	specialkeys.insert (std::pair<int, vector<int>>(ALT_ARROW_DOWN, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,ALT,DOWN}));
	specialkeys.insert (std::pair<int, vector<int>>(ALT_ARROW_RIGHT, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,ALT,RIGHT}));
	specialkeys.insert (std::pair<int, vector<int>>(ALT_ARROW_LEFT, vector<int>{ESC,CURSOR,49,ALTCTRLSHIFT,ALT,LEFT}));

	specialkeys.insert (std::pair<int, vector<int>>(PG_UP, vector<int>{ESC,CURSOR,PGUP,PG}));
	specialkeys.insert (std::pair<int, vector<int>>(PG_DOWN, vector<int>{ESC,CURSOR,PGDOWN,PG}));
	specialkeys.insert (std::pair<int, vector<int>>(CTRL_PG_UP, vector<int>{ESC,CURSOR,PGUP,ALTCTRLSHIFT,CTRL,PG}));
	specialkeys.insert (std::pair<int, vector<int>>(CTRL_PG_DOWN, vector<int>{ESC,CURSOR,PGDOWN,ALTCTRLSHIFT,CTRL,PG}));
	specialkeys.insert (std::pair<int, vector<int>>(ALT_PG_UP, vector<int>{ESC,CURSOR,PGUP,ALTCTRLSHIFT,ALT,PG}));
	specialkeys.insert (std::pair<int, vector<int>>(ALT_PG_DOWN, vector<int>{ESC,CURSOR,PGDOWN,ALTCTRLSHIFT,ALT,PG}));

}



void KeyboardListener::listener() {

	// run end endless loop listenting to the keyboard, handling special characvters and buffering the result
	while (!stop) {
		int n = keyPressed();
		if (n > 0) {
			// fetch all avilbel key strokes from the system
			// (special keys are sent in a row)
			vector<int> s;
			for (int i = 0;i<n;i++) {
				 int c = getchar();
				 s.push_back(c);
				 // cout << "ch =" << c << endl;
			}

			// check all defined special keys
			bool found_special_key = false;
			map<int, vector<int>>::iterator it;
			for (it = specialkeys.begin(); it != specialkeys.end(); it++)
			{
				 vector<int> def = it->second;
			    bool sequence_is_equal = true;
			    for (unsigned i = 0;i<def.size();i++) {
			   	 if (def[i] != s[i]) {
			   		 sequence_is_equal = false;
			   		 break;
			   	 }
			    }
			    if (sequence_is_equal) {
			   	 q.push(it->first);
			   	 found_special_key = true;
			   	 break;
			    }
			}

			// if it is no special key, just add all keys to the queue
			if (!found_special_key) {
				for (unsigned i = 0;i<s.size();i++) {
					q.push(s[i]);
				}
			}
		} else {
			delay_ms(1);
		}
	}
}

//  returns true, if a key is in the buffer
bool KeyboardListener::isKeyAvailable() {
	return q.size() > 0;
}

// thats the most recent key pressed, check isKeyAvailable first
int KeyboardListener::getKey() {
	if (q.size() > 0) {
		int c = q.front();
		q.pop();
		return c;
	}
	return -1;
}

