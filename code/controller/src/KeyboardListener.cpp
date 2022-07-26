/*
 * KeyboardListener.cpp
 *
 *  Created on: 26 Jul 2022
 *      Author: jochen
 */

#include "KeyboardListener.hpp"
#include "Utils.hpp"

#include <stdio.h>
#include <sys/ioctl.h> // For FIONREAD
#include <termios.h>
#include <stdbool.h>
#include <unistd.h>
using namespace std;
KeyboardListener::KeyboardListener()
{}

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
	// start a thread that listens to the keyboard in the background
	keyboard_thread = new std::thread(&KeyboardListener::listener, this);  // spawn new thread that runs MPC in parallel
}

void KeyboardListener::listener() {

	while (!stop) {
		int n = keyPressed();
		if (n > 0) {
			for (int i = 0;i<n;i++) {
				 int c = getchar();
				 cout << "ch =" << c << endl;
			}
		} else {
			delay_ms(1);
		}
	}
}

