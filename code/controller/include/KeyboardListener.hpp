/*
 * KeyboardListener.h
 *
 *  Created on: 26 Jul 2022
 *      Author: jochen
 */

#ifndef SRC_KEYBOARDLISTENER_H_
#define SRC_KEYBOARDLISTENER_H_

#include <thread>
#include <queue>
#include <map>
#include <vector>

class KeyboardListener
{
public:
	static const int ESC = 27;
	static const int ARROW_UP = 1000;
	static const int ARROW_DOWN = 1001;
	static const int ARROW_LEFT= 1002;
	static const int ARROW_RIGHT = 1003;

	static const int SHIFT_ARROW_UP = 1010;
	static const int SHIFT_ARROW_DOWN = 1011;
	static const int SHIFT_ARROW_LEFT= 1012;
	static const int SHIFT_ARROW_RIGHT = 1013;

	static const int CTRL_ARROW_UP = 1020;
	static const int CTRL_ARROW_DOWN = 1021;
	static const int CTRL_ARROW_LEFT= 1022;
	static const int CTRL_ARROW_RIGHT = 1023;

	static const int ALT_ARROW_UP = 1030;
	static const int ALT_ARROW_DOWN = 1031;
	static const int ALT_ARROW_LEFT= 1032;
	static const int ALT_ARROW_RIGHT = 1033;


	static const int PG_UP = 1050;
	static const int PG_DOWN = 1051;
	static const int CTRL_PG_UP = 1060;
	static const int CTRL_PG_DOWN = 1061;
	static const int ALT_PG_UP = 1070;
	static const int ALT_PG_DOWN = 1071;

	KeyboardListener();
	virtual ~KeyboardListener();

	// start a background thread waiting for keystrokes
	void setup();

	// stop the background thread
	void teardown();

	//  returns true, if a key from keyboard is available
	bool isKeyAvailable();

	// thats the key, check isKeyAvailable first
	int getKey();

	bool isSpecialKey(int key) {
		return (key >= 1000);
	}
	// there can be only one
	static KeyboardListener& getInstance() {
		static KeyboardListener singleton;
		return singleton;
	}

private:
	void listener();
	int keyPressed();

	std::thread* keyboard_thread = NULL;	// solver runs in this background thread
	bool stop = false;							// flag to stop the background thread
   std::queue<int> q;							// queue used for keyboard input
   std::map<int, std::vector<int>> specialkeys;		// definition of individual strokes when special keys are pressed
};




#endif /* SRC_KEYBOARDLISTENER_H_ */
