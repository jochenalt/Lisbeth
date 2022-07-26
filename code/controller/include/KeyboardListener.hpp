/*
 * KeyboardListener.h
 *
 *  Created on: 26 Jul 2022
 *      Author: jochen
 */

#ifndef SRC_KEYBOARDLISTENER_H_
#define SRC_KEYBOARDLISTENER_H_

#include <thread>

class KeyboardListener
{
public:
	KeyboardListener();
	virtual ~KeyboardListener();

	// start a background thread waiting for keystrokes
	void setup();

	// stop the background thread
	void teardown();
private:
	void listener();
	int keyPressed();

	std::thread* keyboard_thread = NULL;	// solver runs in this background thread
	bool stop = false;
};



#endif /* SRC_KEYBOARDLISTENER_H_ */
