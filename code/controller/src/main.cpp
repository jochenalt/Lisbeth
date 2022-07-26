#include <cstdlib>
#include <iostream>
#include "Utils.hpp"
#include "KeyboardListener.hpp"

KeyboardListener kb;

using namespace std;
int main(int argc, char** argv) {
	cout << "Lisbeth walking controller" << endl;

	kb.setup();
	while (true) {
		delay_ms(1000);
	}
  return EXIT_SUCCESS;

}
