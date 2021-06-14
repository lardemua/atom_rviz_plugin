// A C++ program that compiles and runs a python program
#include <bits/stdc++.h>
using namespace std;
int main ()
{
	// Build command to execute.
	string str = "./copy_content_to_yaml.py -yc test_config_content.yml -yf test_config_format.yml -yo test_out.yml";

	// Convert string to const char * as system requires
	// parameter of type const char *
	const char *command = str.c_str();

	cout << "Running command " << endl << command << endl;
	system(command);

	return 0;
}
