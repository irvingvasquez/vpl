#include <iostream>
#include <string>
#include <pmutils.h>
#include <string>

// When passing char arrays as parameters they must be pointers
int main(int argc, char* argv[]) {
    if (argc != 3) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
        std::cout << "Usage is ./raw2triangles <infile> <outfile>\n"; // Inform the user of how to use the program
        std::cin.get();
        exit(0);
    } else { // if we got enough parameters...
	std::string infile(argv[1]);
	std::string outfile(argv[2]);
	
	PMUtils::raw2triangles(infile, outfile);

        return 0;
    }
}