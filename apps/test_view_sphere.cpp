#include <math.h>
#include <viewstructure.h>
#include <viewsynthesis.h>
#include <string>

int main(int argc, char **argv) {
  if (argc != 4) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
        std::cout << "Usage is ./test_view_generator <radius> <subs_level> <outfile>\n"; // Inform the user of how to use the program
        //std::cin.get();
        exit(0);
  }
  float r= atof(argv[1]);
  int n= atoi(argv[2]); 
  std::string file(argv[3]);
  
  if(n<0){
     std::cout << "Amount of views must be positive\n"; // Inform the user of how to use the program
     //std::cin.get();
     exit(0);
  }
  
  ViewStructure min;
  min.setPose(-5.0,-5.0,0,0,0,0);

  ViewStructure max;
  max.setPose(5.0,5.0,5.0,3.14,3.14,3.14);
  
  ViewList list;
  
  ViewSphereSynthesis generator(r, 0, 0, 0, n);
  
  generator.getViews(list);
  
  list.save(file);
  
  return 0;
}