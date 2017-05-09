#include <math.h>
#include <viewstructure.h>
#include <viewsynthesis.h>
#include <string>

int main(int argc, char **argv) {
  if (argc != 3) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
        std::cout << "Usage is ./test_view_generator <n_views> <outfile>\n"; // Inform the user of how to use the program
        //std::cin.get();
        exit(0);
  }
  int n= atoi(argv[1]); 
  std::string file(argv[2]);
  
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
  
  RandomViewSynthesis generator(n, min, max);
  
  generator.getViews(list);
  
  list.save(file);
  
  return 0;
}