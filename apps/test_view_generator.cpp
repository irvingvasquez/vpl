#include <viewstructure.h>
#include <viewsynthesis.h>
#include <string>

/* generates random views in a given space */

int main(int argc, char **argv) {
  std::cout << "Consejo Nacional de Ciencia y Tecnología" << std::endl;
  std::cout << "View planning library" << std::endl;
  
  std::cout << "Generates random views in a given space \n"; // Inform the user of how to use the program
  
  if (argc != 3) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.

        std::cout << "Usage is " << argv[0] <<  " <n_views> <outfile.vs>\n"; // Inform the user of how to use the program
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
  
  std::cout << min << std::endl;
  
  ViewStructure max;
  max.setPose(5.0,5.0,5.0,3.14,3.14,3.14);
  
  std::cout << max << std::endl;
  
  ViewList list;
  
  RandomViewSynthesis generator(n, min, max);
  
  generator.getViews(list);
  
  list.save(file);
  
  return 0;
}