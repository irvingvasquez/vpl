
#include <mslvector.h>


  template <typename T> 
  void mslvector2stdvector(MSLVector &A, std::vector<T> &B){
    T value;
    B.clear();
    for(int i = 0; i < A.dim() ; i++){
      value = (T) A[i];
      B.push_back(value);
    } 
  }
  
  template <typename T>
  void stdvector2mslvector(std::vector<T> A, MSLVector &B){
    int size = A.size();
    MSLVector Aux(size);
    for(int i = 0; i < size ; i++){
      Aux[i] = A[i];
    }
    B = Aux;
  }