/*
 * Copyright (c) 2016, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "pmvoctreevasquez14.h"


PMVOctreeVasquez14::PMVOctreeVasquez14(double alphaOccupied, double alphaUnknown)
{
  alpha_occupied = alphaOccupied;
  alpha_unknown = alphaUnknown;
}


int PMVOctreeVasquez14::evaluateView(ViewStructure& v)
{
  if(!poitsToTheObject(v)){
    //cout << "Sorry no points :(" << std::endl;
    return UNFEASIBLE_VIEW;
  }
  
  EvaluationResult result;
  
  bool valid_result = rayTracingHTM(v.HTM, result);
  
  double f_area;
  double f_occlusion;
  double f_navigation;
  double f_quality;
  double total;
  
  if(valid_result){
    /// Evaluate the result of the raytracing
    total = (double)(result.n_occupied + result.n_unknown);
    f_area = functionF(result.n_occupied/total, alpha_occupied) + functionF(result.n_unknown/total, alpha_unknown);
    f_quality = 0;
    f_navigation = 0;
    f_occlusion = alpha_unknown/rays.size();
    
    // f_utility
    v.eval = f_area * (f_quality + f_navigation + f_occlusion);
    
    return FEASIBLE_VIEW;
  }
  
  return UNFEASIBLE_VIEW;
}


double PMVOctreeVasquez14::functionF(double x, double alpha)
{
  
  double a0,a1,a2,a3,amuc,y;
  
  if(x>= 0 && x<=alpha){
    a0 = -2/(alpha*alpha*alpha); 
    a1 = 3/(alpha*alpha);
    y = a0 * x*x*x + a1 * x*x;
  }
  else{
    if(x>alpha && x<=1){
      amuc = (alpha - 1)*(alpha - 1)*(alpha - 1);
      a0 = -2/amuc;
      a1 = (3*(alpha+1))/amuc;
      a2 = -(6*alpha)/amuc;
      a3 = (3*alpha-1)/amuc;
      y = a0*x*x*x + a1*x*x + a2*x + a3;
    }
    else{
      y = 0;
    }
  }
  
  return y;
}

