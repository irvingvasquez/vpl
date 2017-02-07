/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "rrtnbv.h"


RRTNBV::RRTNBV(Problem* problem): RRT(problem)
{
  vpFileReader reader;
  std::string config_file(configFolder);
  config_file.append("/Weights");
  
  Interval = problem->UpperState - problem->LowerState;
  std::cout << "Interval: " << Interval << std::endl;
  
  reader.readMSLVector<double>(Weights, config_file);
  std::cout << "Weights: ";
  PMUtils::printVector(Weights);
  
  std::cout << std::endl;
}

RRTNBV::RRTNBV(Problem* problem, std::string config_folder, std::string data_folder): RRT(problem)
{
  setConfigFolder(config_folder);
  setDataFolder(data_folder);
  
  vpFileReader reader;
  std::string config_file(configFolder);
  config_file.append("/Weights");
  
  Interval = problem->UpperState - problem->LowerState;
  std::cout << "Interval: " << Interval << std::endl;
  
  reader.readMSLVector<double>(Weights, config_file);
  std::cout << "Weights: ";
  PMUtils::printVector(Weights);
  
  tree_file.assign(data_folder);
  tree_file.append("/t_evaluations.dat");
  
  std::cout << std::endl;
}



double RRTNBV::weightedDistance(const MSLVector& x1, const MSLVector& x2)
{
  double rho;
  double dist;
  MSLVector dtheta( x1.dim());
  int i;
  
  rho = 0.0;
  double sum=0.0;
  
//   std::cout << "x1  " << x1 << std::endl;
//   std::cout << "x2  " << x2 << std::endl;
  
  
  double xn;
   for ( i= 0 ; i < x1.dim(); i++){
     if (i<2) { 
       xn = (x1[i] - x2[i])/ Interval[i];
       rho = pow(xn,2);
       rho = Weights[i] * rho;
       sum += rho;
     }
     else { 
       dtheta[i] = min(fabs(x1[i]-x2[i]),2.0*PI - fabs(x1[i]-x2[i]));
       
       xn = (dtheta[i] / Interval[i]);
       rho = pow(xn,2);
       rho = Weights[i] * rho;
       sum += rho;
     }
   }
  
  dist = sqrt(sum);
  //cout << "dist " << dist << std::endl;
  return dist;
}



bool RRTNBV::EvaluateCandidate(MSLNode* node, MSLNode* parent)
{
  bool success = false;
  
  MSLVector x = node->State();
  std::vector<double> st(x.dim());
  for(int i = 0; i<x.dim();i++){
    st[i] = x[i];
  }
  
  MSLVector x_parent = parent->State(); // rem
  std::vector<double> st_parent(x.dim()); // rem
  for(int i = 0; i<x_parent.dim();i++){ // rem
    st_parent[i] = x_parent[i]; // rem
  } // rem
  
  //calcular la distancia
  MSLVector parent_x = parent->State();
  double d = weightedDistance(parent_x, x);
  //double d = weightedDistance(this->P->InitialState, x);
  double d_accumulated = parent->accumulatedDistance + d;
  node->accumulatedDistance = d_accumulated;
  
  // TODO: rem, save or display the line between parent and node and evaluation
  std::list<double> line; // rem
  
  ViewStructure v;
  ViewStructure v_parent; //rem
  robotWithSensor->getViewFromState(v, st);
  
  robotWithSensor->getViewFromState(v_parent, st_parent); // rem
  //cout << v << std::endl;
  line.insert(line.end(),v_parent.w.begin(),v_parent.w.end()); // rem 
  line.insert(line.end(),v.w.begin(), v.w.end()); // rem
  
  
  if(partialModel->evaluateView(v)==FEASIBLE_VIEW){
      
      double f_dist = 1 / (1+d_accumulated);
      v.d = d_accumulated;
      v.eval = v.eval * f_dist;
      
      line.push_back(v.eval); // rem
      
      if(v.eval > nbv_v.eval){
	//cout << parent_x << std::endl;
	//cout << x << std::endl;
	cout << "Feasible view found. new NBV:" << std::endl;
	cout << v;
	cout << "d: " << d_accumulated << std::endl;
	cout << "unknown: " << v.n_unknown << std::endl;
	nbv_v = v;
	nbv_x = x;
	
// 	cout << parent->accumulatedDistance << std::endl;
// 	cout << d << std::endl;
// 	cout << node->accumulatedDistance << std::endl;
	
// 	getchar();
	success = true;
	//return true;
      }
    } else {
      line.push_back(0); // rem
      success = false;
    }
    
    t_evaluations.push_back(line); // rem
    
    return success;
}




bool RRTNBV::Extend(const MSLVector& x, MSLTree* t, MSLNode*& nn, bool forward, bool& new_nbv)
{
  MSLNode *n_best;
  MSLVector nx,u_best;
  bool success;
  
  n_best = SelectNode(x,t,forward); // return the nearest neighboor
  u_best = SelectInput(n_best->State(),x,nx,success,forward);
  // nx gets next state
  if (success) {   // If a collision-free input was found
    // Extend the tree
    nn = t->Extend(n_best, nx, u_best, PlannerDeltaT);
    //ViewStructure v;
    new_nbv = this->EvaluateCandidate(nn, n_best);
    
//     std::cout << "n_best: " << n_best->State() << "\n";
//     std::cout << "New node: " << nn->State() << "\n";
  }

  return success;
}



bool RRTNBV::Plan()
{
  t_evaluations.clear(); // rem
  
  std::cout << "Starting NBVS RRT with " << NumNodes << " nodes " << std::endl;
  
  int i;
  double d;
  MSLNode *n,*nn,*n_goal;
  MSLVector nx,u_best;
  list<MSLNode*> path;

  // Keep track of time
  float t = used_time();

  // Make the root node of G
  if (!T)
    T = new MSLTree(P->InitialState);

  nn = T->Root();

  i = 0;
  n = SelectNode(P->GoalState,T, true);
  n_goal = n;
  foundNBV = false;
  bool new_nbv;
  GoalDist = P->Metric(n->State(),P->GoalState);
  //while ((i < NumNodes)&&(!GapSatisfied(n_goal->State(),P->GoalState))) {
  while (i < NumNodes) {
    if (Extend(ChooseState(),T,nn,true,new_nbv)) { 
      if(new_nbv){
	foundNBV = true;
	d = P->Metric(nn->State(),P->InitialState);
	//if (d < GoalDist) {  // Decrease if goal closer
	GoalDist = d;
	BestState = nn->State();
	n_goal = nn;
	  //cout << "GoalDist " << GoalDist << "\n";
	//}
      } 
    }
    i++;
  }

  CumulativePlanningTime += ((double)used_time(t));
  std::cout << "Planning Time: " << CumulativePlanningTime << "s\n"; 
  
  
  //save t_evaluations
  vpFileReader file_r; // rem
  std::cout << "saving evaluations..."; // rem
  file_r.saveListOfLists<double>(t_evaluations, tree_file); // rem
  std::cout << " done" << std::endl; // rem
  
  // Get the solution path
  if (foundNBV) {
    std::cout << "Success\n";
    this->P->GoalState = BestState;
    path = T->PathToRoot(n_goal);
    path.reverse();
    RecordSolution(path); // Write to Path and Policy
    return true;
  }
  else {
    std::cout << "Failure\n";
    return false;
  }
}

void RRTNBV::setPartialModel(PartialModelBase* pm)
{
  partialModel = pm;
}

void RRTNBV::setRobotWithSensor(RobotSensor* rs)
{
  robotWithSensor = rs;
}

void RRTNBV::setConfigFolder(std::string folder)
{
  configFolder = folder;
}

void RRTNBV::setDataFolder(string folder)
{
  dataFolder = folder;
}



ViewStructure RRTNBV::getNBV()
{
  return nbv_v;
}
