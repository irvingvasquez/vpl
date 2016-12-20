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


#include "nbvplannerexpectedutilitynroot.h"

NBVPlannerExpectedUtilityNRoot::NBVPlannerExpectedUtilityNRoot(RobotSensor* rs, PartialModelBase* pm): NBVPlannerExpectedUtility(rs, pm)
{

}


bool NBVPlannerExpectedUtilityNRoot::generateSampleStateComp(list< MSLVector > trajectory, MSLVector x_0, std::vector< double > nsigmas, MSLVector& sample, double& prob, double& prob_nr, Geom* g)
{
  MSLVector x_t = x_0;
  MSLVector x_tmas1;
  list< MSLVector >::iterator it;
  
  // for analysis only
  std::vector<double> probabilities;
  std::vector<double> point;
  std::vector< std::vector<double> > path;
  vpFileReader reader;
  std::string path_fn(dataFolder + "/randomPath");
  std::string prob_fn(dataFolder + "/probabilities");
  
  double epsilon;
  double p_eps, p_eps_nr;
  double p_mov = 0;
  double p_mov_nr = 1;
  double log_prob_path = 0;
  double prob_path_nr = 1;
  
  randomGenerator.randomize();
  
  for(it = trajectory.begin(); it!= trajectory.end(); it++){
    MSLVector u_circunfleja(m->InputDim);
    
    p_mov = 0;
    for (int i = 0; i < m->InputDim; i++){
      // el error es centrado en cero, luego se le resta la media para trasladarlo, si es que hubiera una media diferente de cero
      // de esta forma se calcula la probabilidad de un error centrado en cero
      do{
	epsilon = randomGenerator.drawGaussian1D(0, nsigmas[i]);  
      } while (epsilon < -(empiricalRule * nsigmas[i]) || epsilon > empiricalRule * nsigmas[i]);
      
      u_circunfleja[i] = (*it)[i] + epsilon + Means[i];
      p_eps = logProbabilityOf(epsilon, pow(nsigmas[i],2));
      p_eps_nr = rootProbOf(epsilon, pow(nsigmas[i],2), trajectory.size());
      p_mov = p_mov + p_eps; 
      p_mov_nr = p_mov_nr * p_eps_nr;
      //cout << "p_eps_nr:" << p_eps_nr << " " ;
    } 
    //cout << "p_mov:" << p_mov << " " ;
    
    x_tmas1 = this->m->Integrate(x_t, u_circunfleja, plannerDeltaT);
    log_prob_path += p_mov;
    prob_path_nr = prob_path_nr * p_mov_nr;
    //cout << "log_prob_path:" << log_prob_path << " " ;
    
    probabilities.push_back(log_prob_path); // remove
    
    nbvs::mslvector2stdvector<double>(x_tmas1,point); // remove
    path.push_back(point); // remove
    
    // Verificar si esta libre de colisión
    if (!g->CollisionFree(m->StateToConfiguration(x_tmas1))){
      std::cout << "Colision!" << std::endl;
      sample = x_t;
      prob = log_prob_path;
      prob_nr = prob_path_nr;
      
      reader.saveVectorOfVectors<double>(path, path_fn);
      reader.saveVector<double>(probabilities, prob_fn);
      //getchar();
      return false;
    }
    
    x_t = x_tmas1;
  }
  
  sample = x_t;
  prob = log_prob_path;
  prob_nr = prob_path_nr;
  
  reader.saveVectorOfVectors<double>(path, path_fn);
  reader.saveVector<double>(probabilities, prob_fn);
  
  std::cout << "Sample state generated successfully." << std::endl;
  //getchar();
  return true;
}




bool NBVPlannerExpectedUtilityNRoot::planNBV(ViewStructure& v)
{
  //Variables
  vpFileReader reader;
  ViewList views_list;
  std::string file_name;
  bool success = false;
  double best_eval = 0;
  list<MSLVector> best_path;
  MSLVector best_goal;
  MSLVector init_state;
  list<MSLVector> best_policy1;
  list<MSLVector> best_policy2;
  
  savePlannerData();
  Geom *g = new GeomPQP3DRigidMulti(configFolder);
  generateCandidateViews(views_list, m, g);
  
  ViewStructure best_sensing_view = views_list.getBestView();

  std::cout <<"-----------Motion Planning ----------" << std::endl;
  //savePlannerData();  
  
  clock_t begin = clock();

  ViewList::iterator itv = views_list.begin();
  file_name.clear();
  file_name = configFolder + "/GoalState";
  reader.saveToMSLVector<double>(itv->q, file_name);
  Problem *p = new Problem(g, m, configFolder);
  init_state = p->InitialState;
  
  /**
   * Tratar de alcanzar n vistas
   */
  int i=0;
  for(itv = views_list.begin(); itv!= views_list.end() && i<nViews; itv++){
    i++; 
    
    // TODO tal vez convendría seleccionar vistas que no esten cercanas
    MSLVector goal(itv->q.size());
    
    for(int i = 0;i<itv->q.size(); i++)
      goal[i] = itv->q[i];
      
    p->GoalState = goal;
      
    std::cout << "Tratando de alcanzar: " << p->GoalState << std::endl; 
    // if(p->CollisionFree( m->StateToConfiguration(p->GoalState) ) ){
    RRT *rrt= new RRTExtExt(p);
    rrt->PlannerDeltaT = plannerDeltaT;
    rrt->NumNodes = rrtNodes;
	
    if(rrt->Plan()){
	  success = true;
	  std::cout << "Alcanzado! " << std::endl;
	  std::cout << "Solution Path size: " << rrt->Path.size() << "\n"; 
	  //cout << "Solution Path: \n" << rrt->Path << "\n"; 
	  //cout << "NBV: \n" << rrt->BestState << std::endl;
	  double d = accumulatedWeightedDist(rrt->Path);   //p->Metric(p->InitialState,p->GoalState);
	  itv->d = d;
	  double f_dist = 1 / (1+d);
	  std::cout << "before: " << itv->eval << std::endl;
	  std::cout << d << std::endl;
	  itv->eval = itv->eval * f_dist;
	  std::cout << "after: " << itv->eval << std::endl;
	  
	  /// --------- ExpectedUtility --------------
	  list<MSLVector> trajectory;
	  ViewStructure Sk_view;
	  
	  // only for display purposes
	  std::vector<double> point(3);
	  // only for display purposes
	  std::vector< std::vector<double> > points;
	  point[0] = init_state[0];
	  point[1] = init_state[1];
	  points.push_back(point);
	  point[0] = goal[0];
	  point[1] = goal[1];
	  points.push_back(point);
	  
	  combinePolycies(trajectory, rrt->Policy1, rrt->Policy2);
	  
	  ViewList lista;
	  double expected_utility = 0;
	  std::vector<double> log_probabilities(LSamples);
	  std::vector<double> nr_probabilities(LSamples);
	  std::vector<double> utilities_of_samples(LSamples);
	  std::vector<double> probabilities(LSamples);
	  std::vector<double> probabilities_nr(LSamples);
	  
	  /*
	   * Get l samples with their utilities and probabilities
	   */
	  for(int k = 0; k<LSamples ; k++){
	    // generar samples
	    MSLVector Sk_state;
	    double P_gk;
	    double P_gk_nr;
	    double gk;
	    
	    if(!generateSampleStateComp(trajectory, init_state, Sigmas, Sk_state, P_gk, P_gk_nr, g)){
	      // there is a collision
	      point[0] = Sk_state[0];
	      point[1] = Sk_state[1];
	      point[2] = -1;
	      points.push_back(point);
	      
	      gk = 0;
	      utilities_of_samples[k] = gk;
	      log_probabilities[k] = P_gk;
	      nr_probabilities[k] = P_gk_nr;
	      std::cout << "gk:" << gk << "  P_gk logaritmo:" << P_gk << " P_gk nroot:" << P_gk_nr << std::endl;
	    } else {
	      // collision free
	      std::vector<double> X_temp;
	      nbvs::mslvector2stdvector(Sk_state, X_temp);
	      
	      point[0] = Sk_state[0];
	      point[1] = Sk_state[1];
	      point[2] = 1;
	      points.push_back(point);
	      
	      robotWithSensor->getViewFromState(Sk_view, X_temp);
	      lista.push_back(Sk_view);
	      partialModel->evaluateView(Sk_view);
	      gk = Sk_view.eval;
	      //cout << std::endl << Sk_view ;
	      std::cout << "gk:" << gk << "  P_gk logaritmo:" << P_gk << " P_gk nroot:" << P_gk_nr << std::endl;
	    }
	    utilities_of_samples[k] = gk;
	    log_probabilities[k] = P_gk;
	    nr_probabilities[k] = P_gk_nr;
	 }
	  
	 /*
	 *************************** Compute the expected utility *****************
	 */
	 
	 std::cout << "Return from the log domain" << std::endl;
	 for(int k = 0; k<LSamples ; k++){
	   probabilities[k] = exp(log_probabilities[k]);
	   probabilities_nr[k] = pow(nr_probabilities[k], trajectory.size());
	   std::cout << "k: " << probabilities[k] << " " << probabilities_nr[k] << std::endl; 
	 }
	 
	 // Normalize the probabilities
	 double sum = 0.0;
	 double sum_nr = 0.0;
	 for(int k = 0; k<LSamples ; k++){
	   sum  += probabilities[k];
	   sum_nr += probabilities_nr[k];
	 }
	 std::cout << "Suma: " << sum << " nr:"<< sum_nr <<  std::endl;
	 
	 //cout << "Normalized probabilities" << std::endl;
	 double log_p_normalized;
	 for(int k = 0; k<LSamples ; k++){
	   probabilities[k] = probabilities[k] / sum;
	   probabilities_nr[k] = probabilities_nr[k] / sum_nr;
	   std::cout << k << ": " << probabilities[k] << "   " << probabilities_nr [k] << std::endl;
	 }
	 
	 // compute expected_utility
	 expected_utility = 0.0;
	 for(int k = 0; k<LSamples ; k++){
	    std::cout << "Utility:" << utilities_of_samples[k] << " prob:" << probabilities[k] << std::endl; 
	    expected_utility += utilities_of_samples[k] * probabilities[k];
	 }
	  
	  // Asignar el valor calculado
	  itv->eval = expected_utility;
	  std::cout << *itv << std::endl;
	  std::cout << "Expected Utility: " << expected_utility << std::endl;
	  
	  // Guardar el muestreo
	  lista.save("lista_de_muestreo.vs");
	  vpFileReader read;
	  read.saveDoubleCoordinates("points.dat", points);
	  /// ----------------------------------------
	  
	  
	  if(itv->eval > best_eval){
	    v = *itv;
	    best_eval = itv->eval;
	    best_path = rrt->Path;
	    best_goal = p->GoalState;
	    best_policy1 = rrt->Policy1;
	    best_policy2 = rrt->Policy2;
	  }  
    } else {
      std::cout << "No path found" << std::endl;
    }
    delete rrt;
  }
  
  
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std_mp_time = elapsed_secs;
  motionPTimes.push_back(elapsed_secs);
  std::cout << "Motion Planing elapsed time: " << elapsed_secs << std::endl;
  
  if(success){
    std::cout << "Motion planning success" << std::endl;
    std::ofstream *outfile; 
    MSLVector pose_configuration;
    
    file_name.clear();
    file_name = dataFolder + "/InitialPose";
    pose_configuration = m->StateToConfiguration(init_state);
    
    outfile = new ofstream(file_name.c_str()); 
    if (*outfile) {   
      *outfile << pose_configuration;
      outfile->close();
    }  
    
    file_name.clear();
    file_name = dataFolder + "/GoalPose";
    pose_configuration = m->StateToConfiguration(best_goal);
    
    outfile->open(file_name.c_str()); 
    if (*outfile) {   
      *outfile << pose_configuration;
      outfile->close();
    }
    
    
    //guardar path
    list<MSLVector> posesPath;
    for(list<MSLVector>::iterator it = best_path.begin(); it!= best_path.end(); it++){
      MSLVector pose = m->StateToConfiguration(*it);
      posesPath.push_back(pose);
    }
    file_name.clear();
    file_name = dataFolder + "/PosePath";
    
    outfile->open(file_name.c_str()); 
    if (*outfile) {   
      *outfile << posesPath;
      outfile->close();
    }
    
    
    //guardar frames;
    list<MSLVector> framesPath;
    int inter = 7; //intermediate frames per state;
    double deltat = 1/inter;
    
    // save the first
    list<MSLVector>::iterator lower_it = best_path.begin();
    list<MSLVector>::iterator upper_it = best_path.begin();

    framesPath.push_back(m->StateToConfiguration(*lower_it));
    
    upper_it ++;
    while(upper_it != best_path.end()){
      // save intermediate frames;
      for(int i = 1; i<=inter; i++){ // saves the upper state
	MSLVector pose = p->StateToConfiguration( p->InterpolateState(*lower_it, *upper_it, deltat*i) );
      }
      lower_it = upper_it;
      upper_it++;
    }

    file_name.clear();
    file_name = dataFolder + "/FramePath";
    
    outfile->open(file_name.c_str()); 
    if (*outfile) {   
      *outfile << posesPath;
      outfile->close();
    }
    
    
    file_name.clear();
    file_name = dataFolder + "/visionTimes";
    reader.saveVector<float>(visionTimes,file_name);
    
    file_name.clear();
    file_name = dataFolder + "/motionPlanningTimes";
    reader.saveVector<float>(motionPTimes, file_name);
    
    //traveled_distance += v.d;
    distances_per_it.push_back(v.d);
    //std_distance = accumulatedDistance(best_path, m);
    std_distance = 0;
    std_accu_distance += std_distance;
    file_name.clear();
    file_name = dataFolder + "/traveledDistance";
    reader.saveVector<double>(distances_per_it, file_name);
    
    copySolution(best_path, best_policy1, best_policy2);
    //saveToLogFile();
  }
  
  pointingViews.remove(v);
  
  delete p;
	delete g;
//delete m;
  
  return success;
  
}

bool NBVPlannerExpectedUtilityNRoot::init()
{
return NBVPlannerExpectedUtility::init();
}

double NBVPlannerExpectedUtilityNRoot::rootProbOf(double a, double b, int n)
{
  if(b == 0){
    if(a == 0)
      return 1;
    else
      return 0;
  }
  
  double coef = 1;
  //double coef = 1 / (sqrt(2 * M_PI * b));
  double exponent = (a*a)/b;
  exponent = -0.5 * exponent;
  double prob = coef * exp(exponent);
  
  return ( pow(prob,1.0/(double)n) );
}
