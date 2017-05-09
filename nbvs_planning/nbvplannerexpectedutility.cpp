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


#include "nbvplannerexpectedutility.h"

NBVPlannerExpectedUtility::NBVPlannerExpectedUtility(RobotSensor* rs, PartialModelBase* pm): NBVPlannerRGFlt(rs, pm)
{
}

double NBVPlannerExpectedUtility::probabilityOf(double a, double b)
{
  if(b == 0){
    if(a == 0)
      return 1;
    else
      return 0;
  }
  
  //double coef = 1;
  double coef = 1 / (sqrt(2 * M_PI * b));
  double exponent = (a*a)/b;
  exponent = -0.5 * exponent;
  double prob = coef * exp(exponent);
  
  return prob;
}

double NBVPlannerExpectedUtility::logProbabilityOf(double a, double b)
{
  if(b == 0){
    if(a == 0)
      return log(1);
    else
      return std::numeric_limits< double >::min();
  }
  
  double coef = 1;
  //double coef = 1 / (sqrt(2 * M_PI * b));
  double exponent = (a*a)/b;
  exponent = -0.5 * exponent;
  double prob = coef * exp(exponent);
  
  return ( log(prob) );
}


bool NBVPlannerExpectedUtility::init()
{
  NBVPlannerRGFlt::init();
  
  vpFileReader reader;
  std::string sigmas_file(configFolder);
  sigmas_file.append("/Sigmas");
  reader.readMSLVector<double>(Sigmas, sigmas_file);
  
  std::string means_file(configFolder);
  means_file.append("/Means");
  reader.readMSLVector<double>(Means, means_file);
  
  // Read some parameters
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("plannerConfig.ini");
  
  mrpt::utils::CConfigFile parser;
  ASSERT_FILE_EXISTS_(config_file);
  parser.setFileName(config_file);
  
  LSamples = parser.read_int("ExpectedUtility", "LSamples", 10);
  
  std::cout << "---------- NBV Planner ExpectedUtility -------" << std::endl;
  std::cout << "L Samples: " << LSamples << std::endl;
  std::cout << "Sigmas: " ;
  printVector(Sigmas);
  std::cout << "Means: ";
  printVector(Means);
  std::cout << "------------------------------" << std::endl;
  
  empiricalRule = 3;
}


bool NBVPlannerExpectedUtility::generateSampleState(list< MSLVector > trajectory, MSLVector x_0, MSLVector goal, std::vector< double > nsigmas, MSLVector& sample, double& prob, Geom *g)
{
  MSLVector x_t = x_0;
  MSLVector x_tmas1;
  list< MSLVector >::iterator it;
  
  // for analysis only
  std::vector<double> probabilities;
  std::vector<double> point;
  std::vector< std::vector<double> > path;
  vpFileReader reader;
  std::string path_fn(dataFolder + "/randomPath_eu");
  std::string prob_fn(dataFolder + "/probabilities");
  
  double epsilon;
  double p_eps;
  double p_mov = 1;
  double log_prob_path = 0;
  
  randomGenerator.randomize();
  
  nbvs::mslvector2stdvector<double>(x_0,point); // remove
  path.push_back(point);
  
  for(it = trajectory.begin(); it!= trajectory.end(); it++){
    MSLVector u_circunfleja(m->InputDim);
    
    p_mov = 0;
    for (int i = 0; i < m->InputDim; i++){
      // Solo si la velocidad es diferente de cero introduciremos error
      if(!((*it)[i] == 0)){
      	// el error es centrado en cero, luego se le resta la media para trasladarlo,
	// si es que hubiera una media diferente de cero
	// de esta forma se calcula la probabilidad de un error centrado en cero
	do{
	  epsilon = randomGenerator.drawGaussian1D(0, nsigmas[i]);
	} while (epsilon < -(empiricalRule * nsigmas[i]) || epsilon > empiricalRule * nsigmas[i]);
	
	u_circunfleja[i] = (*it)[i] + epsilon + Means[i];
	p_eps = logProbabilityOf(epsilon, pow(nsigmas[i],2));
	p_mov = p_mov + p_eps; 
	//cout << "p_eps:" << p_eps << " " ;
	
      }
    }
    //cout << "p_mov:" << p_mov << " " ;
    
    x_tmas1 = this->m->Integrate(x_t, u_circunfleja, plannerDeltaT);
    log_prob_path += p_mov;
    //cout << "log_prob_path:" << log_prob_path << " " ;
    
    probabilities.push_back(log_prob_path); // remove
    
    nbvs::mslvector2stdvector<double>(x_tmas1,point); // remove
    path.push_back(point); // remove
    
    // Verificar si esta en los limites
    if(!m->Satisfied(x_tmas1)){
      sample = x_t;
      prob = log_prob_path;
      return false;
    }
    
    // Verificar si esta libre de colisión
    if (!g->CollisionFree(m->StateToConfiguration(x_tmas1))){
      sample = x_t;
      prob = log_prob_path;
      
      //reader.saveVectorOfVectors<double>(path, dataFolder + "/randomPath_collision_eu");
      //reader.saveVector<double>(probabilities, prob_fn);
      //cout << "Colision!" << std::endl;
      //getchar();
      return false;
    }
    
    x_t = x_tmas1;
  }
  
  // WARNING esto es un patch para que funione con el brazo que es holonómico
  //cout << "Patch, sample:" << x_t << " \tGoal:" << goal << "\n"; 
  for(int j=3; j < goal.dim(); j++){
    x_t[j] = goal[j];
  }
  //cout << " \t Corrected sample:" << x_t << std::endl;
  //getchar();
  
  sample = x_t;
  prob = log_prob_path;
  
  //reader.saveVectorOfVectors<double>(path, path_fn);
  //reader.saveVector<double>(probabilities, prob_fn);
  
  //cout << "Sample state generated successfully." << std::endl;
  //getchar();
  return true;
}


double NBVPlannerExpectedUtility::logAdd(double logX, double logY)
{
  //Tomado de https://facwiki.cs.byu.edu/nlp/index.php/Log_Domain_Computations
  // make x the max
  if(logY > logX){
    double temp = logX;
    logX = logY;
    logY = temp;
  }
  
   // 2. now X is bigger
   if (logX == std::numeric_limits<double>::min()) {
       return logX;
   }    
  
  // 3. how far "down" (think decibels) is logY from logX?
  //    if it's really small (20 orders of magnitude smaller), then ignore
  double negDiff = logY - logX;
  if (negDiff < -20) {
     return logX;
  }
  
  // 4. otherwise use some nice algebra to stay in the log domain
  //    (except for negDiff)
  return (logX + log(1.0 + exp(negDiff)));
}


void NBVPlannerExpectedUtility::combinePolycies(list< MSLVector >& trajectory, list< MSLVector >& policy1, list< MSLVector >& policy2)
{
  // policy1
//   std::cout << "policy 1" << std::endl;
  std::vector<double> u;
  trajectory.clear();
  for(list<MSLVector>::iterator it = policy1.begin() ; it!= policy1.end(); it++){
    nbvs::mslvector2stdvector<double>(*it, u);
//     PMUtils::printVector(u);
//     i++;
    trajectory.push_back(*it);
  }
  
  //insert an empty control to start with the second policy
  //std::vector<double> empty_control(m->InputDim);
  //PMUtils::printVector(empty_control);
  //trajectory.push_back(empty_control);
  
  //policy 2

  if(policy2.size()!=0){
//     std::cout << "policy 2" << std::endl;
    for(list<MSLVector>::iterator it = policy2.begin() ; it!= policy2.end(); it++){
      nbvs::mslvector2stdvector<double>(*it, u);
//       PMUtils::printVector(u);
      if(u.size()!=0){
	trajectory.push_back(*it);
      }
    }
  }

}


bool NBVPlannerExpectedUtility::planNBV(ViewStructure& v)
{
  //Variables
  vpFileReader reader;
  ViewList views_list;
  std::string file_name;
  bool success = false;
  double best_eval = -1;
  int connect_ok = 0;
  double connect_rate = 0.0;
  list<MSLVector> best_path;
  MSLVector best_goal;
  MSLVector best_sensing_goal;
  MSLVector init_state;
  list<MSLVector> best_policy1;
  list<MSLVector> best_policy2;
  
  savePlannerData();
  Geom *g = new GeomPQP3DRigidMulti(configFolder);
  generateCandidateViews(views_list, m, g);
  
  ViewStructure best_sensing_view = views_list.getBestView();
  nbvs::stdvector2mslvector<double>(best_sensing_view.q, best_sensing_goal);
  
  std::cout <<"----------- Motion Planning ----------" << std::endl;
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
    
    for(int j = 0;j<itv->q.size(); j++)
      goal[j] = itv->q[j];
      
    p->GoalState = goal;
      
    std::cout << "\n" << i << "/" << nViews << "Tratando de alcanzar: " << p->GoalState << std::endl; 
    // if(p->CollisionFree( m->StateToConfiguration(p->GoalState) ) ){
    RRT *rrt= new RRTExtExt(p);
    rrt->PlannerDeltaT = plannerDeltaT;
    rrt->NumNodes = rrtNodes;
	
    if(rrt->Plan()){
	  connect_ok ++;
	  //cout << "Alcanzado! " << std::endl;
	  //cout << "Solution Path size: " << rrt->Path.size() << "\n"; 
	  //cout << "Solution Path: \n" << rrt->Path << "\n"; 
	  //cout << "NBV: \n" << rrt->BestState << std::endl;
 	  double d = accumulatedWeightedDist(rrt->Path);   //p->Metric(p->InitialState,p->GoalState);
 	  itv->d = d;
 	  double f_dist = 1 / (1+d);
	  std::cout << "Unknown: " << itv->n_unknown;
	  std::cout << " \tDistance" << d;
	  std::cout << " \tUtility:" << itv->eval * f_dist;
	  //cout << "after: " << itv->eval << std::endl;
	  
	  // Guardar el path para graficación
	  std::vector< std::vector<double> > original_path;
	  std::vector<double> point_of_path;
	  list< MSLVector > rrt_path;
	  list< MSLVector >::iterator it_path;
	  rrt_path = rrt->Path;
	  for(it_path = rrt_path.begin(); it_path != rrt_path.end(); it_path++){
	    nbvs::mslvector2stdvector<double>(*it_path,point_of_path);
	    original_path.push_back(point_of_path);
	  }
	
	  /// --------- ExpectedUtility --------------
	  list<MSLVector> trajectory;
	  ViewStructure Sk_view;
	  
	  // only for display purposes
	  std::vector<double> point(3);
	  // only for display purposes
	  std::vector< std::vector<double> > init_and_goal;
	  std::vector< std::vector<double> > sample_points;
	  std::vector< std::vector<double> > collision_points;
	  
	  point[0] = init_state[0];
	  point[1] = init_state[1];
	  init_and_goal.push_back(point);
	  point[0] = goal[0];
	  point[1] = goal[1];
	  init_and_goal.push_back(point);
	  
	  //vpFileReader read;
	  //read.saveDoubleCoordinates("init_and_goal_eu.dat", points);
	  
	  combinePolycies(trajectory, rrt->Policy1, rrt->Policy2);
	  
	  ViewList lista;
	  double expected_utility = 0;
	  std::vector<double> log_probabilities(LSamples);
	  std::vector<double> utilities_of_samples(LSamples);
	  std::vector<int> coverage_of_samples(LSamples);
	  std::vector<double> probabilities(LSamples);
	  
	   
	  /*
	   * Get l samples with their utilities and probabilities
	   */
	  for(int k = 0; k<LSamples ; k++){
	    // generar samples
	    MSLVector Sk_state;
	    double P_gk;
	    double gk;
	    
	    if(!generateSampleState(trajectory, init_state, goal, Sigmas, Sk_state, P_gk, g)){
	      // there is collision
	      gk = - 1000;
	      //utilities_of_samples[k] = gk;
	      //log_probabilities[k] = P_gk;
	      //cout << "Collision of the sample state" << std::endl;
	      // std::cout << "gk:" << gk << "  P_gk:" << P_gk << std::endl;
	      
	      point[0] = Sk_state[0];
	      point[1] = Sk_state[1];
	      collision_points.push_back(point);
	      coverage_of_samples[k] = 0;
	    } else {
	      // collision free
	      std::vector<double> X_temp;
	      nbvs::mslvector2stdvector(Sk_state, X_temp);
	      
	      point[0] = Sk_state[0];
	      point[1] = Sk_state[1];
	      sample_points.push_back(point);
	      
	      robotWithSensor->getViewFromState(Sk_view, X_temp);
	      lista.push_back(Sk_view);
	      partialModel->evaluateView(Sk_view);
	      gk = Sk_view.eval * f_dist;
	      coverage_of_samples[k] = Sk_view.n_unknown;
	      //cout << std::endl << "sample state with evaluation" << std::endl << Sk_view << std::endl;
	      //cout << "gk:" << gk << "  P_gk:" << P_gk << std::endl;
	    }
	    
	    //getchar();
	    utilities_of_samples[k] = gk;
	    log_probabilities[k] = P_gk;
	 }
	  
	 /*
	 *************************** Compute the expected utility *****************
	 */
	 
	 //cout << "Return from the log domain" << std::endl;
	 for(int k = 0; k<LSamples ; k++){
	   probabilities[k] = exp(log_probabilities[k]);
	   //cout << "k: " << probabilities[k] << std::endl; 
	 }
	 
	 // Normalize the probabilities
	 double sum = 0.0;
	 for(int k = 0; k<LSamples ; k++){
	   sum  += probabilities[k];
	 }
	 //cout << "Suma: " << sum << std::endl;
	 
	 //cout << "Normalized probabilities" << std::endl;
	 double log_p_normalized;
	 for(int k = 0; k<LSamples ; k++){
	   probabilities[k] = probabilities[k] / sum;
	   //cout << k << ": " << probabilities[k] << std::endl;
	 }
	 
	 // compute expected_utility
	 expected_utility = 0.0;
	 double expected_coverage = 0.0;
	 for(int k = 0; k<LSamples ; k++){
	    //cout << "Utility:" << utilities_of_samples[k] << " prob:" << probabilities[k] << std::endl; 
	    expected_utility += utilities_of_samples[k] * probabilities[k];
	    expected_coverage += (double)coverage_of_samples[k] * probabilities[k];
	 }
	  
	 // Asignar el valor calculado
	 itv->eval = expected_utility;
	 itv->n_unknown = (int) floor(expected_coverage + 0.5);
	 //cout << *itv << std::endl;
	 std::cout << "\tExp_Coverage:" << expected_coverage << "\tExp_Utility:" << expected_utility << std::endl;
	  
	 
	 //cout << "expected utility computed" << std::endl;
	 //getchar();
	 /// ----------------------------------------
	  
	 if(itv->eval > best_eval){
	    std::cout << "New NBV founded using expected utility" << std::endl;
	    success = true;
	    v = *itv;
	    best_eval = itv->eval;
	    best_path = rrt->Path;
	    best_goal = p->GoalState;
	    best_policy1 = rrt->Policy1;
	    best_policy2 = rrt->Policy2;
	    
	    // Guardar el muestreo
	    lista.save("lista_de_muestreo.vs");
	    reader.saveDoubleCoordinates(dataFolder + "/init_and_goal_eu", init_and_goal);
	    reader.saveVectorOfVectors<double>(original_path, dataFolder + "/original_path_eu.dat");
	    reader.saveDoubleCoordinates(dataFolder + "/sample_points_eu.dat", sample_points);
	    reader.saveDoubleCoordinates(dataFolder + "/sample_points_collision.dat", collision_points);
	    reader.saveVector<double>(probabilities, dataFolder + "/eu_probabilities");
	    reader.saveVector<double>(utilities_of_samples, dataFolder + "/eu_utilities");
	    
// 	    for(int k = 0; k<LSamples ; k++){
// 	      std::cout << "Sample:" << k+1 << "\tg:" << utilities_of_samples[k] << "\tp(g):" << probabilities[k] << std::endl; 
	      //expected_utility += utilities_of_samples[k] * probabilities[k];
	      //expected_coverage += (double)coverage_of_samples[k] * probabilities[k];
// 	    }
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
  connect_rate = (double) connect_ok / (double) nViews;
  std::cout << "Connection rate:" << connect_rate << "  " << connect_ok << "/" << nViews << std::endl;
  reader.saveData2Text<double>(connect_rate, dataFolder + "/connection_rate", true, '\t');
  
  if(success){
    std::cout << "NBVC Planning SUCCESS" << std::endl;
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
    
        
    file_name.clear();
    file_name = dataFolder + "/AlternativeGoalPose";
    pose_configuration = m->StateToConfiguration(best_sensing_goal);
    outfile->open(file_name.c_str());
    if(*outfile){
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
    
    file_name.clear();
    file_name = dataFolder + "/numControls";
    reader.saveData2Text<int>(solutionControls.size(),file_name,true,'\t');
  }
  
  pointingViews.remove(v);
  
  delete p;
  delete g;
//delete m;
  
  return success;
}

NBVPlannerExpectedUtilityFast::NBVPlannerExpectedUtilityFast(RobotSensor* rs, PartialModelBase* pm): NBVPlannerExpectedUtility(rs, pm)
{

}


bool NBVPlannerExpectedUtilityFast::init()
{
  NBVPlannerExpectedUtility::init();
  deleteUnfeasibleUnknown = true;
}


bool NBVPlannerExpectedUtilityFast::planNBV(ViewStructure& v)
{
  std::cout <<"----------- Expected Utility Fast NBVS ----------" << std::endl;
  
  //Variables
  vpFileReader reader;
  ViewList views_list;
  std::string file_name;
  bool success = false;
  double best_eval = 0;
  int connect_ok = 0;
  double connect_rate = 0.0;
  list<MSLVector> best_path;
  MSLVector best_goal;
  MSLVector best_sensing_goal;
  MSLVector init_state;
  list<MSLVector> best_policy1;
  list<MSLVector> best_policy2;
  
  savePlannerData();
  //delete g;
  Geom *g = new GeomPQP3DRigidMulti(configFolder);
  generateCandidateViews(views_list, m, g);
  
  ViewStructure best_sensing_view = views_list.getBestView();
  nbvs::stdvector2mslvector<double>(best_sensing_view.q, best_sensing_goal);

  //savePlannerData();  
  
  clock_t begin = clock();

  ViewList::iterator itv = views_list.begin();
  file_name.clear();
  file_name = configFolder + "/GoalState";
  reader.saveToMSLVector<double>(itv->q, file_name);
  Problem *p = new Problem(g, m, configFolder);
  init_state = p->InitialState;
  
  
  //Aproximar con distancia euclidiana la distancia de cada una
  for(itv = views_list.begin(); itv!= views_list.end(); itv++){
     MSLVector q(itv->q.size());
      
     for(int i = 0;i<itv->q.size(); i++)
	q[i] = itv->q[i];
      
     //double d = p->Metric(p->InitialState,q);
     //cout << "unk: " << itv->n_unknown << "--------" << std::endl;
     double d = weightedDistance(p->InitialState, q);
     double f_dist = 1 / pow((1+d),2);
     
     std::cout << "Utility: " << itv->eval << " \tDistance:" << d; 
     itv->d = d;
     itv->eval = itv->eval * f_dist;
     std::cout << " \t New utility:" << itv->eval << std::endl;
     
     //cout << "f_dist " << f_dist << std::endl;
     //itv->eval = itv->eval * f_dist;
     //cout << "----" << *itv << std::endl; 
  }
  views_list.sortHighToLow();
  
  
  std::cout << "Ordenadas-----------" << std::endl;
  for(itv = views_list.begin(); itv!= views_list.end(); itv++){ 
     std::cout << "Utility: " << itv->eval << "\t Distance: " << itv->d << "\t Unknown: " << itv->n_unknown << std::endl;
  }
  
  
  
  /**
   * Tratar de alcanzar n vistas
   */
  int i=0;
  for(itv = views_list.begin(); itv!= views_list.end() && i<nViews; itv++){
    i++; 
    
    // TODO tal vez convendría seleccionar vistas que no esten cercanas
    MSLVector goal(itv->q.size());
    
    for(int j = 0;j<itv->q.size(); j++) //todo: check this, before it has i instead j
      goal[j] = itv->q[j];
      
    p->GoalState = goal;
      
    std::cout << "\n" << i << "/" << nViews << "Tratando de alcanzar: " << p->GoalState << std::endl; 
    // if(p->CollisionFree( m->StateToConfiguration(p->GoalState) ) ){
    RRT *rrt= new RRTExtExt(p);
    rrt->PlannerDeltaT = plannerDeltaT;
    rrt->NumNodes = rrtNodes;
	
    if(rrt->Plan()){
         connect_ok ++;
	  //cout << "Alcanzado! " << std::endl;
	  //cout << "Solution Path size: " << rrt->Path.size() << "\n"; 
	  //cout << "Solution Path: \n" << rrt->Path << "\n"; 
	  //cout << "NBV: \n" << rrt->BestState << std::endl;
 	  double d = accumulatedWeightedDist(rrt->Path);   //p->Metric(p->InitialState,p->GoalState);
 	  itv->d = d;
 	  double f_dist = 1 / (1+d);
	  std::cout << "Unknown: " << itv->n_unknown;
	  std::cout << " \tDistance of the RRT Path" << d;
	  std::cout << " \tUtility with RRT Path:" << itv->n_unknown * f_dist << std::endl;
	  //cout << "after: " << itv->eval << std::endl;
	  
	  // Guardar el path para graficación
	  std::vector< std::vector<double> > original_path;
	  std::vector<double> point_of_path;
	  list< MSLVector > rrt_path;
	  list< MSLVector >::iterator it_path;
	  rrt_path = rrt->Path;
	  for(it_path = rrt_path.begin(); it_path != rrt_path.end(); it_path++){
	    nbvs::mslvector2stdvector<double>(*it_path,point_of_path);
	    original_path.push_back(point_of_path);
	  }
	
	  /// --------- ExpectedUtility --------------
	  std::cout << "Calculating expected utility" << std::endl;
	  list<MSLVector> trajectory;
	  ViewStructure Sk_view;
	  
	  // only for display purposes
	  std::vector<double> point(3);
	  // only for display purposes
	  std::vector< std::vector<double> > init_and_goal;
	  std::vector< std::vector<double> > sample_points;
	  std::vector< std::vector<double> > collision_points;
	  
	  clock_t time_eu_begin = clock();
	  
	  point[0] = init_state[0];
	  point[1] = init_state[1];
	  init_and_goal.push_back(point);
	  point[0] = goal[0];
	  point[1] = goal[1];
	  init_and_goal.push_back(point);
	  
	  //vpFileReader read;
	  //read.saveDoubleCoordinates("init_and_goal_eu.dat", points);
	  
	  combinePolycies(trajectory, rrt->Policy1, rrt->Policy2);
	  
	  ViewList lista;
	  double expected_utility = 0;
	  std::vector<double> log_probabilities(LSamples);
	  std::vector<double> utilities_of_samples(LSamples);
	  std::vector<int> coverage_of_samples(LSamples);
	  std::vector<double> probabilities(LSamples);
	  
	   
	  /*
	   * Get l samples with their utilities and probabilities
	   */
	  std::cout << "Generating samples" << std::endl;
	  for(int k = 0; k<LSamples ; k++){
	    // generar samples
	    MSLVector Sk_state;
	    double P_gk;
	    double gk;
	    //cout << "sample " << k << std::endl;
	    if(!generateSampleState(trajectory, init_state, goal, Sigmas, Sk_state, P_gk, g)){
	      //cout << "Collision of the sample state" << std::endl;
	      // there is collision
	      gk = - 1000;
	      //utilities_of_samples[k] = gk;
	      //log_probabilities[k] = P_gk;
	      // std::cout << "gk:" << gk << "  P_gk:" << P_gk << std::endl;
	      
	      point[0] = Sk_state[0];
	      point[1] = Sk_state[1];
	      collision_points.push_back(point);
	      coverage_of_samples[k] = 0;
	    } else {
	      // collision free
	      //cout << "collision free sample" << std::endl;
	      std::vector<double> X_temp;
	      nbvs::mslvector2stdvector(Sk_state, X_temp);
	      
	      point[0] = Sk_state[0];
	      point[1] = Sk_state[1];
	      sample_points.push_back(point);
	      
	      robotWithSensor->getViewFromState(Sk_view, X_temp);
	      lista.push_back(Sk_view);
	      partialModel->evaluateView(Sk_view);
	      gk = Sk_view.eval * f_dist;
	      //cout << std::endl << "sample state with evaluation" << std::endl << Sk_view << std::endl;
	      //cout << "gk:" << gk << "  P_gk:" << P_gk << std::endl;
	    }
	    
	    //getchar();
	    utilities_of_samples[k] = gk;
	    log_probabilities[k] = P_gk;
	 }
	  
	 /*
	 *************************** Compute the expected utility *****************
	 */
	 
	 
	 
	 //cout << "Return from the log domain" << std::endl;
	 for(int k = 0; k<LSamples ; k++){
	   probabilities[k] = exp(log_probabilities[k]);
	   //cout << "k: " << probabilities[k] << std::endl; 
	 }
	 
	 // Normalize the probabilities
	 double sum = 0.0;
	 for(int k = 0; k<LSamples ; k++){
	   sum  += probabilities[k];
	 }
	 //cout << "Suma: " << sum << std::endl;
	 
	 //cout << "Normalized probabilities" << std::endl;
	 double log_p_normalized;
	 for(int k = 0; k<LSamples ; k++){
	   probabilities[k] = probabilities[k] / sum;
	   //cout << k << ": " << probabilities[k] << std::endl;
	 }
	 
	 // compute expected_utility
	 expected_utility = 0.0;
	 double expected_coverage = 0.0;
	 for(int k = 0; k<LSamples ; k++){
	    //cout << "Utility:" << utilities_of_samples[k] << " prob:" << probabilities[k] << std::endl; 
	    expected_utility += utilities_of_samples[k] * probabilities[k];
	    expected_coverage += (double)coverage_of_samples[k] * probabilities[k];
	 }
	  
	 // Asignar el valor calculado
	 itv->eval = expected_utility;
	 itv->n_unknown = (int) floor(expected_coverage + 0.5);
	 //cout << *itv << std::endl;
	 std::cout << "\tExp_Coverage:" << expected_coverage << "\tExp_Utility:" << expected_utility << std::endl;
	 
	 clock_t time_eu_end = clock();
	 double elapsed_secs_eu = double(time_eu_end - time_eu_begin) / CLOCKS_PER_SEC;
	 std_eu_time = elapsed_secs_eu;
	 
	 //cout << "expected utility computed" << std::endl;
	 //getchar();
	 /// ----------------------------------------
	  
	  
	 if(itv->eval > best_eval){
	    std::cout << "New NBV founded using expected utility" << std::endl;
	    success = true;
	    v = *itv;
	    best_eval = itv->eval;
	    best_path = rrt->Path;
	    best_goal = p->GoalState;
	    best_policy1 = rrt->Policy1;
	    best_policy2 = rrt->Policy2;
	    
	    // Guardar el muestreo
	    lista.save("lista_de_muestreo.vs");
	    reader.saveDoubleCoordinates(dataFolder + "/init_and_goal_eu", init_and_goal);
	    reader.saveVectorOfVectors<double>(original_path, dataFolder + "/original_path_eu.dat");
	    reader.saveDoubleCoordinates(dataFolder + "/sample_points_eu.dat", sample_points);
	    reader.saveDoubleCoordinates(dataFolder + "/sample_points_collision.dat", collision_points);
	    reader.saveVector<double>(probabilities, dataFolder + "/eu_probabilities");
	    reader.saveVector<double>(utilities_of_samples, dataFolder + "/eu_utilities");
	    i = nViews;
	   // 	    for(int k = 0; k<LSamples ; k++){
// 	      std::cout << "Sample:" << k+1 << "\tg:" << utilities_of_samples[k] << "\tp(g):" << probabilities[k] << std::endl; 
	      //expected_utility += utilities_of_samples[k] * probabilities[k];
	      //expected_coverage += (double)coverage_of_samples[k] * probabilities[k];
// 	    }
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
  //connect_rate = (double) connect_ok / (double) nViews;
  //cout << "Connection rate:" << connect_rate << "  " << connect_ok << "/" << nViews << std::endl;
  //reader.saveData2Text<double>(connect_rate, dataFolder + "/connection_rate", true, '\t');
  
  if(success){
    std::cout << "NBVC Planning SUCCESS" << std::endl;
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
    
    
    file_name.clear();
    file_name = dataFolder + "/AlternativeGoalPose";
    pose_configuration = m->StateToConfiguration(best_sensing_goal);
    outfile->open(file_name.c_str());
    if(*outfile){
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
    file_name = dataFolder + "/euTimes";
    reader.saveData2Text<float>(std_eu_time, file_name, true);
    
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
    
    file_name.clear();
    file_name = dataFolder + "/numControls";
    reader.saveData2Text<int>(solutionControls.size(),file_name,true,'\t');
  }
  
  pointingViews.remove(v);
  
  delete p;
  delete g;
  //delete m;
  
  return success;
}


bool NBVPlannerEUFastBiased::generateCandidateViews(ViewList& views, Model* model, Geom* geometry)
{
  std::cout << "Generating candidate views" << std::endl;
  if(readCandidateViews)
    return readViews(views, model, geometry);
  
  std::vector<double> state;
  ViewStructure view;
  
  views.clear();
  
  // max iterations 
  int i =0;
  int feasible_stored = 0;
  int feasible_views = 0;
  int n_pointing_views =0;
  int n_collision_free_views = 0;
  int max_stored_view = 500;
  
  clock_t begin = clock();
  
  ViewList::iterator it;
  
  it = pointingViews.begin();
  int i_pointing=0;
  while(it!=pointingViews.end()){
    if(i_pointing<max_stored_view){
      
      if(partialModel->evaluateView(*it) == FEASIBLE_VIEW){
	  views.push_back(*it);
	  feasible_stored++;
	  it++;
      } else {
	it = pointingViews.erase(it);
      }
      
    }else{
      it = pointingViews.erase(it);
    }
    i_pointing++;
  }
  
  while(i<max_i){
    robotWithSensor->getRandomState(state);
    robotWithSensor->getViewFromState(view, state);
    
    if(partialModel->poitsToTheObject(view)){
      n_pointing_views ++;
      if(!robotWithSensor->pointsToTheRobot(view,state)){
	MSLVector q(view.q.size());
	for(int i = 0;i<view.q.size(); i++)
	  q[i] = view.q[i];
	
	// collision checking
	if(geometry->CollisionFree( model->StateToConfiguration(q) ) ){
	  n_collision_free_views ++;
	  if(partialModel->evaluateView(view) == FEASIBLE_VIEW){
	      views.push_back(view);
	      feasible_views++;
	      
	      // guardar la vista para futuras evaluaciones
	      pointingViews.push_back(view);
	  }
	}
      }
    }
   
    i++;
  }
  
  clock_t end = clock();
  float elapsed_secs = (float) (end - begin)/ (float) CLOCKS_PER_SEC;
  std_v_time = elapsed_secs;
  visionTimes.push_back(elapsed_secs);
  
  std::cout << "Pointing views " << n_pointing_views << std::endl;
  std::cout << "Collision Free views " << n_collision_free_views << std::endl;
  std::cout << "Feasible views " << feasible_views << std::endl;
  std::cout << "Feasible stored views " << feasible_stored << std::endl;
  std::cout << "New Stored views: " << pointingViews.size() << std::endl;
  std::cout << "Elapsed time:" << elapsed_secs << std::endl;
  
  pointingViews.sortHighToLow();
  views.sortHighToLow();
  
  return true;
}
