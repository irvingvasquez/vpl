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


#include "nbvplannerrgflt.h"


NBVPlannerRGFlt::NBVPlannerRGFlt(RobotSensor* rs, PartialModelBase* pm): NBVPlanner(rs, pm)
{
  
}


NBVPlannerRGFlt::~NBVPlannerRGFlt()
{
  //delete g;
  delete m;
}


double NBVPlannerRGFlt::accumulatedWeightedDist(list< MSLVector >& path)
{

  double d = 0;
  
  list< MSLVector >::iterator a = path.begin();
  list< MSLVector >::iterator b = path.begin();
  b++;
  while(b != path.end()){
    d += weightedDistance(*a, *b);
    a = b;
    b++;
  }
  
  return d;
}



double NBVPlannerRGFlt::weightedDistance(const MSLVector& x1, const MSLVector& x2)
{
  double rho;
  MSLVector dtheta( x1.dim());
  int i;
  
  rho = 0.0;
  double sum=0.0;
  
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
  
  return sqrt(sum);

}


bool NBVPlannerRGFlt::planNBV(ViewStructure& v)
{
  vpFileReader reader;
  ViewList views_list;
  std::string file_name;
  
  //m = new Model3DRigidEVAmetricJ(configFolder);
  
  savePlannerData();
  Geom *g = new GeomPQP3DRigidMulti(configFolder);
  generateCandidateViews(views_list, m, g);
  ViewStructure best_sensing_view = views_list.getBestView();

  std::cout <<"-----------Motion Planning ----------" << std::endl;  
  clock_t begin = clock();

  bool success = false;
  
  double best_eval = 0;
  list<MSLVector> best_path;
  MSLVector best_goal;
  MSLVector init_state;
  list<MSLVector> best_policy1;
  list<MSLVector> best_policy2;

  ViewList::iterator itv = views_list.begin();
  file_name.clear();
  file_name = configFolder + "/GoalState";
  reader.saveToMSLVector<double>(itv->q, file_name);
  Problem *p = new Problem(g, m, configFolder);
  
// Aproximar con distancia euclidiana la distancia de cada una
//   for(itv = views_list.begin(); itv!= views_list.end(); itv++){
//      MSLVector q(itv->q.size());
//       
//      for(int i = 0;i<itv->q.size(); i++)
// 	q[i] = itv->q[i];
//       
//      //double d = p->Metric(p->InitialState,q);
//      //cout << "unk: " << itv->n_unknown << "--------" << std::endl;
//      double d = weightedDistance(p->InitialState, q);
//      double f_dist = 1 / (1+d);
//      
//      std::cout << "Utility: " << itv->eval << " \tDistance:" << d; 
//      itv->d = d;
//      itv->eval = itv->eval * f_dist;
//      std::cout << " \t New utility:" << itv->eval << std::endl;
//      
//      //cout << "f_dist " << f_dist << std::endl;
//      //itv->eval = itv->eval * f_dist;
//      //cout << "----" << *itv << std::endl; 
//   }
//   views_list.sortHighToLow();
  
  
  /**
   * Tratar de alcanzar n vistas
   */
  int i=0;
  for(itv = views_list.begin(); itv!= views_list.end() && i<nViews; itv++){
    
    // TODO tal vez convendría seleccionar vistas que no esten cercanas
    i++; 
    
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
	  std::cout << "Solution Path size: " << rrt->Path.size(); 
	  //cout << "Solution Path: \n" << rrt->Path << "\n"; 
	  //cout << "NBV: \n" << rrt->BestState << std::endl;
	  double d = accumulatedWeightedDist(rrt->Path);   //p->Metric(p->InitialState,p->GoalState);
	  itv->d = d;
	  double f_dist = 1 / (1+d);
	  //cout << "before: " << itv->eval << std::endl;
	  std::cout << "\t Distance:" << d;
	  itv->eval = itv->eval * f_dist;
	  std::cout << "\t Evaluation: " << itv->eval << std::endl;
	  
	  if(itv->eval > best_eval){
	    v = *itv;
	    best_eval = itv->eval;
	    best_path = rrt->Path;
	    best_goal = p->GoalState;
	    init_state = p->InitialState;
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
    std_distance = 0; //TODO
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
//   delete m;
  
  return success;
}


bool NBVPlannerRGFlt::init()
{
  NBVPlanner::init();
     
  //TODO Read parameters
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("plannerConfig.ini");
  
  dictionary * ini_file;
    
  ini_file = iniparser_load(config_file.c_str());
  if (ini_file ==NULL ) {
      fprintf(stderr, "cannot parse file: %s\n", config_file.c_str());
      return false ;
  }
  //iniparser_dump(ini_file, stderr);
  
  nViews = iniparser_getint(ini_file, "NBVPlanner:nViews", 0);
  max_i = iniparser_getint(ini_file, "NBVPlanner:maxI", 3000);
  
  readCandidateViews = iniparser_getboolean(ini_file, "NBVPlanner:readCandidateViews", false);
  
  readOnce = iniparser_getboolean(ini_file, "NBVPlanner:readOnce", false);
  alreadyReadedOnce = false;
  deleteUnfeasibleUnknown = false;
  
  viewsFile.clear();
  viewsFile.assign(iniparser_getstring(ini_file, "NBVPlanner:viewsFile", ""));
  std::cout << "Read Candidate views:" << readCandidateViews << std::endl;
  if(readCandidateViews)
    std::cout << "Views File:" << viewsFile.c_str() << std::endl;
    
  std::cout << "---------- NBV Planner Rand C space -------" << std::endl;
  std::cout << "Views: " << nViews << std::endl;
  std::cout << "maxI: " << max_i << std::endl;
  
  // TODO read from configuration nuevo modelo
  m = new Model3DRigidEVATR(configFolder);
  
  Interval = m->UpperState - m->LowerState;
  std::cout << "Interval: " << Interval << std::endl;
  
  vpFileReader reader;
  config_file.clear();
  config_file.append(configFolder);
  config_file.append("/Weights");
  reader.readMSLVector<double>(Weights, config_file);
  std::cout << "Weights: " ;
  PMUtils::printVector(Weights);
  std::cout << std::endl;
  
  return true;
}


bool NBVPlannerRGFlt::readViews(ViewList& views, Model* model, Geom* geometry)
{
  views.clear();
  
  // if not only once time the lis should be readed then read all the time
  if(!readOnce){
    std::cout << "Reading candidate views" << std::endl;
    if(!pointingViews.read(viewsFile))
      exit(0);
  } else {
    // if none time the views have been read then read the file
    if( (!alreadyReadedOnce) || (pointingViews.size() == 0)){
      std::cout << "Reading candidate views" << std::endl;
      if(!pointingViews.read(viewsFile))
	exit(0);
      
      alreadyReadedOnce = true;
    }
  }
  
  std::cout << "Evaluating views" << std::endl;
  
  // max iterations 
  int i =0;
  int feasible_views = 0;
  int collision_views = 0;
  int unfeasible_unknown = 0;
  bool delete_view = false;
  
  clock_t begin = clock();
  
  ViewList::iterator it;
  
  it = pointingViews.begin();
  while(it!=pointingViews.end()){
    MSLVector q(it->q.size());
    for(int i = 0; i < it->q.size(); i++)
      q[i] = it->q[i];
    
    if(geometry->CollisionFree( model->StateToConfiguration(q) ) ){
      if(partialModel->evaluateView(*it) == FEASIBLE_VIEW){
	  views.push_back(*it);
	  feasible_views++;
      } else {
	if(deleteUnfeasibleUnknown){
	  if((*it).n_unknown == 0){
	    delete_view = true;
	    unfeasible_unknown ++;
	  }
	}	
      }
    } else {
      collision_views ++;
    }
    
    if(delete_view)
    {
      it = pointingViews.erase(it);
      delete_view = false;
    } else {
      it++;
    }
  }
  
  clock_t end = clock();
  float elapsed_secs = (float) (end - begin)/ (float) CLOCKS_PER_SEC;
  std_v_time = elapsed_secs;
  visionTimes.push_back(elapsed_secs);
  
  std::cout << "Candidate views: " << pointingViews.size() << std::endl;
  std::cout << "Feasible: " << feasible_views 
      << " \t In collision:" << collision_views 
      << "\t ZeroUnknown:" << unfeasible_unknown << std::endl;
 
  std::cout << "Elapsed time:" << elapsed_secs << std::endl;
  
  views.sortHighToLow();
  
  return true;
}


bool NBVPlannerRGFlt::generateCandidateViews(ViewList& views, Model* model, Geom* geometry)
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



NBVPlannerRGFltOneGoal::NBVPlannerRGFltOneGoal(RobotSensor* rs, PartialModelBase* pm): NBVPlannerRGFlt(rs, pm)
{

}


bool NBVPlannerRGFltOneGoal::planNBV(ViewStructure& v)
{
  vpFileReader reader;
  ViewList views_list;
  std::string file_name;
  
  savePlannerData();
  Geom *g = new GeomPQP3DRigidMulti(configFolder);
  
  generateCandidateViews(views_list, m, g);
  ViewStructure best_sensing_view = views_list.getBestView();
 
  std::cout <<"-----------Motion Planning ----------" << std::endl;
  //savePlannerData();
  
  clock_t begin = clock();

  bool success = false;
  
  //double best_eval = 0;
  list<MSLVector> best_path;
  MSLVector best_goal;
  MSLVector init_state;
  list<MSLVector> best_policy1;
  list<MSLVector> best_policy2;

  ViewList::iterator itv = views_list.begin();
  file_name.clear();
  file_name = configFolder + "/GoalState";
  reader.saveToMSLVector<double>(itv->q, file_name);
  Problem *p = new Problem(g, m, configFolder);
  
  
  // Aproximar con distancia euclidiana la distancia de cada una
  for(itv = views_list.begin(); itv!= views_list.end(); itv++){
     MSLVector q(itv->q.size());
      
     for(int i = 0;i<itv->q.size(); i++)
	q[i] = itv->q[i];
      
     //double d = p->Metric(p->InitialState,q);
     //cout << "unk: " << itv->n_unknown << "--------" << std::endl;
     double d = weightedDistance(p->InitialState, q);
     double f_dist = 1 / (1+d);
     //cout << "f_dist " << f_dist << std::endl;
     itv->d = d;
     itv->eval = itv->eval * f_dist;
     //cout << "----" << *itv << std::endl; 
  }
  views_list.sortHighToLow();
  
  
  itv = views_list.begin();
  int k=0;
  while(!success && itv!= views_list.end() && k<nViews ){
      //cout << "Selected. " << *itv << std::endl;
    
      k++;
      MSLVector goal(itv->q.size());
      for(int i = 0;i<itv->q.size(); i++)
      goal[i] = itv->q[i];
      
      p->GoalState = goal;
      
      RRT *rrt= new RRTExtExt(p);
      rrt->PlannerDeltaT = plannerDeltaT;
      rrt->NumNodes = rrtNodes;
	
      if(rrt->Plan()){
	  success = true;
	  
	  v = *itv;
	  std_utility = (float) itv->eval;
	  std_distance_uf = (float) 1 / (1+itv->d);
	  std_surface_uf = (float) itv->n_unknown;
	  
	  best_path = rrt->Path;
	  best_goal = p->GoalState;
	  init_state = p->InitialState;
	  best_policy1 = rrt->Policy1;
	  best_policy2 = rrt->Policy2;
	  
	} else {
	  std::cout << "No path found" << std::endl;
	}
	
      delete rrt;
      
      itv++;
  }
   
  clock_t end = clock(); 
  std_mp_time = float(end - begin) / CLOCKS_PER_SEC;
  motionPTimes.push_back(std_mp_time);
  std::cout << "Motion Planing elapsed time: " << std_mp_time << std::endl;
  
  
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
    //std_distance = accumulatedDistance(best_path, m);
    std_distance = 0; // TODO
    std_accu_distance += std_distance;
    distances_per_it.push_back(std_distance);
    
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
  
  delete g;
  delete p;
  
  return success;
}

void NBVPlannerRGFlt::copySolution(list< MSLVector > path, list< MSLVector > policy1, list< MSLVector > policy2)
{
  //cout << "Copiying solution" << std::endl;
  
  // path
//   std::cout << "path" << std::endl;
  solutionPath.clear();
  std::vector<double> q;
  
  // we do not consider the first element because it is the initial configuration
  list<MSLVector>::iterator it_path = path.begin();
  it_path ++;
  while(it_path!=path.end()){
  //for( it!= path.end(); it++){
    nbvs::mslvector2stdvector<double>(*it_path, q);
//     PMUtils::printVector(q);
//     i++;
    solutionPath.push_back(q);

    it_path ++;
  }
  
  // policy1
//   std::cout << "policy 1" << std::endl;
  std::vector<double> u;
  solutionControls.clear();
  for(list<MSLVector>::iterator it = policy1.begin() ; it!= policy1.end(); it++){
    nbvs::mslvector2stdvector<double>(*it, u);
//     PMUtils::printVector(u);
//     i++;
    solutionControls.push_back(u);
  }
  
  //insert an empty control to start with the second policy
  std::vector<double> empty_control(m->InputDim);
  //PMUtils::printVector(empty_control);
  solutionControls.push_back(empty_control);
  
  //policy 2

  if(policy2.size()!=0){
//     std::cout << "policy 2" << std::endl;
    for(list<MSLVector>::iterator it = policy2.begin() ; it!= policy2.end(); it++){
      nbvs::mslvector2stdvector<double>(*it, u);
//       PMUtils::printVector(u);
      if(u.size()!=0){
	solutionControls.push_back(u);
      }
    }
  }
  
}



void NBVPlannerRGFlt::copySingleSolution(list< MSLVector > path, list< MSLVector > policy1)
{
  //cout << "copiying solution" << std::endl;
  
//   std::cout << "path" << std::endl;
  solutionPath.clear();
  std::vector<double> q;
  
  // we do not consider the first element because it is the initial configuration
  list<MSLVector>::iterator it_path = path.begin();
  it_path ++;
  while(it_path!=path.end()){
    //for( it!= path.end(); it++){
    nbvs::mslvector2stdvector<double>(*it_path, q);
//     PMUtils::printVector(q);
//     i++;
    solutionPath.push_back(q);

    it_path ++;
  }
  
// policy1
//   std::cout << "policy 1" << std::endl;
  std::vector<double> u;
  solutionControls.clear();
  for(list<MSLVector>::iterator it = policy1.begin() ; it!= policy1.end(); it++){
    nbvs::mslvector2stdvector<double>(*it, u);
//     PMUtils::printVector(u);
    solutionControls.push_back(u);
  }
}



NBVPlannerExtendTree::NBVPlannerExtendTree(RobotSensor* rs, PartialModelBase* pm): NBVPlannerRGFlt(rs, pm)
{
   
}




bool NBVPlannerExtendTree::planNBV(ViewStructure& v)
{

  vpFileReader reader;
  ViewList views_list;
  std::string file_name;
  
  //generateCandidateViews(views_list, m, g);
  //ViewStructure best_sensing_view = views_list.getBestView();

  std::cout <<"-----------Motion Planning with node evaluation ----------" << std::endl;
  
  savePlannerData();
  Geom *g = new GeomPQP3DRigidMulti(configFolder);
  
  clock_t begin = clock();

  bool success = false;
  
  double best_eval = 0;
  list<MSLVector> best_path;
  MSLVector best_goal;
  MSLVector init_state;
  list<MSLVector> best_policy1;
  list<MSLVector> best_policy2;

  ViewList::iterator itv = views_list.begin();
  file_name.clear();
  file_name = configFolder + "/GoalState";
  reader.saveToMSLVector<double>(itv->q, file_name);
  Problem *p = new Problem(g, m, configFolder);
  
  //RRT *rrt= new RRTExtExt(p);
  RRTNBV *rrt = new RRTNBV(p, configFolder, dataFolder);
  rrt->setPartialModel(partialModel);
  rrt->setRobotWithSensor(robotWithSensor);
  
  rrt->PlannerDeltaT = plannerDeltaT;
  rrt->NumNodes = rrtNodes;
  rrt->UseANN = false;
  
  if(rrt->Plan()){
    success = true;
    //cout << "Solution Path: \n" << rrt->Path << "\n"; 
    //cout << "NBV: \n" << rrt->BestState << std::endl;

    v = rrt->getNBV();
    
    std_utility = (float) v.eval;
    std_distance_uf = (float)  1 / (1+v.d);
    std_surface_uf = (float) v.n_unknown;
    
    best_path = rrt->Path;
    best_goal = rrt->BestState;
    init_state = p->InitialState;
    best_policy1 = rrt->Policy1;
    best_policy2 = rrt->Policy2;
  } else {
    std::cout << "No path found" << std::endl;
  }
  delete rrt; 
  
  
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
    file_name.clear();
    file_name = dataFolder + "/Path";
    
    outfile->open(file_name.c_str()); 
    if (*outfile) {   
      *outfile << best_path;
      outfile->close();
    }
    
    //guardar pose path
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
    //std_distance = accumulatedDistance(best_path, m);
    std_distance = 0; //TODO
    std_accu_distance += std_distance;
    distances_per_it.push_back(std_distance);
    
    file_name.clear();
    file_name = dataFolder + "/traveledDistance";
    reader.saveVector<double>(distances_per_it, file_name);
    
    copySingleSolution(best_path, best_policy1);
    //saveToLogFile();
    
    file_name.clear();
    file_name = dataFolder + "/numControls";
    reader.saveData2Text<int>(solutionControls.size(),file_name,true,'\t');
  }
  
  //pointingViews.remove(v);
  
  delete p;
  delete g;
  
  return success;
}




