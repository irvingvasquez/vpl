#include <iostream>
#include "nbvPartialModelConfig.h"

#include "partialmodelbase.h"
#include <octomap/octomap.h>

int main(int argc, char **argv) {
    std::cout << "Hello, This is the partial model library" << std::endl;
    std::cout << "Version " << pm_VERSION_MAJOR << "." << pm_VERSION_MINOR << std::endl;
    
    double res = 0.2;  // create empty tree with resolution 0.05 (different from default 0.1 for test)
    ColorOcTree tree (res);
    
    // insert some measurements of occupied cells
    octomap::point3d o(2,0,0);
    octomap::point3d p(0,0,0);
    
    tree.insertRay(o,p);
    //tree2.insertRay(o,p);    
    
    //  
    cout << "deepth: " << tree.getTreeDepth() << endl;
    cout << "nodes: " << tree.calcNumNodes() << endl;
    cout << "number of leaf nodes: " << tree.getNumLeafNodes() << endl;

    ////////////// Cuadro vacío
    point3d cv1(-1,-1,-1);
    point3d cv2(1,1,1);
    point3d_list unknown_points;
    tree.getUnknownLeafCenters(unknown_points, cv1, cv2);
    tree.updateInnerOccupancy();
    tree.write("test_occupancy.ot");
    
    ColorOcTree tree2(tree);
    tree2.setResolution(2*res);

    tree2.updateInnerOccupancy();
    tree2.prune();
    
    cout << "Aqui depth" << tree.getTreeDepth() << endl;
    
    int deepth = tree2.getTreeDepth();
    ColorOcTree::iterator it;
    
    it = tree2.begin(deepth-1);
    int i=1;
    
   cout << "tree occupancy thres: " << tree2.getOccupancyThres();
   cout << "tree occupancy thres log: " << tree2.getOccupancyThresLog();
   getchar();
    
   while(it!= tree2.end()){
      cout << "Node: 		" << i << endl;
      cout << "LogOdds: 	" << it->getLogOdds() << endl;
      cout << "Max child:	" << it->getMaxChildLogOdds() << endl;
      cout << "Mean child	" << it->getMeanChildLogOdds() << endl;
      cout << "Occupancy	" << it->getOccupancy() << endl;
      cout << "Value		" << it->getValue() << endl;
      cout << "Value		" << it->getValue() << endl;
      cout << "Has children	" << it->hasChildren() << endl;
      
      // si esta ocupado elimina todos sus hijos??
      if(it->getOccupancy() > 0.5){
	for(unsigned int j = 0; j<8; j++){
	  if(it->childExists(j))
	  {
	    it->deleteChild(j);
	  }
	}
      }
      // si no esta ocupado
      else {
	for(unsigned int j = 0; j<8; j++){
	  if(it->childExists(j)) {
	      it->deleteChild(j);
	  }
	  // si alguno de sus hijos no existe significa que tiene almenos un unknown por lo tanto la probabilidad de ocupación debe ser de 0
	  else {
	    // creamos ese hijo con probabilidad 0.5 y luego actualizamos este mismo nodo
	    it->createChild(j);
	    ColorOcTreeNode *n = it->getChild(j);
	    n->setLogOdds(0.0);
	    n->setValue(0.0);
	    it->updateOccupancyChildren();
	    it->setColor(255,255,0);
	    
	    cout << "New LogOdds: 	" << it->getLogOdds() << endl;
	    cout << "New Max child:	" << it->getMaxChildLogOdds() << endl;
	    cout << "New Mean child	" << it->getMeanChildLogOdds() << endl;
	    cout << "New Occupancy	" << it->getOccupancy() << endl;
	    cout << "New Value		" << it->getValue() << endl;
	    
	    it->deleteChild(j);
	  }
	} 
      }
      
      cout << endl; 
      i++;
      it ++;
    }

    
    tree2.write("test_occupancy_reduced.ot");
    cout << "new resolution" << tree.getResolution() << endl;
    
//     point3d_list::iterator it_pointv;
//     for (it_pointv = points_v.begin(); it_pointv != points_v.end(); it_pointv++){
//       ColorOcTreeNode *node = tree.updateNode(*it_pointv, false);
//       cout << "logOdds " << node->getLogOdds() << endl;
//       ColorOcTreeNode::Color c(255,255,0);
//       node->setColor(c);
//       //node->setLogOdds(-0.1);
//       cout << node->getLogOdds() << endl;
//     }
// 
//     
     
    
    /////////////// cuadro unknown
//     point3d c1(-0.6,-0.6,-0.6);
//     point3d c2(0.6,0.6,0.6);
//     point3d_list points;
//     tree.getUnknownLeafCenters(points, c1, c2);
//     
//     point3d_list::iterator it_point;
//     for (it_point = points.begin(); it_point != points.end(); it_point++){
//       ColorOcTreeNode *node = tree.updateNode(*it_point, true);
//       cout << "logOdds " << node->getLogOdds() << endl;
//       ColorOcTreeNode::Color c(255,255,0);
//       node->setColor(c);
//       node->setLogOdds(0.0);
//       cout << node->getLogOdds() << endl;
//     }
//     tree.updateInnerOccupancy(); 
//     
    
    /******************* **************************/
    cout << "Ready to remove children" << endl;
    
    ColorOcTree ot2(0.8);
    
    ot2.insertRay(o,p);
    
    // copiar unknown children
    point3d_list::iterator it_point;
    for (it_point = unknown_points.begin(); it_point != unknown_points.end(); it_point++){
      //cout << it_point->x() << " " << it_point->y() << " " << it_point->z() << endl;
      ColorOcTreeNode *node = ot2.updateNode(*it_point, true);
      //cout << "logOdds " << node->getLogOdds() << endl;
      ColorOcTreeNode::Color c(255,255,0);
      node->setColor(c);
      node->setLogOdds(0.0);
      //cout << node->getLogOdds() << endl;
     }
     
     ot2.updateInnerOccupancy(); 
    
     point3d hit;
     point3d dir(-2,0,0);
     
     if(ot2.castRay(o,dir,hit)){
       cout << "occupied" << endl;
       cout << hit.x() << " " << hit.y() << " " << hit.z() << endl;
     }
     else
       cout << "uknown" << endl;
     
     
     
//     int deepth = ot2.getTreeDepth();
//     ColorOcTree::iterator it;
//     it = ot2.begin(deepth-1);
//     int i=1;
//     
//     while(it!= tree.end()){
//       cout << "Node:" << i << endl;
//       i++;
//       cout << it->getLogOdds() << endl;
//       cout << it->getMaxChildLogOdds() << endl;
//       cout << it->getMeanChildLogOdds() << endl;
//       cout << it->getOccupancy() << endl;
//       cout << it->getValue() << endl;
//       it ++;
//     }
    
    ot2.write("test_occupancy_2.ot");
    
    return 0;
}
