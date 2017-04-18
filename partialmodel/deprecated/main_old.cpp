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
    std::cout << "deepth: " << tree.getTreeDepth() << std::endl;
    std::cout << "nodes: " << tree.calcNumNodes() << std::endl;
    std::cout << "number of leaf nodes: " << tree.getNumLeafNodes() << std::endl;

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
    
    std::cout << "Aqui depth" << tree.getTreeDepth() << std::endl;
    
    int deepth = tree2.getTreeDepth();
    ColorOcTree::iterator it;
    
    it = tree2.begin(deepth-1);
    int i=1;
    
   std::cout << "tree occupancy thres: " << tree2.getOccupancyThres();
   std::cout << "tree occupancy thres log: " << tree2.getOccupancyThresLog();
   getchar();
    
   while(it!= tree2.end()){
      std::cout << "Node: 		" << i << std::endl;
      std::cout << "LogOdds: 	" << it->getLogOdds() << std::endl;
      std::cout << "Max child:	" << it->getMaxChildLogOdds() << std::endl;
      std::cout << "Mean child	" << it->getMeanChildLogOdds() << std::endl;
      std::cout << "Occupancy	" << it->getOccupancy() << std::endl;
      std::cout << "Value		" << it->getValue() << std::endl;
      std::cout << "Value		" << it->getValue() << std::endl;
      ColorOcTreeNode temp_node = *it;
      std::cout << "Has children	" << tree2.nodeHasChildren(&temp_node) << std::endl;
      
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
	    
	    std::cout << "New LogOdds: 	" << it->getLogOdds() << std::endl;
	    std::cout << "New Max child:	" << it->getMaxChildLogOdds() << std::endl;
	    std::cout << "New Mean child	" << it->getMeanChildLogOdds() << std::endl;
	    std::cout << "New Occupancy	" << it->getOccupancy() << std::endl;
	    std::cout << "New Value		" << it->getValue() << std::endl;
	    
	    it->deleteChild(j);
	  }
	} 
      }
      
      std::cout << std::endl; 
      i++;
      it ++;
    }

    
    tree2.write("test_occupancy_reduced.ot");
    std::cout << "new resolution" << tree.getResolution() << std::endl;
    
//     point3d_list::iterator it_pointv;
//     for (it_pointv = points_v.begin(); it_pointv != points_v.end(); it_pointv++){
//       ColorOcTreeNode *node = tree.updateNode(*it_pointv, false);
//       std::cout << "logOdds " << node->getLogOdds() << std::endl;
//       ColorOcTreeNode::Color c(255,255,0);
//       node->setColor(c);
//       //node->setLogOdds(-0.1);
//       std::cout << node->getLogOdds() << std::endl;
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
//       std::cout << "logOdds " << node->getLogOdds() << std::endl;
//       ColorOcTreeNode::Color c(255,255,0);
//       node->setColor(c);
//       node->setLogOdds(0.0);
//       std::cout << node->getLogOdds() << std::endl;
//     }
//     tree.updateInnerOccupancy(); 
//     
    
    /******************* **************************/
    std::cout << "Ready to remove children" << std::endl;
    
    ColorOcTree ot2(0.8);
    
    ot2.insertRay(o,p);
    
    // copiar unknown children
    point3d_list::iterator it_point;
    for (it_point = unknown_points.begin(); it_point != unknown_points.end(); it_point++){
      //cout << it_point->x() << " " << it_point->y() << " " << it_point->z() << std::endl;
      ColorOcTreeNode *node = ot2.updateNode(*it_point, true);
      //cout << "logOdds " << node->getLogOdds() << std::endl;
      ColorOcTreeNode::Color c(255,255,0);
      node->setColor(c);
      node->setLogOdds(0.0);
      //cout << node->getLogOdds() << std::endl;
     }
     
     ot2.updateInnerOccupancy(); 
    
     point3d hit;
     point3d dir(-2,0,0);
     
     if(ot2.castRay(o,dir,hit)){
       std::cout << "occupied" << std::endl;
       std::cout << hit.x() << " " << hit.y() << " " << hit.z() << std::endl;
     }
     else
       std::cout << "uknown" << std::endl;
     
     
     
//     int deepth = ot2.getTreeDepth();
//     ColorOcTree::iterator it;
//     it = ot2.begin(deepth-1);
//     int i=1;
//     
//     while(it!= tree.end()){
//       std::cout << "Node:" << i << std::endl;
//       i++;
//       std::cout << it->getLogOdds() << std::endl;
//       std::cout << it->getMaxChildLogOdds() << std::endl;
//       std::cout << it->getMeanChildLogOdds() << std::endl;
//       std::cout << it->getOccupancy() << std::endl;
//       std::cout << it->getValue() << std::endl;
//       it ++;
//     }
    
    ot2.write("test_occupancy_2.ot");
    
    return 0;
}
