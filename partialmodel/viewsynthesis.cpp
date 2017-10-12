/*
 * Copyright (c) 2017, <copyright holder> <email>
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

#include "viewsynthesis.h"

ViewSynthesis::ViewSynthesis()
{
  n_max = -1;
}


RandomViewSynthesis::RandomViewSynthesis(int n, ViewStructure min, ViewStructure max):ViewSynthesis()
{
  n_max = n;
  min_view = min;
  max_view = max;
}


void RandomViewSynthesis::getViews(ViewList& views)
{
  views.clear();
  
  ViewStructure view;
  double random, interval, rand_interval;
  std::vector<double> pose(6);
  
  srand (time(NULL));
  
  for(int i=0;i<n_max;i++){
    for(int j=0;j<6;j++){
      random = (double) rand()/RAND_MAX; 
      interval = max_view.w[j] - min_view.w[j];
      rand_interval = random * interval;
      // random pose
      pose[j] = min_view.w[j] + rand_interval;
      // in this case the configuration is the same than the pose
      // view.q = view.w;
    }
    
    view.setPose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    views.push_back(view);
  }
}


ViewSphereSynthesis::ViewSphereSynthesis(double r, double x, double y, double z, int level): ViewSynthesis()
{
  radius = r;
  cx = x;
  cy = y;
  cz = z;
  tesselation_level = level;
  
  vertices = NULL;
  faces = NULL;
  
  start = NULL;
  end = NULL;
  midpoint = NULL;
}


int ViewSphereSynthesis::getSpherePositionsByTesselation(int level, std::vector< std::vector< double > >& points)
{
  
  initIcosahedro();
  
  for (int i=0; i<tesselation_level; i++) 
    subdivide (); 
  
  copySphere(points);
  //output_sphere (argv[3]);
  return points.size();
}


void ViewSphereSynthesis::getViews(ViewList& views)
{
  views.clear();
  
  ViewStructure view;
  
  std::vector< std::vector<double> > points;
  n_max =  getSpherePositionsByTesselation(tesselation_level, points);
  
  getPointedViews(points, views);
}


void ViewSphereSynthesis::getPointedViews(std::vector< std::vector< double > > points, ViewList& views)
{
  views.clear();
  ViewStructure view;
  
  std::vector< std::vector<double> >::iterator point_it;
  
  /// unit sphere point
  std::vector<double> usp(3); 
  
  /// view sphere point
  std::vector<double> vsp(3);
  
  std::vector<double> pointing_v(3);
  
  std::vector<double> coordinates(6);
  
  double yaw, pitch, roll;
  double norm;
  
  for(point_it = points.begin(); point_it != points.end(); point_it++){
    if(point_it->size() != 3){
      std::cout << "Error in points " << std::endl;
      exit(0);
    }
    
    usp[0] = (*point_it)[0];
    usp[1] = (*point_it)[1];
    usp[2] = (*point_it)[2];
    
    // expander el punto al radio indicado; 
    usp[0] = radius * usp[0];
    usp[1] = radius * usp[1];
    usp[2] = radius * usp[2];
        

    vsp[0] = cx + usp[0];
    vsp[1] = cy + usp[1];
    vsp[2] = cz + usp[2];
    
    // calcular el vector que apunta al objeto
    pointing_v[0] = cx - vsp[0];
    pointing_v[1] = cy - vsp[1];
    pointing_v[2] = cz - vsp[2];
    
    // calcular los angulos de rotación
    yaw = atan2(pointing_v[1], pointing_v[0]);
    
    norm = sqrt( pow(pointing_v[0],2) + pow(pointing_v[1],2) + pow(pointing_v[2],2) );
    //norm = radio;
    pitch = asin( pointing_v[2] / norm);
    pitch = -pitch; // I did this because the positive angles lie before the x-y plane.  
    roll = 0;
   
    // determinar la configuración
    view.w[0] = (double) vsp[0];
    view.w[1] = (double) vsp[1];
    view.w[2] = (double) vsp[2];
    view.w[3] = (double) (yaw);
    view.w[4] = (double) (pitch);
    view.w[5] = (double) (roll);
    
    view.q = view.w;
    // determinar la matriz de transformación homogenea
    view.setPose(view.w[0], view.w[1], view.w[2], view.w[3], view.w[4], view.w[5]);
//     mrpt::poses::CPose3D pose(view.w[0], view.w[1], view.w[2], view.w[3], view.w[4], view.w[5]);
//     mrpt::math::CMatrixDouble44 htm_p;
//     pose.getHomogeneousMatrix(htm_p);
//   
//     view.HTM(0,0) = htm_p(0,0);
//     view.HTM(0,1) = htm_p(0,1); 
//     view.HTM(0,2) = htm_p(0,2); 
//     view.HTM(0,3) = htm_p(0,3);
//     
//     view.HTM(1,0) = htm_p(1,0); 
//     view.HTM(1,1) = htm_p(1,1); 
//     view.HTM(1,2) = htm_p(1,2);
//     view.HTM(1,3) = htm_p(1,3);
//     
//     view.HTM(2,0) = htm_p(2,0);
//     view.HTM(2,1) = htm_p(2,1);
//     view.HTM(2,2) = htm_p(2,2);
//     view.HTM(2,3) = htm_p(2,3);
//     
//     view.HTM(3,0) = htm_p(3,0);
//     view.HTM(3,1) = htm_p(3,1);
//     view.HTM(3,2) = htm_p(3,2);
//     view.HTM(3,3) = htm_p(3,3);
    
    views.push_back(view);
  }
}


void ViewSphereSynthesis::copySphere(std::vector< std::vector< double > >& points)
{
  int i, j; 
  points.clear();

  /* Calcular el centroide de las caras*/
  int i_vertice_a;
  int i_vertice_b;
  int i_vertice_c;
  float vertice_a[3];
  float vertice_b[3];
  float vertice_c[3];
  float centroide[3];
  float **vistas = NULL;
  
  vistas = mFMatrix(0,n_faces-1,0,2);

  for (i=0; i<n_faces; i++){
     i_vertice_a = faces[3*i];
     for (j=0;j<3;j++)
        vertice_a[j] = vertices[3*i_vertice_a+j];

     i_vertice_b = faces[3*i+1];
     for (j=0;j<3;j++)
        vertice_b[j] = vertices[3*i_vertice_b+j];

     i_vertice_c = faces[3*i+2];
     for (j=0;j<3;j++)
        vertice_c[j] = vertices[3*i_vertice_c+j];

     for (j=0;j<3;j++){
        centroide[j] = vertice_a[j] + vertice_b[j] + vertice_c[j];
        centroide[j] = centroide[j] / 3;  
     }

     for (j=0;j<3;j++){
        vistas[i][j]=centroide[j];
     }
  }

  /* */
  float length;
  std::vector<double> point(3);
  
  for (i=0; i<n_faces; i++){
    //Ajustar puntos en la superficie de la esfera
    length = sqrt (vistas[i][0] * vistas[i][0] +
		       vistas[i][1] * vistas[i][1] +
		       vistas[i][2] * vistas[i][2]);
    length = 1/length;
    vistas[i][0] *= length;
    vistas[i][1] *= length;
    vistas[i][2] *= length;

    //escribir los puntos en el vector
    point[0] = vistas[i][0];
    point[1] = vistas[i][1];
    point[2] = vistas[i][2];
    points.push_back(point);
    //fprintf(ptr, "%f %f %f\n", vistas[i][0], vistas[i][1], vistas[i][2]);  
  }

  mFreeFMatrix(vistas,0,n_faces-1,0,2);
  // fclose(ptr);
}


float** ViewSphereSynthesis::mFMatrix(int nrl, int nrh, int ncl, int nch)
{
    int i;
    float **m;
    
    m=(float **) malloc((unsigned) (nrh-nrl+1)*sizeof(float *));
    if (!m) printf("allocation failure 1 in mFMatrix()");
    m -= nrl;
    
    for (i=nrl; i<=nrh; i++)
      {
	m[i]=(float *) malloc((unsigned) (nch-ncl+1)*sizeof(float));
	if (!m[i]) printf("allocation failure 2 in mFMatrix()");
	m[i] -= ncl;
      }
    return m;
}


void ViewSphereSynthesis::mFreeFMatrix(float** m, int nrl, int nrh, int ncl, int nch)
{
    int i;
    
    for (i=nrh; i>=nrl; i--) free((char *) (m[i]+ncl));
      free((char*) (m+nrl));
}



void ViewSphereSynthesis::subdivide()
{
//  std::cout << "subdividing" << std::endl;
  int n_vertices_new = n_vertices+2*n_edges; 
  int n_faces_new = 4*n_faces; 
  int i; 

  edge_run = 0; 
  n_edges = 2*n_vertices + 3*n_faces; 
  start = (int*)malloc(n_edges*sizeof (int)); 
  end = (int*)malloc(n_edges*sizeof (int)); 
  midpoint = (int*)malloc(n_edges*sizeof (int)); 

  int *faces_prev = (int*)malloc (3*n_faces*sizeof(int)); 
  faces_prev = (int*)memcpy((void*)faces_prev, (void*)faces, 3*n_faces*sizeof(int)); 
  vertices = (float*)realloc ((void*)vertices, 3*n_vertices_new*sizeof(float)); 
  faces = (int*)realloc ((void*)faces, 3*n_faces_new*sizeof(int)); 
  n_faces_new = 0; 

  for (i=0; i<n_faces; i++) 
    { 
      int a = faces_prev[3*i]; 
      int b = faces_prev[3*i+1]; 
      int c = faces_prev[3*i+2]; 

      int ab_midpoint = midpointSearch (b, a); 
      int bc_midpoint = midpointSearch (c, b); 
      int ca_midpoint = midpointSearch (a, c); 

      faces[3*n_faces_new] = a; 
      faces[3*n_faces_new+1] = ab_midpoint; 
      faces[3*n_faces_new+2] = ca_midpoint; 
      n_faces_new++; 
      faces[3*n_faces_new] = ca_midpoint; 
      faces[3*n_faces_new+1] = ab_midpoint; 
      faces[3*n_faces_new+2] = bc_midpoint; 
      n_faces_new++; 
      faces[3*n_faces_new] = ca_midpoint; 
      faces[3*n_faces_new+1] = bc_midpoint; 
      faces[3*n_faces_new+2] = c; 
      n_faces_new++; 
      faces[3*n_faces_new] = ab_midpoint; 
      faces[3*n_faces_new+1] = b; 
      faces[3*n_faces_new+2] = bc_midpoint; 
      n_faces_new++; 
    } 
  n_faces = n_faces_new; 
  
  free (start); 
  free (end); 
  free (midpoint); 
  free (faces_prev); 
}


int ViewSphereSynthesis::midpointSearch(int index_start, int index_end)
{
  int i;
  for (i=0; i<edge_run; i++) 
    if ((start[i] == index_start && end[i] == index_end) || 
	(start[i] == index_end && end[i] == index_start)) 
      {
	int res = midpoint[i];

	/* update the arrays */
	start[i]    = start[edge_run-1];
	end[i]      = end[edge_run-1];
	midpoint[i] = midpoint[edge_run-1];
	edge_run--;
	
	return res; 
      }

  /* vertex not in the list, so we add it */
  start[edge_run] = index_start;
  end[edge_run] = index_end; 
  midpoint[edge_run] = n_vertices; 
  
  /* create new vertex */ 
  vertices[3*n_vertices]   = (vertices[3*index_start] + vertices[3*index_end]) / 2.0;
  vertices[3*n_vertices+1] = (vertices[3*index_start+1] + vertices[3*index_end+1]) / 2.0;
  vertices[3*n_vertices+2] = (vertices[3*index_start+2] + vertices[3*index_end+2]) / 2.0;
  
  /* normalize the new vertex */ 
  float length = sqrt (vertices[3*n_vertices] * vertices[3*n_vertices] +
		       vertices[3*n_vertices+1] * vertices[3*n_vertices+1] +
		       vertices[3*n_vertices+2] * vertices[3*n_vertices+2]);
  length = 1/length;
  vertices[3*n_vertices] *= length;
  vertices[3*n_vertices+1] *= length;
  vertices[3*n_vertices+2] *= length;
  
  n_vertices++;
  edge_run++;
  return midpoint[edge_run-1];
}



void ViewSphereSynthesis::initIcosahedro()
{
  //std::cout << "initializing icosahedron" << std::endl;
  
  if(vertices!= NULL)
    free(vertices);
  
  if(faces!= NULL)
    free(faces);
  
  float t;
  float tau;
  float one;
  float vaux;
  t = (1+sqrt(5))/2;
  vaux = 1+t*t;
  tau = t/sqrt(vaux);
  one =  1/sqrt(vaux);
  float icosahedron_vertices[] = {tau, one, 0.0,
				  -tau, one, 0.0,
				  -tau, -one, 0.0,
				  tau, -one, 0.0,
				  one, 0.0 ,  tau,
				  one, 0.0 , -tau,
				  -one, 0.0 , -tau,
				  -one, 0.0 , tau,
				  0.0 , tau, one,
				  0.0 , -tau, one,
				  0.0 , -tau, -one,
				  0.0 , tau, -one};
 int icosahedron_faces[] = {4, 8, 7,
			    4, 7, 9,
			    5, 6, 11,
			    5, 10, 6,
			    0, 4, 3,
			    0, 3, 5,
			    2, 7, 1,
			    2, 1, 6,
			    8, 0, 11,
			    8, 11, 1,
			    9, 10, 3,
			    9, 2, 10,
			    8, 4, 0,
			    11, 0, 5,
			    4, 9, 3,
			    5, 3, 10,
			    7, 8, 1,
			    6, 1, 11,
			    7, 2, 9,
			    6, 10, 2};
  n_vertices = 12; 
  n_faces = 20;
  n_edges = 30;
  vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
  faces = (int*)malloc(3*n_faces*sizeof(int)); 
  std::memcpy ((void*)vertices, (void*)icosahedron_vertices, 3*n_vertices*sizeof(float)); 
  std::memcpy ((void*)faces, (void*)icosahedron_faces, 3*n_faces*sizeof(int)); 
}

