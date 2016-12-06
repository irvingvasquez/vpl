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

#ifndef SPHEREGENERATOR_H
#define SPHEREGENERATOR_H

#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <math.h> 
#include <viewstructure.h>

 /* Generates a view sphere by tesselation */
class SphereGenerator
{
public:
SphereGenerator();
~SphereGenerator();

int n_vertices;
int n_faces;
int n_edges;
float *vertices = NULL;
int *faces = NULL; 

int edge_walk; 
int *start = NULL; 
int *end = NULL; 
int *midpoint = NULL; 

/* Generates a view sphere by tesselation 
 @param polihedron possible values: 20 icosahedron (default), 4 tetrahedro, octrahedro 8
 */
int getPoints(std::vector< std::vector<double> > points, int n_subdivisions, int polihedron = 20);

/* Sphere tesselation algorithm */
//int getPoints(std::vector< std::vector<double> > points, int level, std::string polihedron="icosahedro");
  
protected:
    
float **mFMatrix(int nrl,int nrh,int ncl,int nch)
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

void mFreeFMatrix(float **m,int nrl,int nrh,int ncl,int nch)
{
  int i;
  
  for (i=nrh; i>=nrl; i--) free((char *) (m[i]+ncl));
  free((char*) (m+nrl));
}


void init_tetrahedron (void) 
{ 
  float sqrt3 = 1 / sqrt(3.0);
  float tetrahedron_vertices[] = {sqrt3, sqrt3, sqrt3,
				  -sqrt3, -sqrt3, sqrt3,
				  -sqrt3, sqrt3, -sqrt3,
				  sqrt3, -sqrt3, -sqrt3}; 
  int tetrahedron_faces[] = {0, 2, 1, 0, 1, 3, 2, 3, 1, 3, 2, 0};

  n_vertices = 4; 
  n_faces = 4; 
  n_edges = 6; 
  vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
  faces = (int*)malloc(3*n_faces*sizeof(int)); 
  memcpy ((void*)vertices, (void*)tetrahedron_vertices, 3*n_vertices*sizeof(float)); 
  memcpy ((void*)faces, (void*)tetrahedron_faces, 3*n_faces*sizeof(int)); 
} 

void init_octahedron (void) 
{ 
  float octahedron_vertices[] = {0.0, 0.0, -1.0,
				 1.0, 0.0, 0.0,
				 0.0, -1.0, 0.0,
				 -1.0, 0.0, 0.0,
				 0.0, 1.0, 0.0,
				 0.0, 0.0, 1.0}; 
  int octahedron_faces[] = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 5, 2, 1, 5, 3, 2, 5, 4, 3, 5, 1, 4}; 

  n_vertices = 6; 
  n_faces = 8;
  n_edges = 12; 
  vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
  faces = (int*)malloc(3*n_faces*sizeof(int)); 
  memcpy ((void*)vertices, (void*)octahedron_vertices, 3*n_vertices*sizeof(float)); 
  memcpy ((void*)faces, (void*)octahedron_faces, 3*n_faces*sizeof(int)); 
} 

void init_icosahedron (void) 
{ 
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
  memcpy ((void*)vertices, (void*)icosahedron_vertices, 3*n_vertices*sizeof(float)); 
  memcpy ((void*)faces, (void*)icosahedron_faces, 3*n_faces*sizeof(int)); 
} 

int search_midpoint (int index_start, int index_end) 
{ 
  int i;
  for (i=0; i<edge_walk; i++) 
    if ((start[i] == index_start && end[i] == index_end) || 
	(start[i] == index_end && end[i] == index_start)) 
      {
	int res = midpoint[i];

	/* update the arrays */
	start[i]    = start[edge_walk-1];
	end[i]      = end[edge_walk-1];
	midpoint[i] = midpoint[edge_walk-1];
	edge_walk--;
	
	return res; 
      }

  /* vertex not in the list, so we add it */
  start[edge_walk] = index_start;
  end[edge_walk] = index_end; 
  midpoint[edge_walk] = n_vertices; 
  
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
  edge_walk++;
  return midpoint[edge_walk-1];
} 

void subdivide (void) 
{ 
  int n_vertices_new = n_vertices+2*n_edges; 
  int n_faces_new = 4*n_faces; 
  int i; 

  edge_walk = 0; 
  n_edges = 2*n_vertices + 3*n_faces; 
  start = (int*)malloc(n_edges*sizeof (int)); 
  end = (int*)malloc(n_edges*sizeof (int)); 
  midpoint = (int*)malloc(n_edges*sizeof (int)); 

  int *faces_old = (int*)malloc (3*n_faces*sizeof(int)); 
  faces_old = (int*)memcpy((void*)faces_old, (void*)faces, 3*n_faces*sizeof(int)); 
  vertices = (float*)realloc ((void*)vertices, 3*n_vertices_new*sizeof(float)); 
  faces = (int*)realloc ((void*)faces, 3*n_faces_new*sizeof(int)); 
  n_faces_new = 0; 

  for (i=0; i<n_faces; i++) 
    { 
      int a = faces_old[3*i]; 
      int b = faces_old[3*i+1]; 
      int c = faces_old[3*i+2]; 

      int ab_midpoint = search_midpoint (b, a); 
      int bc_midpoint = search_midpoint (c, b); 
      int ca_midpoint = search_midpoint (a, c); 

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
  free (faces_old); 
} 



// static void 
// output_sphere (char *filename) 
// { 
//   int i, j; 
//   FILE *ptr = fopen (filename, "w"); 
//   if (ptr == NULL) 
//     printf ("Unable to open \"%s\" :(\n", filename); 
// 
//   /*
//   fprintf (ptr, "OFF\n%d %d %d\n", n_vertices, n_faces, n_edges); 
//   for (i=0; i<n_vertices; i++) 
//     fprintf (ptr, "%f %f %f\n", vertices[3*i], vertices[3*i+1], vertices[3*i+2]); 
//   for (i=0; i<n_faces; i++)
//     fprintf (ptr, "%d %d %d\n", faces[3*i], faces[3*i+1], faces[3*i+2]);  
//   */
// 
//   /* Calcular el centroide de las caras*/
//   int i_vertice_a;
//   int i_vertice_b;
//   int i_vertice_c;
//   float vertice_a[3];
//   float vertice_b[3];
//   float vertice_c[3];
//   float centroide[3];
//   float **vistas = NULL;
//   
//   vistas = mFMatrix(0,n_faces-1,0,2);
// 
//   for (i=0; i<n_faces; i++){
//      i_vertice_a = faces[3*i];
//      for (j=0;j<3;j++)
//         vertice_a[j] = vertices[3*i_vertice_a+j];
// 
//      i_vertice_b = faces[3*i+1];
//      for (j=0;j<3;j++)
//         vertice_b[j] = vertices[3*i_vertice_b+j];
// 
//      i_vertice_c = faces[3*i+2];
//      for (j=0;j<3;j++)
//         vertice_c[j] = vertices[3*i_vertice_c+j];
// 
//      for (j=0;j<3;j++){
//         centroide[j] = vertice_a[j] + vertice_b[j] + vertice_c[j];
//         centroide[j] = centroide[j] / 3;  
//      }
// 
//      for (j=0;j<3;j++){
//         vistas[i][j]=centroide[j];
//      }
//   }
// 
//   /* */
//   float length;
//   for (i=0; i<n_faces; i++){
//     //Ajustar puntos en la superficie de la esfera
//     length = sqrt (vistas[i][0] * vistas[i][0] +
// 		       vistas[i][1] * vistas[i][1] +
// 		       vistas[i][2] * vistas[i][2]);
//     length = 1/length;
//     vistas[i][0] *= length;
//     vistas[i][1] *= length;
//     vistas[i][2] *= length;
// 
//     //escribir los puntos en el archivo
//     fprintf(ptr, "%f %f %f\n", vistas[i][0], vistas[i][1], vistas[i][2]);  
//   }
// 
//   mFreeFMatrix(vistas,0,n_faces-1,0,2);
//   fclose(ptr);
// }


void copy_sphere(std::vector< std::vector<double> > points);
    
};

#endif // SPHEREGENERATOR_H
