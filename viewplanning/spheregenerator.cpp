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

#include "spheregenerator.h"


SphereGenerator::SphereGenerator()
{

}

SphereGenerator::~SphereGenerator()
{

}


int SphereGenerator::getPoints(std::vector< std::vector< double > > points, int n_subdivisions, int polihedron)
{
  int i;
  int l=0;

  if (polihedron == 6)
    init_tetrahedron ();
  if (polihedron == 8)
    init_octahedron ();
  if (polihedron == 20)
    init_icosahedron ();

  for (i=0; i<n_subdivisions; i++) 
    subdivide (); 
  
  copy_sphere(points);
  l = points.size();
  //output_sphere (argv[3]); 

  if (vertices) free (vertices); 
  if (faces) free (faces); 

  return l; 
}


void SphereGenerator::copy_sphere(std::vector< std::vector<double> > points)
{
  int i, j; 
  points.clear();
  
//   FILE *ptr = fopen (filename, "w"); 
//   if (ptr == NULL) 
//     printf ("Unable to open \"%s\" :(\n", filename); 

  /*
  fprintf (ptr, "OFF\n%d %d %d\n", n_vertices, n_faces, n_edges); 
  for (i=0; i<n_vertices; i++) 
    fprintf (ptr, "%f %f %f\n", vertices[3*i], vertices[3*i+1], vertices[3*i+2]); 
  for (i=0; i<n_faces; i++)
    fprintf (ptr, "%d %d %d\n", faces[3*i], faces[3*i+1], faces[3*i+2]);  
  */

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

