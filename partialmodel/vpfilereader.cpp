/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

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


#include "vpfilereader.h"

vpFileReader::vpFileReader()
{

}

vpFileReader::~vpFileReader()
{

}


bool vpFileReader::readDoubleCoordFromPCD(std::string file_name, std::vector< std::vector< double > >& data, std::string& header)
{
  //cout << " Reading double  ---" <<std::endl;
  data.clear();
  std::vector<double> lineal;
  if(!readLineCoordinatesfromPCD(file_name, lineal, header))
    return false;
  
  std::vector<double> point(3);
  std::vector<double>::iterator it;
  it = lineal.begin();
  
  //cout << "Readed values: " << lineal.size() <<std::endl;
  std::cout << "Readed " << lineal.size()/3 << "coordinates from PCD" <<  std::endl;	
  //getchar();
  
  long int i=0;
  while(it != lineal.end()){
    point[0] = *it;
    it ++;
    if(it == lineal.end())
      break;
    
    point[1] = *it;
    it ++;
    if(it == lineal.end())
      break;
    
    point[2] = *it;
    it ++;
    if(it == lineal.end())
      break;
    
    //cout << i << " " << point[0] << " "  <<  point[1] << " "  << point[2] <<std::endl;
    i++;
    data.push_back(point);
  }
  
  //getchar();
  
  //cout << "--- Done rcf pcd ---" <<std::endl;
  return true;
}



bool vpFileReader::readLineCoordinatesfromPCD(std::string file_name, std::vector< double >& data, std::string& header)
{
  double value;
  
  std::ifstream file(file_name.c_str());
  char line[100];
  int n = 100;
  
  if(file.is_open()){
    // Read the header
    
    //# .PCD v0.7 - Point Cloud Data file format
    file.getline(line,100);
    header.append(line);
    //VERSION 0.7
    file.getline(line,100);
    header.append(line);
    //FIELDS x y z
    file.getline(line,n);
    header.append(line);
    //SIZE 4 4 4
    file.getline(line,n);
    header.append(line);
    //TYPE F F F
    file.getline(line,n);
    header.append(line);
    //COUNT 1 1 1
    file.getline(line,n);
    header.append(line);
    //WIDTH 212879
    file.getline(line,n);
    header.append(line);
    //HEIGHT 1
    file.getline(line,n);
    header.append(line);
    //VIEWPOINT 0 0 0 1 0 0 0
    file.getline(line,n);
    header.append(line);
    //POINTS 212879
    file.getline(line,n);
    header.append(line);
    //DATA ascii
    file.getline(line,n);
    header.append(line);
    
    std::cout << "File readed info:" << std::endl << header.c_str() << std::endl;

    while(file.good()){
	//cout << value <<std::endl;
	file >> value;
	data.push_back(value);
    }
    
    file.close();
    
    std::cout << "closed file" << std::endl;
    return true;
  } else {
    std::cout << "Unable to open file" << std::endl;
    return false;
  }
}


bool vpFileReader::saveDoubleCoordinates(std::string file_name, std::vector< std::vector< double > >& data)
{
  std::ofstream file;
  std::vector< std::vector< double > >::iterator it_points;
  
  //cout << "Saving data..." <<std::endl;
  file.open(file_name.c_str());
  
  if(file.is_open()){
    for(it_points = data.begin(); it_points!= data.end(); it_points++){
      file << (*it_points)[0] << "\t" << (*it_points)[1] << "\t" << (*it_points)[2] << "\n";
    }
    file.close();
  }
  else {
    std::cout << "ERROR: unable to save file "  << file_name << std::endl;
    return false;
  }
  
  std::cout << file_name << "Saved" << std::endl;
  return true;
}


bool vpFileReader::saveDoubleCoordinatesAsOBJ(std::string file_name, std::vector< std::vector< double > >& data)
{
  std::ofstream file;
  std::vector< std::vector< double > >::iterator it_points;
  
  //cout << "Saving data..." <<std::endl;
  file.open(file_name.c_str());
  
  if(file.is_open()){
    file << "# Wavefront file of vertices \n";
    file << "o Object.001 \n"; 
    for(it_points = data.begin(); it_points!= data.end(); it_points++){
      file << "v " << (*it_points)[0] << " " << (*it_points)[1] << " " << (*it_points)[2] << "\n";
    }
    file.close();
  }
  else {
    std::cout << "Error: unable to save file " << file_name << std::endl;
    return false;
  }
  
  std::cout << file_name << "Saved" << std::endl;
  return true;
}



bool vpFileReader::saveDoubleCoordinatesAsPCD(std::string file_name, std::vector< std::vector< double > >& data)
{
   // TODO:
}


bool vpFileReader::readDoubleCoordinates(std::string file_name, std::vector< std::vector< double > >& data)
{
  data.clear();
  std::vector<double> lineal;
  std::cout << "Reading double coordinates from " << file_name.c_str() << std::endl;
  if(!readDouble(file_name, lineal)){
    std::cout << "Error" << std::endl;
    return false;
  }
    
  std::vector<double> point(3);
  std::vector<double>::iterator it;
  it = lineal.begin();
  
  while(it != lineal.end()){
    point[0] = *it;
    it ++;
    point[1] = *it;
    it ++;
    point[2] = *it;
    it ++;
    data.push_back(point);
  }
  
  std::cout << data.size() << " readed points." << std::endl;
  return true;
}



bool vpFileReader::readDouble(std::string file_name, std::vector< double >& data)
{
  double value;
  
  std::ifstream file(file_name.c_str());
  
  if(file.is_open()){
    
//     while(file.good()){
//       file >> value;
//       data.push_back(value);
//     }
    while(file>>value)
      data.push_back(value);
    
   // std::cout << data.size() << "points" <<std::endl;
    
    file.close();
    return true;
  } else {
    std::cout << "Unable to open file" << std::endl;
    return false;
  }
}


bool vpFileReader::readInt(std::string file_name, std::vector< int >& data)
{
  int value;
  
  std::ifstream file(file_name.c_str());
  
  if(file.is_open()){
    
    while(file >> value)
	data.push_back(value);
    
    file.close();
    return true;
  } else {
    std::cout << "Unable to open file " << file_name.c_str() << std::endl;
    return false;
  }
}



bool vpFileReader::readDoubleCoordinatesFromWRL(std::string file_name, std::vector< std::vector< double > >& data)
{
  std::vector< double> point(3);
  //point.reserve(3);
  
  
  std::cout << "Reading WRL file: " << file_name.c_str() << std::endl;
  
  
  std::vector< std::vector<double> > vertices;
  //double ***vertices; /* output, pointer to FMatrix, not allocated [1..no_vertices][1..3] */
  int ***triangles;   /* output, pointer to IMatrix, not allocated [1..no_triangles][1..3] */
  //double ***colours;  /* output, pointer to DMatrix, not allocated [1..no_triangles][1..3] */
  int *no_vertices = new int;   /* output */
  int *no_triangles = new int;   /* output */
  
  double x,y,z;
  int v1,v2,v3;
  char code[80];
  std::FILE *tmesh;
  int i=0;

  tmesh=fopen(file_name.c_str(),"r");
  if (tmesh==NULL) return false;

  *no_vertices=0;
  *no_triangles=0;

  /* count vertices and triangles */
  while (fscanf(tmesh,"%s",code)==1)
    {
      std::string strcode(code);
      if (strcode == "point") /* count vertices */
	{
	  fscanf(tmesh,"%s",code); /* [ */
	  while(true)
	    {
	      if (fscanf(tmesh,"%lf,",&x)==0) break;
	      else
		{
		  fscanf(tmesh,"%lf %lf,",&y,&z);
		  (*no_vertices)++;
		}
	    }
	}

      if (strcode == "coordIndex") /* count triangles */
	{
	  fscanf(tmesh,"%s",code); /* [ */
	  while(true)
	    {
	      if (fscanf(tmesh,"%d,",&v1)==0) break;
	      else
		{
		  fscanf(tmesh,"%d, %d, -1,",&v2,&v3);
		  (*no_triangles)++;
		}
	    }
	}
    }

  /* printf("vertices %d, triangles %d\n",*no_vertices,*no_triangles); */

  /* allocate */
  //*vertices=mDMatrix(1,*no_vertices,1,3);
  //*triangles=mIMatrix(1,*no_triangles,1,3);
  //*colours=NULL;

  /* read data */
  rewind(tmesh);

  while (fscanf(tmesh,"%s",code)==1)
    {
      std::string strcode(code);
      if (strcode == "point") /* read vertices */
	{
	  fscanf(tmesh,"%s",code); /* [ */
	  for (i=1; i<=(*no_vertices); i++)
	    {
	      fscanf(tmesh,"%lf %lf %lf,",&x,&y,&z);
	      point[0] = x;
	      point[1] = y;
	      point[2] = z;
	      vertices.push_back(point);
// 	      (*vertices)[i][1] = x;
// 	      (*vertices)[i][2] = y;
// 	      (*vertices)[i][3] = z;

	       //printf("%d: %f %f %f\n",i,x,y,z); 
	    }
	}

      if (strcode == "coordIndex") /* read triangles */
	{
	  fscanf(tmesh,"%s",code); /* [ */
	  for (i=1; i<=(*no_triangles); i++)
	    {
	      fscanf(tmesh,"%d, %d, %d, -1,",&v1,&v2,&v3);
	      //(*triangles)[i][1]=v1+1;
	      //(*triangles)[i][2]=v2+1;
	      //(*triangles)[i][3]=v3+1;

	      /* printf("%d: %d %d %d\n",i,v1,v2,v3); */
	    }
	}
    }
  fclose(tmesh);
  
  data = vertices;
  std::cout << "Readed " << data.size() << " vertices";
  return true;
}



