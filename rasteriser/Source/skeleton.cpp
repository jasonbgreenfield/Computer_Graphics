#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;
using glm::vec2;

SDL_Event event;

#define SCREEN_WIDTH 555
#define SCREEN_HEIGHT 555
#define FULLSCREEN_MODE false
#define FOCAL_LENGTH 326.274

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS
   */
struct Pixel
{
  int x;
  int y;
  float zinv;
  vec4 pos3d;
};

struct Vertex
{
  vec4 position;
};

struct Trapezoid
{
  vec4 left_original;
  vec4 left_intersect;
  vec4 right_original;
  vec4 right_intersect;
};

bool Update(vec4& cameraPos);
void Draw(screen* screen, vec4 cameraPos, vector<Triangle>& triangles);
void VertexShader( const Vertex& v, Pixel& p, vec4 cameraPos );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels );
int max3(int a, int b, int c);
int min3(int a, int b, int c);
float max2(float a, float b);
void DrawPolygonRows( screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 color );
void DrawPolygon( screen* screen,  vector<Vertex>& vertices, vec3 color, vec4 cameraPos );
void PixelShader( screen* screen, const Pixel& p, vec3 color );
vec3 LightVector( Pixel v );
vec3 DirectLight( Pixel v );
void updateWorld( vector<Triangle>& t );
float deg2rad( float a );
float rad2deg( float a );
void updateR( int angle, int axis );
void clipAndCull( vector<Triangle>& triangles);
float vec4DP(vec4 a, vec4 b);
void convertBackToNormal(Triangle& t);
// void addVertices(vector<Vertex>& vertices, int triangle_index, vec4 a, vec4 b);
// bool sameVector(Vertex a, vec4 b);
// void make_set(vector<vec4>& v);
void checkBoundary( vector<Triangle>& triangles, vec4 p, vec4 n);
void checkTrap(Trapezoid trap, Triangle current, vector<Triangle>& list, bool isClipped, bool isCulled, bool isSame);

float theta;
int change;
int changeR;

float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
vec3 lightPos(0,-0.5,-0.7);
int brightness = 15;
vec3 lightPower = 1.1f*vec3( brightness, brightness, brightness);
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );
vec4 currentNormal;
vec3 currentReflectance(.5, .5, .5);

mat3 R;

int main( int argc, char* argv[] )
{
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  vec4 cameraPos( 0, 0, -3.001,1 );
  theta = 0;
  change = 1;
  changeR = 0;

  vector<Triangle> triangles;
  LoadTestModel( triangles );

  while ( Update(cameraPos))
    {
      if (change == 1){
        Draw(screen, cameraPos, triangles);
        SDL_Renderframe(screen);
        change = 0;
      }

    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

float deg2rad(float a){
  return a*(M_PI/180);
}

float rad2deg(float a){
  return a*(180/M_PI);
}

/*Place your drawing here*/
void Draw(screen* screen, vec4 cameraPos, vector<Triangle>& triangles)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  if(changeR == 1){
    updateWorld( triangles );
    changeR = 0;
  }

  for( int y=0; y<SCREEN_HEIGHT; ++y ){
    for( int x=0; x<SCREEN_WIDTH; ++x ){
       depthBuffer[y][x] = 0;
    }
  }

  clipAndCull(triangles);

  for( uint32_t i=0; i<triangles.size(); ++i )
     {

       currentNormal = triangles[i].normal;

       //check if the triangle is forward-facing
       bool alwaysTrue = true;
       if( currentNormal.z < 0.1 ){
       // if( alwaysTrue ){

         vector<Vertex> vertices(3);

         vertices[0].position = triangles[i].v0;
         vertices[1].position = triangles[i].v1;
         vertices[2].position = triangles[i].v2;

         // cout << "Triangle (v0): " << i << "(" << triangles[i].v0.x << ", " << triangles[i].v0.y << ", "<< triangles[i].v0.z << ", "<<triangles[i].v0.w << ")" << endl;
         // cout << "Triangle (v1): " << i << "(" << triangles[i].v1.x << ", " << triangles[i].v1.y << ", "<< triangles[i].v1.z << ", "<<triangles[i].v1.w << ")" << endl;
         // cout << "Triangle (v2): " << i << "(" << triangles[i].v2.x << ", " << triangles[i].v2.y << ", "<< triangles[i].v2.z << ", "<<triangles[i].v2.w << ")" << endl;

         vec3 currentColor = triangles[i].color;
         DrawPolygon(screen, vertices, currentColor, cameraPos);

       }


    }

}

// bool sameVector(Vertex a, vec4 b){
//
//   if( (a.position.x==b.x) && (a.position.y==b.y) && (a.position.z==b.z) && (a.position.w==b.w) ){
//     return true;
//   }
//   return false;
// }
//
//
// void addVertices(vector<Vertex>& vertices, int triangle_index, vec4 a, vec4 b)
// {
//   bool a_is_in = false;
//   bool b_is_in = false;
//   for(int i = 0; i < vertices.size(); i++){
//
//     if( sameVector(vertices[i], a) ){
//       a_is_in = true;
//     }
//     if( sameVector(vertices[i], b) ){
//       b_is_in = true;
//     }
//
//   }//end of for loop
//
//   if(a_is_in && b_is_in){
//     //add two new triangles
//
//   }
//   else if(a_is_in){
//     //just change the triangles
//
//   }
//   else if(b_is_in){
//     //just change the triangles
//
//   }
//
// }
//
// void make_set(vector<vec4>& v){
//   //this will turn the list of vertices into a set
//
// }

void convertBackToNormal(Triangle& t)
{
  t.v0.x = t.v0.x/t.v0.w;
  t.v0.y = t.v0.y/t.v0.w;
  t.v0.z = t.v0.z/t.v0.w;
  t.v0.w = t.v0.w/t.v0.w;

  t.v1.x = t.v1.x/t.v1.w;
  t.v1.y = t.v1.y/t.v1.w;
  t.v1.z = t.v1.z/t.v1.w;
  t.v1.w = t.v1.w/t.v1.w;

  t.v2.x = t.v2.x/t.v2.w;
  t.v2.y = t.v2.y/t.v2.w;
  t.v2.z = t.v2.z/t.v2.w;
  t.v2.w = t.v2.w/t.v2.w;

  glm::vec3 e1 = glm::vec3(t.v1.x-t.v0.x,t.v1.y-t.v0.y,t.v1.z-t.v0.z);
  glm::vec3 e2 = glm::vec3(t.v2.x-t.v0.x,t.v2.y-t.v0.y,t.v2.z-t.v0.z);
  glm::vec3 normal3 = glm::normalize( glm::cross( e2, e1 ) );
  t.normal.x = normal3.x;
  t.normal.y = normal3.y;
  t.normal.z = normal3.z;
  t.normal.w = 1.0;
}

void checkTrap(Trapezoid trap, Triangle current, vector<Triangle>& list, bool isClipped, bool isCulled, bool isSame)
{
  //do I need to change these coordinates back to normal?

  //checks if the original triangle was clipped/culled
  if(isClipped){
    //checks if there are 3 unique vertices of 4
    //if 4, we need two new numTriangles
    if( (trap.left_original.x==trap.right_original.x) && (trap.left_original.y==trap.right_original.y) && (trap.left_original.z==trap.right_original.z) && (trap.left_original.w==trap.right_original.w) ){
      Triangle new_triangle(trap.left_original, trap.left_intersect, trap.right_intersect, current.color );
      convertBackToNormal(new_triangle);
      list.push_back(new_triangle);
    }
    else{
      Triangle new_a(trap.left_original, trap.left_intersect, trap.right_intersect, current.color);
      Triangle new_b(trap.right_original, trap.right_intersect, trap.left_original, current.color);

      convertBackToNormal(new_a);
      convertBackToNormal(new_b);

      list.push_back(new_a);
      list.push_back(new_b);
    }//end of case with 2 new triangles

  }//end of clipped
  else if(isSame){
    list.push_back(current);
  }
 cout << "checked a trap!!! " << current.color.x<< endl;
}

void checkBoundary( vector<Triangle>& triangles, vec4 p, vec4 n)
{
  //this loops through all triangles

  vector<Triangle> toReturn;

  for(int i = 0; i < triangles.size(); i++){

    Trapezoid trap;
    bool no_left = true;
    bool isClipped = false;
    bool isCulled = false;
    bool isSame = true;
    int numEdgesIn = 0;
    int numEdgesOut = 0;
    //this loop checks one triangle
    for(int j = 0; j < 3; j++){

      vec4 q1;
      vec4 q2;
      if(j==0){
        q1.x = triangles[i].v0.x;
        q1.y = triangles[i].v0.y;
        q1.z = triangles[i].v0.z;
        q1.w = triangles[i].v0.z/FOCAL_LENGTH;

        q2.x = triangles[i].v1.x;
        q2.y = triangles[i].v1.y;
        q2.z = triangles[i].v1.z;
        q2.w = triangles[i].v1.z/FOCAL_LENGTH;
      }
      else if(j==1){
        q1.x = triangles[i].v1.x;
        q1.y = triangles[i].v1.y;
        q1.z = triangles[i].v1.z;
        q1.w = triangles[i].v1.z/FOCAL_LENGTH;

        q2.x = triangles[i].v2.x;
        q2.y = triangles[i].v2.y;
        q2.z = triangles[i].v2.z;
        q2.w = triangles[i].v2.z/FOCAL_LENGTH;
      }
      else{
        q1.x = triangles[i].v2.x;
        q1.y = triangles[i].v2.y;
        q1.z = triangles[i].v2.z;
        q1.w = triangles[i].v2.z/FOCAL_LENGTH;

        q2.x = triangles[i].v0.x;
        q2.y = triangles[i].v0.y;
        q2.z = triangles[i].v0.z;
        q2.w = triangles[i].v0.z/FOCAL_LENGTH;
      }

      vec4 q1_minus_p = (q1 - p);
      vec4 q2_minus_p = (q2 - p);
      float d1 = vec4DP(q1_minus_p, n);
      float d2 = vec4DP(q2_minus_p, n);

      //check four cases
      if( (d1>0) && (d2>0)){
        //both vertices are in, we continue
        numEdgesIn += 1;
        continue;
        cout << "both in q1: " << q1.x<<", "<<q1.y<< endl;
      }
      else if( (d1<0) && (d2<0)){
        //both are outside, we do nothing
        numEdgesOut += 1;
        continue;
        cout << "both out q1: " << q1.x<<", "<<q1.y<< endl;
      }
      else if( (d1>0) && (d2<0) ){
        cout << "one in on eout q1: " << q1.x<<", "<<q1.y<< endl;
        //q1 is in, q2 is out, calculate one intersection
        isClipped = true;
        vec4 intersect;
        float t = d1/(d1-d2);
        intersect = q1 + t*(q1-q2);
        cout<< "I1: (" << intersect.x << ", "<<intersect.y << ", "<<intersect.z << ", "<<intersect.w << ")"<<endl;

        //we need to change what was q2 in vertices -> intersect
        if(no_left){
          trap.left_original = q1;
          trap.left_intersect = intersect;
          no_left = false;
        }
        else{
          trap.right_original = q1;
          trap.right_intersect = intersect;
        }

      }
      else if( (d1<0) && (d2>0) ){
        cout << "q1: " << q1.x<<", "<<q1.y<< endl;
        //q2 is in, q1 is out, caluclate one intersection
        isClipped = true;
        vec4 intersect;
        float t = d2/(d2-d1);
        intersect = q2 + t*(q2-q1);
        cout<< "I2: Trianl"<<"(" << intersect.x << ", "<<intersect.y << ", "<<intersect.z << ", "<<intersect.w << ")"<<endl;


        //we need to change what was q1 in vertices -> intersect
        if(no_left){
          trap.left_original = q2;
          trap.left_intersect = intersect;
          no_left = false;
        }
        else{
          trap.right_original = q2;
          trap.right_intersect = intersect;
        }

      }

    }//end of j loop

    //whole triangle is in boundary
    if(numEdgesIn == 3){
      isSame = true;
      isClipped = false;
      isCulled = false;
    }
    //whole triangle is outside boundary
    if(numEdgesOut == 3){
      isCulled = true;
      isSame = false;
      isClipped = false;
    }

    checkTrap(trap, triangles[i], toReturn, isClipped, isCulled, isSame);

  }//end of i loop

  triangles = toReturn;

}

void clipAndCull( vector<Triangle>& triangles )
{

  //check left plane
  vec4 p_left(-1, 0, 0, 0);
  vec4 n_left(1, 0, 0, 0);
  checkBoundary(triangles, p_left, n_left);


}

void DrawPolygon( screen* screen, vector<Vertex>& vertices, vec3 color, vec4 cameraPos )
{

        bool alwaysTrue = true;
       // if( clipAndCull(vertices) ){
        if( alwaysTrue ){

         int numTriangles = vertices.size()/3;
         for(int i = 0; i < numTriangles; i++ ){

           vector<Pixel> vertexPixels(3);
           for(int j = 0; j < 3; j++){
             VertexShader(vertices[i+j], vertexPixels[j], cameraPos);
           }

           vector<Pixel> leftPixels;
           vector<Pixel> rightPixels;
           ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
           DrawPolygonRows( screen, leftPixels, rightPixels, color );

         }//end of loop for all the new triangles

       }//end of clipAndCull() if statement

}

void DrawPolygonRows( screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 color )
{
  for(int i = 0; i < leftPixels.size(); i++){
    //cout << "        start drawing row:" << i << endl;

    int start = leftPixels[i].x;
    int end = rightPixels[i].x;
    //int yHeight = leftPixels[i].y;
    int width = (end-start+1);

    Pixel left = leftPixels[i];
    Pixel right = rightPixels[i];
    vector<Pixel> row(width);
    Interpolate(left,right,row);
    //cout << "        interpolated row: " << i << endl;

    for(int j = 0; j < width; j++){
      //cout << "          check pixel depth for pixel " << j << " of row " << i
           //<< " , xyz: "<< row[j].x << "," << row[j].y << "," << row[j].zinv << endl;
      if(row[j].zinv > depthBuffer[row[j].x][row[j].y]){
        //cout << "          draw pixel & update buffer start" << endl;


        PixelShader( screen, row[j], color);
        depthBuffer[row[j].x][row[j].y] = row[j].zinv;
        //cout << "          finished drawing pixel" << endl;

      }
    }
  }
}

void Interpolate( Pixel a, Pixel b, vector<Pixel>& result )
{
      // cout << "pixel A, illum, Interpo (" << a.illumination.x << ", " << a.illumination.y << ", " << a.illumination.z << ")" << endl;
      // cout << "pixel B, illum, Interpo (" << b.illumination.x << ", " << b.illumination.y << ", " << b.illumination.z << ")" << endl;

       vec4 apz = a.pos3d * a.zinv;
       vec4 bpz = b.pos3d * b.zinv;

       int N = result.size();
       vec3 step;
       step.x = ((b.x-a.x)/float(max(N-1,1)));
       step.y = ((b.y-a.y)/float(max(N-1,1)));
       step.z = ((b.zinv-a.zinv)/float(max(N-1,1)));
       vec3 current(a.x,a.y,a.zinv);

       vec4 istep;
       istep = (bpz - apz)/float(max(N-1,1));
       vec4 icurrent(apz);

       for( int i=0; i<N; ++i )
       {
           result[i].x = (round(current.x));
           result[i].y = (round(current.y));
           result[i].zinv = current.z;
           result[i].pos3d = icurrent/current.z;

           // cout << "pixel current, illum, Interpo (" << result[i].illumination.x << ", " << result[i].illumination.y << ", " << result[i].illumination.z << ")" << endl;


           current.x += step.x;
           current.y += step.y;
           current.z += step.z;
           icurrent += istep;
       }
}

void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels )
{
  // 1. Find max and min y-value of the polygon
  //    and compute the number of rows it occupies.
  //this is currently hardcoded here to only handle triangles (3 vertices)
  int maxY = max3(vertexPixels[0].y, vertexPixels[1].y, vertexPixels[2].y);
  int minY = min3(vertexPixels[0].y, vertexPixels[1].y, vertexPixels[2].y);
  int numRows = maxY - minY + 1;

  // 2. Resize leftPixels and rightPixels
  //    so that they have an element for each row.
  leftPixels.resize(numRows);
  rightPixels.resize(numRows);


  // 3. Initialize the x-coordinates in leftPixels
  //    to some really large value and the x-coordinates
  //    in rightPixels to some really small value.
  for( int i = 0; i < numRows; ++i )
  {
      leftPixels[i].x  += numeric_limits<int>::max();
      rightPixels[i].x -= numeric_limits<int>::max();

      leftPixels[i].y = minY + i;
      rightPixels[i].y = minY + i;
  }

  // 4. Loop through all edges of the polygon and use
  //    linear interpolation to find the x-coordinate for
  //    each row it occupies. Update the corresponding
  // values in rightPixels and leftPixels.
  for(int i = 0; i < 3; i++){

    ivec2 delta;
    delta.x = glm::abs( vertexPixels[i].x - vertexPixels[ (i+1)%3 ].x );
    delta.y = glm::abs( vertexPixels[i].y - vertexPixels[ (i+1)%3 ].y );
    int pixels = glm::max( delta.x, delta.y ) + 1;
    vector<Pixel> line( pixels );
    Interpolate( vertexPixels[i], vertexPixels[ (i+1)%3 ], line );

    //cout << "      for v" << i << endl;

    for(int j = 0; j < line.size(); j++){


      if(leftPixels[line[j].y - minY].x > line[j].x){
        leftPixels[line[j].y - minY].x = line[j].x;
        leftPixels[line[j].y - minY].zinv = line[j].zinv;
        leftPixels[line[j].y - minY].pos3d = line[j].pos3d;
      }
      //
      if(rightPixels[line[j].y - minY].x < line[j].x){
        rightPixels[line[j].y - minY].x = line[j].x;
        rightPixels[line[j].y - minY].zinv = line[j].zinv;
        rightPixels[line[j].y - minY].pos3d = line[j].pos3d;
      }
      //cout << "         left z = " << leftPixels[line[j].y - minY].zinv << " at " << j << endl;
      //cout << "        right z = " << rightPixels[line[j].y - minY].zinv << " at " << j << endl;

    }//end of j for loop for each line
  }//end of loop doing step 4

}

vec3 DirectLight( Pixel v )
{
  vec3 d;
  vec3 r (LightVector(v));
  vec3 n (currentNormal.x,currentNormal.y,currentNormal.z);
  float RtimesN = (r.x*n.x) + (r.y*n.y) + (r.z*n.z);
  float radius = sqrt( pow(r.x,2) + pow(r.y,2) + pow(r.z,2)  );
  float scaler = max2(RtimesN, 0)/(4*M_PI* (radius) );
  d.x = scaler*lightPower.x;
  d.y = scaler*lightPower.y;
  d.z = scaler*lightPower.z;
  return d;
}
//
// vec3 Reflection( vec3 d, Vertex& v )
// {
//   vec3 R = crossProduct( v.reflectance, vec3(d + indirectLightPowerPerArea) );
// }

vec3 LightVector( Pixel v )
{
  vec3 delta;
  delta.x = lightPos.x - v.pos3d.x;
  delta.y = lightPos.y - v.pos3d.y;
  delta.z = lightPos.z - v.pos3d.z;
  return delta;
}

void VertexShader( const Vertex& v, Pixel& p, vec4 cameraPos )
{

  // translate first: P' = P - C
  float x = v.position.x - cameraPos.x;
  float y = v.position.y - cameraPos.y;
  float z = v.position.z - cameraPos.z;
  // then rotate:
  if(z!=0){
    p.zinv = 1/z;
  }
  else {
    p.zinv = 0;
  }

  // then move from 3D to 2D:
  p.x = (FOCAL_LENGTH*x/z) + (SCREEN_WIDTH/2);
  p.y = (FOCAL_LENGTH*y/z) + (SCREEN_HEIGHT/2);

  // p.illumination = DirectLight(v); //used to be done here, now donw in pixelshader
  // cout << "illumination in VertexShader(" << p.illumination.x << ", " << p.illumination.y << ", " << p.illumination.z << ")" << endl;

  p.pos3d = v.position;

}

void PixelShader( screen* screen, const Pixel& p, vec3 color )
{
  int x = p.x;
  int y = p.y;
  if( p.zinv > depthBuffer[x][y])
  {
    depthBuffer[x][y] = p.zinv;
    vec3 illumination;
    illumination = DirectLight(p);
    PutPixelSDL(screen, x, y, illumination*color);
    // cout << "color in PixelShader (" << color.x << ", " << color.y << ", " << color.z << ")" << endl;
    // if( (color.x*p.illumination.x > 1) || (color.y*p.illumination.y > 1) || (color.z*p.illumination.z > 1) ){
    //   cout << "ill*color in PixelShader(" << color.x*p.illumination.x << ", " << color.y*p.illumination.y << ", " << color.z*p.illumination.z << ")" << endl;
    // }
  }
}

void updateR( int angle, int axis )
{
  //rotate x
  if(axis==0){
    R[0][0] = 1;
    R[0][1] = 0;
    R[0][2] = 0;
    R[1][0] = 0;
    R[1][1] = cos(deg2rad(angle));
    R[1][2] = -sin(deg2rad(angle));
    R[2][0] = 0;
    R[2][1] = sin(deg2rad(angle));
    R[2][2] = cos(deg2rad(angle));
  }
  //rotate y
  if(axis==1){
    R[0][0] = cos(deg2rad(angle));
    R[0][1] = 0;
    R[0][2] = sin(deg2rad(angle));
    R[1][0] = 0;
    R[1][1] = 1;
    R[1][2] = 0;
    R[2][0] = -sin(deg2rad(angle));
    R[2][1] = 0;
    R[2][2] = cos(deg2rad(angle));
  }
  //rotate z
  if(axis==2){
    R[0][0] = cos(deg2rad(angle));
    R[0][1] = -sin(deg2rad(angle));
    R[0][2] = 0;
    R[1][0] = sin(deg2rad(angle));
    R[1][1] = cos(deg2rad(angle));
    R[1][2] = 0;
    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = 1;
  }

}

void updateWorld(vector<Triangle>& t)
{

  vec4 newv0;
  vec4 newv1;
  vec4 newv2;
  vec4 newnormal;

  vec3 newlight( lightPos.x * R[0][0] + lightPos.y * R[0][1] + lightPos.z * R[0][2],
                 lightPos.x * R[1][0] + lightPos.y * R[1][1] + lightPos.z * R[1][2],
                 lightPos.x * R[2][0] + lightPos.y * R[2][1] + lightPos.z * R[2][2] );

  lightPos = newlight;

  for(int i = 0; i < t.size(); i++){
    newv0.x = t[i].v0.x * R[0][0] + t[i].v0.y * R[0][1] + t[i].v0.z * R[0][2];
    newv0.y = t[i].v0.x * R[1][0] + t[i].v0.y * R[1][1] + t[i].v0.z * R[1][2];
    newv0.z = t[i].v0.x * R[2][0] + t[i].v0.y * R[2][1] + t[i].v0.z * R[2][2];
    newv0.w = 1;

    newv1.x = t[i].v1.x * R[0][0] + t[i].v1.y * R[0][1] + t[i].v1.z * R[0][2];
    newv1.y = t[i].v1.x * R[1][0] + t[i].v1.y * R[1][1] + t[i].v1.z * R[1][2];
    newv1.z = t[i].v1.x * R[2][0] + t[i].v1.y * R[2][1] + t[i].v1.z * R[2][2];
    newv1.w = 1;

    newv2.x = t[i].v2.x * R[0][0] + t[i].v2.y * R[0][1] + t[i].v2.z * R[0][2];
    newv2.y = t[i].v2.x * R[1][0] + t[i].v2.y * R[1][1] + t[i].v2.z * R[1][2];
    newv2.z = t[i].v2.x * R[2][0] + t[i].v2.y * R[2][1] + t[i].v2.z * R[2][2];
    newv2.w = 1;

    newnormal.x = t[i].normal.x * R[0][0] + t[i].normal.y * R[0][1] + t[i].normal.z * R[0][2];
    newnormal.y = t[i].normal.x * R[1][0] + t[i].normal.y * R[1][1] + t[i].normal.z * R[1][2];
    newnormal.z = t[i].normal.x * R[2][0] + t[i].normal.y * R[2][1] + t[i].normal.z * R[2][2];
    newnormal.w = 1;

    t[i].v0 = newv0;
    t[i].v1 = newv1;
    t[i].v2 = newv2;
    t[i].normal = newnormal;
  }
}

/*Place updates of parameters here*/
bool Update(vec4& cameraPos)
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  //float dt = float(t2-t);
  t = t2;

  theta = 0;

  SDL_Event e;
  while(SDL_PollEvent(&e))
    {
      if (e.type == SDL_QUIT)
	{
	  return false;
	}
      else
	if (e.type == SDL_KEYDOWN)
	  {
      change = 1;
	    int key_code = e.key.keysym.sym;
	    switch(key_code)
	      {
	      case SDLK_w:
        		/* Move camera forward */
            cameraPos.y -= .15;
        		break;
	      case SDLK_s:
        		/* Move camera backwards */
            cameraPos.y += .15;
        		break;
	      case SDLK_a:
        		/* Move camera left */
            cameraPos.x += .15;
        		break;
	      case SDLK_d:
        		/* Move camera right */
            cameraPos.x -= .15;
        		break;
        case SDLK_q:
            /* Move camera forwards */
            cameraPos.z += .15;
            break;
        case SDLK_e:
    /* Move camera backwards */
    cameraPos.z -= .15;
    break;

    case SDLK_i:
      /* rotate world forward x */
      theta = -10;
      updateR(theta,0);
      changeR = 1;
      break;
    case SDLK_k:
      /* rotate world backward x */
      theta = 10;
      updateR(theta,0);
      changeR = 1;
      break;
    case SDLK_j:
      /* rotate world left y*/
      theta = 10;
      updateR(theta,1);
      changeR = 1;
      break;
    case SDLK_l:
      /* rotate world right y*/
      theta = -10;
      updateR(theta,1);
      changeR = 1;
      break;
    case SDLK_u:
      /* rotate world left z*/
      theta = -10;
      updateR(theta,2);
      changeR = 1;
      break;
    case SDLK_o:
      /* rotate world right z */
      theta = 10;
      updateR(theta,2);
      changeR = 1;
      break;



        case SDLK_UP:
    /* Move light up */
    lightPos.y -= .15;
    break;
        case SDLK_DOWN:
    /* Move light down */
    lightPos.y += .15;
    break;
        case SDLK_RIGHT:
    /* Move light right */
    lightPos.x += .15;
    break;
        case SDLK_LEFT:
    /* Move light left */
    lightPos.x -= .15;
    break;
        case SDLK_z:
    /* Move light forwad */
    lightPos.z += .15;
    break;
        case SDLK_x:
    /* Move light backwards */
    lightPos.z -= .15;
    break;



	      case SDLK_ESCAPE:
		/* Move camera quit */
		return false;
	      }
	  }
    }
  return true;
}

int max3(int a, int b, int c)
{
  int toReturn = a;
  if(toReturn < b){
    toReturn = b;
  }
  if(toReturn < c){
    toReturn = c;
  }
  return toReturn;
}

int min3(int a, int b, int c)
{
  int toReturn = a;
  if(toReturn > b){
    toReturn = b;
  }
  if(toReturn > c){
    toReturn = c;
  }
  return toReturn;
}

float max2(float a, float b)
{
  if(a > b){
    return a;
  }
  else{
    return b;
  }
}

float vec4DP(vec4 a, vec4 b)
{
  return (a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w);
}
