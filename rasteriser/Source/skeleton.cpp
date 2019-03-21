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

bool Update(vec4& cameraPos);
void Draw(screen* screen, vec4 cameraPos, vector<Triangle>& triangles);
void VertexShader( const Vertex& v, Pixel& p, vec4 cameraPos );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels );
int max3(int a, int b, int c);
int min3(int a, int b, int c);
float max2(float a, float b);
void DrawPolygonRows( screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 color );
void DrawPolygon( screen* screen, const vector<Vertex>& vertices, vec3 color, vec4 cameraPos );
void PixelShader( screen* screen, const Pixel& p, vec3 color );
vec3 LightVector( Pixel v );
vec3 DirectLight( Pixel v );
void updateWorld( vector<Triangle>& t );
float deg2rad( float a );
float rad2deg( float a );
void updateR( int angle, int axis );

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

  for( uint32_t i=0; i<triangles.size(); ++i )
     {

       currentNormal = triangles[i].normal;

       vector<Vertex> vertices(3);

       vertices[0].position = triangles[i].v0;
       // vertices[0].reflectance = vec2(0.5,0.5);
       // vertices[0].normal = triangles[i].normal;

       vertices[1].position = triangles[i].v1;
       // vertices[1].reflectance = vec2(0.5,0.5);
       // vertices[1].normal = triangles[i].normal;

       vertices[2].position = triangles[i].v2;
       // vertices[2].reflectance = vec2(0.5,0.5);
       // vertices[2].normal = triangles[i].normal;

       // // produces bare skeleton structure of triangles
       // DrawPolygonEdges(vertices, screen, cameraPos);
       // for(int v=0; v<3; ++v)
       //   {
       //     ivec2 projPos;
       //     VertexShader( vertices[v], projPos, cameraPos );
       //     vec3 color(1,1,1);
       //     PutPixelSDL( screen, projPos.x, projPos.y, color );
       //  }


        // vector<ivec2> leftPixels( 31 );
        // vector<ivec2> rightPixels( 31 );
        // for(int i = 0; i < 3; i++){
        //   cout << "I: " << i << " (" << polygonVertices[i].x << ", " << polygonVertices[i].y << ")" << endl;
        // }

        // //this is a check to prove ComputePolygonRows() works properly
        // vector<ivec2> vertexPixels(3);
        // vertexPixels[0] = ivec2(10, 5);
        // vertexPixels[1] = ivec2( 5,10);
        // vertexPixels[2] = ivec2(15,15);
        // vector<ivec2> leftPixels;
        // vector<ivec2> rightPixels;
        // ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
        //  for( int row=0; row<leftPixels.size(); ++row )
        //  {
        //      cout << "Start: ("
        //           << leftPixels[row].x << ","
        //           << leftPixels[row].y << "). "
        //           << "End: ("
        //           << rightPixels[row].x << ","
        //           << rightPixels[row].y << "). " << endl;
        //  }
        //  //end of ComputePolygonRows() check

        // // old stuff i used to use before it was all put into DrawPolygon()
        // vector<ivec2> polygonVertices(3);
        // for(int v = 0; v < 3; ++v){
        //   VertexShader( vertices[v], polygonVertices[v], cameraPos);
        // }
        // vector<ivec2> leftPixels;
        // vector<ivec2> rightPixels;
        // ComputePolygonRows(polygonVertices, leftPixels, rightPixels);
        // DrawRows(screen, leftPixels, rightPixels);
        // //end of outdated stuff
        //cout << "drawing triangle: " << i << endl;
        vec3 currentColor = triangles[i].color;
        DrawPolygon(screen, vertices, currentColor, cameraPos);
        //cout << "finished drawing polygon" << endl;

    }

}

void DrawPolygon( screen* screen, const vector<Vertex>& vertices, vec3 color, vec4 cameraPos )
{
       int V = vertices.size();
       vector<Pixel> vertexPixels( V );
       for( int i=0; i<V; ++i ){
         // vertexPixels[i].illumination = color;
         VertexShader( vertices[i], vertexPixels[i], cameraPos );
       }
       //cout << "    vertex shader works" << endl;

       vector<Pixel> leftPixels;
       vector<Pixel> rightPixels;

       ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
       //cout << "    compute p rows works" << endl;

       DrawPolygonRows( screen, leftPixels, rightPixels, color );
       //cout << "    draw p rows works" << endl;

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

void updateR( int angle, int axis ){
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

void updateWorld(vector<Triangle>& t){

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
