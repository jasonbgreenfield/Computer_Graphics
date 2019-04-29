#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <math.h>
#include <cstdlib>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using std::max;


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128
#define FULLSCREEN_MODE true

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */
struct Intersection{
  vec4 position;
  float distance;
  int triangleIndex;
};

bool Update(vec4& cameraPos,vec4& lightPos);
void updateWorld( vector<Triangle>& t , vec4& lightPos);
void updateR(int angle, int axis);
float deg2rad(float a);
float rad2deg( float a );
// float dx();
// float dz();
float distance( vec4 camera );
void Draw( screen* screen, vec4 cameraPos, vec4 lightPos, vector<Triangle>& t );
bool ClosestIntersection( vec4 start,
                          vec4 dir,
                          const vector<Triangle>& triangles,
                          Intersection& closestIntersection );
vec3 DirectLight(Intersection interS, vec4 lightPos, vector<Triangle> triangles);


const float focalLength = 226.274;
mat3 R;
float theta;
int change;
int changeR;

int main( int argc, char* argv[] ){

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  //initializing variables that will be updated/used throughout
  vec4 cameraPos( 0, 0, -5, 1 );
  vec4 lightPos( 0, -0.5, -0.7, 1 );

  vector<Triangle> triangles;
  LoadTestModel( triangles );

  theta = 0;
  change = 1;
  changeR = 0;

  while( Update(cameraPos, lightPos) ) //could also use: NoQuitMessageSDL() and call Update() in the loop
    {
      if (change == 1){
        Draw(screen, cameraPos, lightPos, triangles);
        SDL_Renderframe(screen);
        change = 0;
      }
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

void Draw(screen* screen, vec4 cameraPos, vec4 lightPos, vector<Triangle>& t){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  if(changeR == 1){
    updateWorld( t, lightPos );
    changeR = 0;
  }

  Intersection closest;

  for(int i = 0; i < SCREEN_WIDTH; i ++){
    for(int j = 0; j < SCREEN_HEIGHT; j++){

      vec4 dir(i - (SCREEN_WIDTH/2), j - (SCREEN_HEIGHT/2), focalLength, 1);

      if( ClosestIntersection(cameraPos, dir, t, closest) ){
        //final illumination: direct light, shadows, and indirectLight approxomation
        vec3 indirectLight = 0.5f*vec3( 1, 1, 1 );
        PutPixelSDL(screen, i, j, t[closest.triangleIndex].color * (DirectLight(closest, lightPos, t) + indirectLight) ); // this is how we're diong it with B&W illumination

      }
      else{
        vec3 tempColor(0.25, 0.25, 0.25);
        PutPixelSDL(screen, i, j, tempColor );
      }
    }
  } //end of nested for loops where we draw each pixel
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

void updateWorld(vector<Triangle>& t, vec4& lightPos){

  vec4 newv0;
  vec4 newv1;
  vec4 newv2;
  vec4 newnormal;

  vec4 newlight( lightPos.x * R[0][0] + lightPos.y * R[0][1] + lightPos.z * R[0][2],
                 lightPos.x * R[1][0] + lightPos.y * R[1][1] + lightPos.z * R[1][2],
                 lightPos.x * R[2][0] + lightPos.y * R[2][1] + lightPos.z * R[2][2],
                 1 );

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

float deg2rad(float a){
  return a*(M_PI/180);
}

float rad2deg(float a){
  return a*(180/M_PI);
}

// float dx(){
//   return ( 2*abs(initcamera.z)*(sin(deg2rad(theta)/2))*(cos(deg2rad(theta)/2)) );
// }
//
// float dz(){
//   return ( 2*abs(initcamera.z)*(sin(deg2rad(theta)/2))*(sin(deg2rad(theta)/2)) );
// }

float distance(vec4 camera){
  float d = sqrt((camera.x*camera.x)+(camera.z*camera.z));
  return d;
}

bool ClosestIntersection( vec4 start, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection ){

  bool toReturn = false;
  float closestSoFar = std::numeric_limits<float>::max();

  for(int i = 0; i < triangles.size(); ++i){

    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;

    vec3 e1 = vec3(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x, start.y-v0.y, start.z-v0.z);

    vec3 threeDir(dir.x, dir.y, dir.z);
    mat3 A(-threeDir, e1, e2);
    vec3 x = glm::inverse(A)*b;

    vec4 pos( (v0.x + (x.y*e1.x) + (x.z*e2.x)),
              (v0.y + (x.y*e1.y) + (x.z*e2.y)),
              (v0.z + (x.y*e1.z) + (x.z*e2.z)),
              1);

    if( (x.x >= 0) && (x.y >= 0) && (x.z >= 0) && ((x.y + x.z)<=1) ){
        toReturn = true;
        float tempDist = x.x;

        if(tempDist < closestSoFar){
          closestIntersection.position = pos;
          closestIntersection.distance = tempDist;
          closestIntersection.triangleIndex = i;
          closestSoFar = tempDist;
        }

    }//end of if statement checking u, v, t

  }//end of loop for all triangles

  return toReturn;
}

vec3 DirectLight(Intersection interS, vec4 lightPos, vector<Triangle> triangles){
  vec3 toReturn(0, 0, 0);
  vec3 lightColor = 14.f * vec3( 1, 1, 1 );

  vec4 rCarrot(lightPos.x - interS.position.x,
         lightPos.y - interS.position.y,
         lightPos.z - interS.position.z,
         // lightPos.w - interS.position.w); // this is what I was doing when it didn't really work...maybe this should always just be 1 still?
         1);
  vec4 nCarrot = triangles[interS.triangleIndex].normal;
  float radius = sqrt( pow(lightPos.x - interS.position.x ,2) + pow(lightPos.y - interS.position.y, 2) + pow(lightPos.z - interS.position.z, 2) + pow(lightPos.w - interS.position.w, 2) ); //diong it with 1 for w now instead of subtracting them
  // float radius = sqrt( pow(lightPos.x - interS.position.x ,2) + pow(lightPos.y - interS.position.y, 2) + pow(lightPos.z - interS.position.z, 2) + pow(lightPos.w - interS.position.w, 2) );
  float dotProduct = (rCarrot.x*nCarrot.x + rCarrot.y*nCarrot.y + rCarrot.z*nCarrot.z + rCarrot.w*nCarrot.w );

  if( dotProduct > 0){
    float multiplyBy = ( dotProduct/ (4*pow(radius, 2)*M_PI) );
    toReturn.x = lightColor.x*multiplyBy;
    toReturn.y = lightColor.y*multiplyBy;
    toReturn.z = lightColor.z*multiplyBy;
  }

  Intersection closestToLight;
  vec4 shadowDirection(rCarrot.x*-1, rCarrot.y*-1, rCarrot.z*-1, 1);
  if( ClosestIntersection(lightPos, shadowDirection, triangles, closestToLight) ){
    float tempDistance = sqrt( pow(lightPos.x - closestToLight.position.x, 2) + pow(lightPos.y - closestToLight.position.y, 2) + pow(lightPos.z - closestToLight.position.z, 2) + 1  ); //this is distance from light to closest intersection
    if(tempDistance < radius){
      return vec3(0, 0, 0);
    }
  }

  return toReturn;
}

/*Place updates of parameters here*/
bool Update(vec4& cameraPos, vec4& lightPos){
  //static int t = SDL_GetTicks();
  /* Compute frame time */
  // int t2 = SDL_GetTicks();
  // float dt = float(t2-t);
  // t = t2;
  //std::cout << "Render time: " << dt << " ms." << std::endl;

  theta = 0;
  SDL_Event e;

  while(SDL_PollEvent(&e))
  {
    if(e.type == SDL_QUIT){
      return false;
    }
    else{
      if(e.type == SDL_KEYDOWN){
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
  }
  return true;
  }
