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
/* FUNCTIONS                                                                   */

bool Update(vec4& cameraPos);
void Draw(screen* screen, vec4 cameraPos);
void VertexShader( const vec4& v, ivec2& p, vec4 cameraPos );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void DrawLineSDL( screen* surface, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec4>& vertices, screen* screen, vec4 cameraPos );

int main( int argc, char* argv[] ){

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  vec4 cameraPos( 0, 0, -3.001,1 );
  while ( Update(cameraPos))
    {
      Draw(screen, cameraPos);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, vec4 cameraPos){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vector<Triangle> triangles;
  LoadTestModel( triangles );


   for( uint32_t i=0; i<triangles.size(); ++i )
     {
       vector<vec4> vertices(3);
       vertices[0] = triangles[i].v0;
       vertices[1] = triangles[i].v1;
       vertices[2] = triangles[i].v2;

       DrawPolygonEdges(vertices, screen, cameraPos);

       for(int v=0; v<3; ++v)
         {
           ivec2 projPos;
           VertexShader( vertices[v], projPos, cameraPos );
           vec3 color(1,1,1);
           PutPixelSDL( screen, projPos.x, projPos.y, color );
        }
    }


}

void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color ){

  ivec2 delta = glm::abs( a - b );
  int pixels = glm::max( delta.x, delta.y ) + 1;
  vector<ivec2> line( pixels );
  Interpolate( a, b, line );

  for(int i = 0; i < line.size(); i++){
    PutPixelSDL( screen, line[i].x, line[i].y, color );
  }

}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result ){
       int N = result.size();
       vec2 step = vec2(b-a) / float(max(N-1,1));
       vec2 current( a );
       for( int i=0; i<N; ++i )
       {
           result[i] = current;
           current += step;
       }
}

void DrawPolygonEdges( const vector<vec4>& vertices, screen* screen, vec4 cameraPos ){
    int V = vertices.size();
    // Transform each vertex from 3D world position to 2D image position:
    vector<ivec2> projectedVertices( V );
    for( int i=0; i<V; ++i ){
        VertexShader( vertices[i], projectedVertices[i], cameraPos );
    }
    // Loop over all vertices and draw the edge from it to the next vertex:
    for( int i=0; i<V; ++i ){
      int j = (i+1)%V; // The next vertex
      vec3 color( 1, 1, 1 );
      DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
    }
}

/*Place updates of parameters here*/
bool Update(vec4& cameraPos){
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

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
	    int key_code = e.key.keysym.sym;
	    switch(key_code)
	      {
	      case SDLK_UP:
		/* Move camera forward */
    cameraPos.y += .15;
		break;
	      case SDLK_DOWN:
		/* Move camera backwards */
    cameraPos.y -= .15;
		break;
	      case SDLK_LEFT:
		/* Move camera left */
    cameraPos.x += .15;
		break;
	      case SDLK_RIGHT:
		/* Move camera right */
    cameraPos.x -= .15;
		break;
	      case SDLK_ESCAPE:
		/* Move camera quit */
		return false;
	      }
	  }
    }
  return true;
}

void VertexShader( const vec4& v, ivec2& p, vec4 cameraPos ){
  // translate first: P' = P - C
  float x = v.x - cameraPos.x;
  float y = v.y - cameraPos.y;
  float z = v.z - cameraPos.z;
  // then rotate:
  // Angus is roation man
  // then move from 3D to 2D:
  p.x = (FOCAL_LENGTH*x/z) + (SCREEN_WIDTH/2);
  p.y = (FOCAL_LENGTH*y/z) + (SCREEN_HEIGHT/2);

}
