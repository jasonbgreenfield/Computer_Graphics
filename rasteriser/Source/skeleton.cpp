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
void ComputePolygonRows(const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels );
int max3(int a, int b, int c);
int min3(int a, int b, int c);
void DrawPolygonRows( screen* screen, const vector<ivec2>& leftPixels, const vector<ivec2>& rightPixels, vec3 color );
void DrawPolygon( screen* screen, const vector<vec4>& vertices, vec3 color, vec4 cameraPos );


int main( int argc, char* argv[] )
{

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
void Draw(screen* screen, vec4 cameraPos)
{
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

        vec3 currentColor = triangles[i].color;
        DrawPolygon(screen, vertices, currentColor, cameraPos);
    }

}

void DrawPolygon( screen* screen, const vector<vec4>& vertices, vec3 color, vec4 cameraPos )
{
       int V = vertices.size();
       vector<ivec2> vertexPixels( V );
       for( int i=0; i<V; ++i )
           VertexShader( vertices[i], vertexPixels[i], cameraPos );

       vector<ivec2> leftPixels;
       vector<ivec2> rightPixels;

       ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
       DrawPolygonRows( screen, leftPixels, rightPixels, color );
}

void DrawPolygonRows( screen* screen, const vector<ivec2>& leftPixels, const vector<ivec2>& rightPixels, vec3 color )
{
  for(int i = 0; i < leftPixels.size(); i++){
    int start = leftPixels[i].x;
    int end = rightPixels[i].x;
    int yHeight = leftPixels[i].y;

    for(int j = 0; j < (end - start + 1); j++){

      PutPixelSDL( screen, start + j, yHeight, color );
    }
  }
}

void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color )
{

  ivec2 delta = glm::abs( a - b );
  int pixels = glm::max( delta.x, delta.y ) + 1;
  vector<ivec2> line( pixels );
  Interpolate( a, b, line );

  for(int i = 0; i < line.size(); i++){
    PutPixelSDL( screen, line[i].x, line[i].y, color );
  }

}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result )
{
       int N = result.size();
       vec2 step = vec2(b-a) / float(max(N-1,1));
       vec2 current( a );
       for( int i=0; i<N; ++i )
       {
           result[i] = current;
           current += step;
       }
}

void DrawPolygonEdges( const vector<vec4>& vertices, screen* screen, vec4 cameraPos )
{
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

void ComputePolygonRows(const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels )
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

    ivec2 delta = glm::abs( vertexPixels[i] - vertexPixels[ (i+1)%3 ] );
    int pixels = glm::max( delta.x, delta.y ) + 1;
    vector<ivec2> line( pixels );
    Interpolate( vertexPixels[i], vertexPixels[ (i+1)%3 ], line );

    for(int j = 0; j < line.size(); j++){
      if(leftPixels[line[j].y - minY].x > line[j].x){
        leftPixels[line[j].y - minY].x = line[j].x;
      }
      if(rightPixels[line[j].y - minY].x < line[j].x){
        rightPixels[line[j].y - minY].x = line[j].x;
      }
    }//end of j for loop for each line

  }//end of loop doing step 4

}

void VertexShader( const vec4& v, ivec2& p, vec4 cameraPos )
{
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

/*Place updates of parameters here*/
bool Update(vec4& cameraPos)
{
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
    cameraPos.x -= .15;
		break;
	      case SDLK_d:
		/* Move camera right */
    cameraPos.x += .15;
		break;
        case SDLK_q:
    /* Move camera forwards */
    cameraPos.z += .15;
    break;
        case SDLK_e:
    /* Move camera backwards */
    cameraPos.z += .15;
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
