#include "viewport.h"

#include "CMU462.h"

namespace CMU462 {

void ViewportImp::set_viewbox( float x, float y, float span ) {

  // Task 5 (part 2): 
  // Set svg to normalized device coordinate transformation. Your input
  // arguments are defined as SVG canvas coordinates.
  this->x = x;
  this->y = y;
  this->span = span; 

  // translate to origin
  Matrix3x3 B;

  B(0,0) = 1.; B(0,1) = 0.; B(0,2) = -1 * (x-span);
  B(1,0) = 0.; B(1,1) = 1.; B(1,2) = -1 * (y-span);
  B(2,0) = 0.; B(2,1) = 0.; B(2,2) = 1.;

  // scale down
  Matrix3x3 A;

  A(0,0) = 0.5 / span; A(0,1) = 0.; A(0,2) = 0.;
  A(1,0) = 0.; A(1,1) = 0.5 / span; A(1,2) = 0.;
  A(2,0) = 0.; A(2,1) = 0.; A(2,2) = 1.;

  set_canvas_to_norm(A * B);
}

void ViewportImp::update_viewbox( float dx, float dy, float scale ) { 
  
  this->x -= dx;
  this->y -= dy;
  this->span *= scale;
  set_viewbox( x, y, span );
}

} // namespace CMU462
