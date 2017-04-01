#include "software_renderer.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

#include "triangulation.h"

using namespace std;

namespace CMU462 {


// Implements SoftwareRenderer //

void SoftwareRendererImp::draw_svg( SVG& svg ) {

  // set top level transformation
  transformation = canvas_to_screen;

  // draw all elements
  for ( size_t i = 0; i < svg.elements.size(); ++i ) {
    draw_element(svg.elements[i]);
  }

  // draw canvas outline
  Vector2D a = transform(Vector2D(    0    ,     0    )); a.x--; a.y++;
  Vector2D b = transform(Vector2D(svg.width,     0    )); b.x++; b.y++;
  Vector2D c = transform(Vector2D(    0    ,svg.height)); c.x--; c.y--;
  Vector2D d = transform(Vector2D(svg.width,svg.height)); d.x++; d.y--;

  rasterize_line(a.x, a.y, b.x, b.y, Color::Black);
  rasterize_line(a.x, a.y, c.x, c.y, Color::Black);
  rasterize_line(d.x, d.y, b.x, b.y, Color::Black);
  rasterize_line(d.x, d.y, c.x, c.y, Color::Black);

  // resolve and send to render target
  resolve();

}

void SoftwareRendererImp::set_sample_rate( size_t sample_rate ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->sample_rate = sample_rate;

}

void SoftwareRendererImp::set_render_target( unsigned char* render_target,
                                             size_t width, size_t height ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->render_target = render_target;
  this->target_w = width;
  this->target_h = height;

}

void SoftwareRendererImp::draw_element( SVGElement* element ) {

  // Task 5 (part 1):
  // Modify this to implement the transformation stack

  switch(element->type) {
    case POINT:
      draw_point(static_cast<Point&>(*element));
      break;
    case LINE:
      draw_line(static_cast<Line&>(*element));
      break;
    case POLYLINE:
      draw_polyline(static_cast<Polyline&>(*element));
      break;
    case RECT:
      draw_rect(static_cast<Rect&>(*element));
      break;
    case POLYGON:
      draw_polygon(static_cast<Polygon&>(*element));
      break;
    case ELLIPSE:
      draw_ellipse(static_cast<Ellipse&>(*element));
      break;
    case IMAGE:
      draw_image(static_cast<Image&>(*element));
      break;
    case GROUP:
      draw_group(static_cast<Group&>(*element));
      break;
    default:
      break;
  }

}


// Primitive Drawing //

void SoftwareRendererImp::draw_point( Point& point ) {

  Vector2D p = transform(point.position);
  rasterize_point( p.x, p.y, point.style.fillColor );

}

void SoftwareRendererImp::draw_line( Line& line ) { 

  Vector2D p0 = transform(line.from);
  Vector2D p1 = transform(line.to);
  rasterize_line( p0.x, p0.y, p1.x, p1.y, line.style.strokeColor );

}

void SoftwareRendererImp::draw_polyline( Polyline& polyline ) {

  Color c = polyline.style.strokeColor;

  if( c.a != 0 ) {
    int nPoints = polyline.points.size();
    for( int i = 0; i < nPoints - 1; i++ ) {
      Vector2D p0 = transform(polyline.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polyline.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_rect( Rect& rect ) {

  Color c;
  
  // draw as two triangles
  float x = rect.position.x;
  float y = rect.position.y;
  float w = rect.dimension.x;
  float h = rect.dimension.y;

  Vector2D p0 = transform(Vector2D(   x   ,   y   ));
  Vector2D p1 = transform(Vector2D( x + w ,   y   ));
  Vector2D p2 = transform(Vector2D(   x   , y + h ));
  Vector2D p3 = transform(Vector2D( x + w , y + h ));
  
  // draw fill
  c = rect.style.fillColor;
  if (c.a != 0 ) {
    rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    rasterize_triangle( p2.x, p2.y, p1.x, p1.y, p3.x, p3.y, c );
  }

  // draw outline
  c = rect.style.strokeColor;
  if( c.a != 0 ) {
    rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    rasterize_line( p1.x, p1.y, p3.x, p3.y, c );
    rasterize_line( p3.x, p3.y, p2.x, p2.y, c );
    rasterize_line( p2.x, p2.y, p0.x, p0.y, c );
  }

}

void SoftwareRendererImp::draw_polygon( Polygon& polygon ) {

  Color c;

  // draw fill
  c = polygon.style.fillColor;
  if( c.a != 0 ) {

    // triangulate
    vector<Vector2D> triangles;
    triangulate( polygon, triangles );

    // draw as triangles
    for (size_t i = 0; i < triangles.size(); i += 3) {
      Vector2D p0 = transform(triangles[i + 0]);
      Vector2D p1 = transform(triangles[i + 1]);
      Vector2D p2 = transform(triangles[i + 2]);
      rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    }
  }

  // draw outline
  c = polygon.style.strokeColor;
  if( c.a != 0 ) {
    int nPoints = polygon.points.size();
    for( int i = 0; i < nPoints; i++ ) {
      Vector2D p0 = transform(polygon.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polygon.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_ellipse( Ellipse& ellipse ) {

  // Extra credit 

}

void SoftwareRendererImp::draw_image( Image& image ) {

  Vector2D p0 = transform(image.position);
  Vector2D p1 = transform(image.position + image.dimension);

  rasterize_image( p0.x, p0.y, p1.x, p1.y, image.tex );
}

void SoftwareRendererImp::draw_group( Group& group ) {

  for ( size_t i = 0; i < group.elements.size(); ++i ) {
    draw_element(group.elements[i]);
  }

}

// Rasterization //

// The input arguments in the rasterization functions 
// below are all defined in screen space coordinates

void SoftwareRendererImp::rasterize_point( float x, float y, Color color ) {

  // fill in the nearest pixel
  int sx = (int) floor(x);
  int sy = (int) floor(y);

  // check bounds
  if ( sx < 0 || sx >= target_w ) return;
  if ( sy < 0 || sy >= target_h ) return;

  // fill sample - NOT doing alpha blending!
  paint_int(sx, sy, color);
}

void SoftwareRendererImp::paint_int(int x, int y, Color color) {
  paint_int(x, y, color, 1);
}

/** alpha is a scalar [0,1] multiplied by color.a */
void SoftwareRendererImp::paint_int(int x, int y, Color color, float alphaMult) {
  render_target[4 * (x + y * target_w)    ] = (uint8_t) (color.r * 255);
  render_target[4 * (x + y * target_w) + 1] = (uint8_t) (color.g * 255);
  render_target[4 * (x + y * target_w) + 2] = (uint8_t) (color.b * 255);
  render_target[4 * (x + y * target_w) + 3] = (uint8_t) (color.a*alphaMult * 255);
}

void SoftwareRendererImp::rasterize_line( float x0f, float y0f,
                                          float x1f, float y1f,
                                          Color color) {

  bool reflect = false;
  // if |slope| > 1, reflect over y=x by setting a flag and inverting x,y coords
  // note: also transforms vertical line to horizontal line
  if (abs(y1f - y0f) > abs(x1f - x0f)) {
    reflect = true;
    float dumx0f = x0f;
    x0f = y0f;
    y0f = dumx0f;
    float dumx1f = x1f;
    x1f = y1f;
    y1f = dumx1f;
  };

  // normalize pofloats so x0f,y0f to x1f,y1f is down to up
  if (y1f < y0f) {
    float dumxf = x0f;
    x0f = x1f;
    x1f = dumxf;
    float dumyf = y0f;
    y0f = y1f;
    y1f = dumyf;
  }

  // truncate decimals: close enough
  int x0 = x0f;
  int x1 = x1f;
  int y0 = y0f;
  int y1 = y1f;

  int dx = x1 - x0;
  int dy = y1 - y0;

  bool sweep_left = false;
  if (dx * dy < 0 || (dy == 0 && x1 < x0)) {
    sweep_left = true;
  }

  int SWEEP_INCR = sweep_left ? -1 : 1;
  int abs_dx = abs(dx);
  int lp = (y1f - y1) * abs_dx;
  int x = x0;
  int ly = y0;

  if (reflect) {
    paint_int(ly, x, color);
  } else {
    paint_int(x, ly, color);
  }
  x += SWEEP_INCR;

  // you could improve performance by pre-computing some of the values
  // and removing the conditionals via arith tricks and macros.
  while ((!sweep_left && x <= x1) || (sweep_left && x >= x1)) {
    /*
    p/abs_dx is our deviation from drawing on y.
    p ranges from -abs_dx/2 to abs_dx/2
    when we exceed that upper bound, we incr y.
    */
    int p = lp + dy;
    if (p >= abs_dx/2) {
      ly++;
      lp = p - abs_dx;
    } else {
      /* noop */
      lp = p;
    }
    int newY = lp < 0 ? ly - 1 : ly + 1;
    if (reflect) {
      paint_int(ly, x, color, (1-abs(lp)/(float)abs_dx));
      paint_int(newY, x, color, (abs(lp)/(float)abs_dx));
    } else {
      paint_int(x, ly, color, (1-abs(lp)/(float)abs_dx));
      paint_int(x, newY, color, (abs(lp)/(float)abs_dx));
    }
    x += SWEEP_INCR;
  }
}

void SoftwareRendererImp::rasterize_triangle( float x0, float y0,
                                              float x1, float y1,
                                              float x2, float y2,
                                              Color color ) {
  // 1. create a bounding box of pixels, points of box are {min_x, max_x} X {min_y, max_y}
  float min_x = min(min(x0, x1), x2);
  float max_x = max(max(x0, x1), x2);
  float min_y = min(min(y0, y1), y2);
  float max_y = max(max(y0, y1), y2);

  // 2. iterate over the pixels of the bounding box
  for (int i = floor(min_x); i <= floor(max_x); i++) {
    bool in_triangle = false;
    for (int j = floor(min_y); j <= floor(max_y); j++) {
      int x = min_x + i;
      int y = min_y + j;
      // 3. sample this.sample_rate^2 points in the pixel. weight the alpha by the coverage.
      //  sr = 1 => .5
      //  sr = 2 => .25
      //  sr = 3 => .1666
      float incr =  1.0 / (2*this->sample_rate);
      int cover_hits = 0;
      for (int si = 1; si <= this->sample_rate; si++) {
        for (int sj = 1; sj <= this->sample_rate; sj++) {
          // TODO implement point in triangle-test
          if (sample_at(x + si*incr, y + sj*incr)) {
            cover_hits++;
          }
        }
      }
      if (cover_hits > 0) {
        float alphaMult = ((float)cover_hits) / (this-> sample_rate * this->sample_rate);

        // 4. draw
        paint_int(x, y, color, alphaMult);

        // to help detect early-exit condition
        in_triangle = true;
      } else {
        if (in_triangle) {
          // 5. exit iteration early if you go from in the triangle to out of the triangle.
          break;
        }
      }
    }
  }
}

void SoftwareRendererImp::rasterize_image( float x0, float y0,
                                           float x1, float y1,
                                           Texture& tex ) {
  // Task 6: 
  // Implement image rasterization

}

// resolve samples to render target
void SoftwareRendererImp::resolve( void ) {

  // Task 4: 
  // Implement supersampling
  // You may also need to modify other functions marked with "Task 4".
  return;

}


} // namespace CMU462
