#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <stdlib.h>
#include <variant>
#include <vector>

#include <SDL.h>
#include <glm/glm.hpp>

#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"

#define WINDOW_WIDTH 600
#define WINDOW_HEIGHT 600

using namespace glm;
using namespace std;

using Color = tvec3<char>;

SDL_Renderer *renderer;

vec3 camPos(0, 0, 0);
vec3 camDir(0, 0, -1);
vec3 camUp(0, 1, 0);
vec3 camRight = glm::normalize(glm::cross(camDir, camUp));

float camSize = 10.0f; // The size (radius) of the camera plane

struct Ray {
  vec3 origin;
  vec3 dir;
};

/*****************************************************************************
 *                      HELPER FUNCTIONS                                     *
 *****************************************************************************/
// helper type for the visitor #4
template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template <class... Ts> overloaded(Ts...)->overloaded<Ts...>;

void clearScreen(Color color) {
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
  SDL_RenderClear(renderer);
}

void drawPixel(int x, int y, Color color) {
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
  SDL_RenderDrawPoint(renderer, x, y);
}

/*****************************************************************************
 *                      Shape Function                                       *
 *****************************************************************************/

struct Sphere {
  vec3 pos;
  float radius;
  Color color;
};

struct Triangle {
  array<vec3, 3> pos;
  Color color;
};

using Shape = variant<Sphere, Triangle>;

// RayHit = distance t along ray + color of object at that point
struct RayHit {
  float t;
  Color color;
};

optional<RayHit> collide(Ray ray, Sphere sphere) {
  float a = dot(ray.dir, ray.dir);
  vec3 tmp = ray.origin - sphere.pos;
  float b = 2 * dot(ray.dir, tmp);
  float c = dot(tmp, tmp) - sphere.radius * sphere.radius;

  float D = b * b - 4 * a * c;

  if (D > 0) {
    float t1 = (-b + sqrt(D)) / (2 * a);
    float t2 = (-b - sqrt(D)) / (2 * a);
    if (t1 < 0 && t2 < 0) {
      return nullopt;
    } else if (t1 < 0 || t2 < t1) {
      return RayHit{t2, sphere.color};
    } else if (t2 < 0 || t1 < t2) {
      return RayHit{t1, sphere.color};
    }
    return RayHit{t1, sphere.color};
  } else if (D == 0) {
    return RayHit{-b / (2 * a), sphere.color};
  } else if (D < 0) {
    return nullopt;
  }
  return nullopt;
}

optional<RayHit> collide(Ray ray, Triangle triangle) {
  // from:
  // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
  const float EPSILON = 0.0000001;
  vec3 vertex0 = triangle.pos[0];
  vec3 vertex1 = triangle.pos[1];
  vec3 vertex2 = triangle.pos[2];
  vec3 edge1, edge2, h, s, q;

  float a, f, u, v;
  edge1 = vertex1 - vertex0;
  edge2 = vertex2 - vertex0;
  h = cross(ray.dir, edge2);
  a = dot(edge1, h);
  if (a > -EPSILON && a < EPSILON)
    return nullopt;

  f = 1 / a;
  s = ray.origin - vertex0;
  u = f * (dot(s, h));
  if (u < 0.0 || u > 1.0)
    return nullopt;
  q = cross(s, edge1);
  v = f * dot(ray.dir, q);
  if (v < 0.0 || u + v > 1.0)
    return nullopt;
  // At this stage we can compute t to find out where the intersection point is
  // on the line.
  float t = f * dot(edge2, q);
  if (t > EPSILON) // ray intersection
  {
    return RayHit{t, triangle.color};
  } // This means that there is a line intersection but not a ray
  return nullopt;
}

vector<Shape> shapes;

void initShapes() {
  shapes.push_back(Sphere{vec3(0, 0, -10), 2.5f, Color(255, 0, 0)});
  shapes.push_back(Triangle{
      {vec3(-5, -5, -15), vec3(5, -5, -15), vec3(0, 0, -5)}, Color(0, 0, 255)});

  std::ifstream ss("model.ply", std::ios::binary);

  if (ss.fail())
    throw std::runtime_error("failed to open model");

  PlyFile file;
  file.parse_header(ss);

  // Tinyply treats parsed data as untyped byte buffers. See below for examples.
  std::shared_ptr<PlyData> vertices, faces;

  // The header information can be used to programmatically extract properties
  // on elements known to exist in the header prior to reading the data. For
  // brevity of this sample, properties like vertex position are hard-coded:
  try {
    vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
  } catch (const std::exception &e) {
    std::cerr << "tinyply exception: " << e.what() << std::endl;
  }

  // Providing a list size hint (the last argument) is a 2x performance
  // improvement. If you have arbitrary ply files, it is best to leave this 0.
  try {
    faces = file.request_properties_from_element("face", {"vertex_indices"}, 3);
  } catch (const std::exception &e) {
    std::cerr << "tinyply exception: " << e.what() << std::endl;
  }


  file.read(ss);

  if (vertices)
    std::cout << "\tRead " << vertices->count << " total vertices "
              << std::endl;
  if (faces)
    std::cout << "\tRead " << faces->count << " total faces (triangles) "
              << std::endl;

  std::vector<array<unsigned, 3>> triangles(faces->count);
  std::memcpy(triangles.data(), faces->buffer.get(),
              faces->buffer.size_bytes());
  std::vector<vec3> verts(vertices->count);
  std::memcpy(verts.data(), vertices->buffer.get(),
              vertices->buffer.size_bytes());

  for (const auto &trig : triangles) {
    shapes.push_back(Triangle{{verts[trig[0]], verts[trig[1]], verts[trig[2]]},
                              Color(0, 255, 0)});
  }
}

/*****************************************************************************
 *                      Ray Tracing *
 *****************************************************************************/

Color getPixel(int x, int y) {
  // get ray
  float nx = x / (float)WINDOW_WIDTH;
  float ny = y / (float)WINDOW_HEIGHT;
  nx = nx * 2.0f - 1.0f; // from -1 to 1
  ny = ny * 2.0f - 1.0f;
  nx *= camSize;
  ny *= -camSize; // flip (0 = bottom)


  vec3 origin = camPos + nx * camRight + ny * camUp;
  Ray ray{origin, camDir};

  // collide ray
  RayHit nearest{1000.0f, Color(0, 0, 0)};
  for (const auto &shape : shapes) {
    auto hit = visit(overloaded{[ray](Sphere s) { return collide(ray, s); },
                                [ray](Triangle t) { return collide(ray, t); }},
                     shape);
    if (!hit)
      continue;

    if (hit->t < nearest.t)
      nearest = *hit;
  }
  // get ray color
  return nearest.color;
}

void render() {
  clearScreen(Color(255, 255, 255));

  for (int x = 0; x < WINDOW_WIDTH; x++) {
    for (int y = 0; y < WINDOW_HEIGHT; y++) {
      auto color = getPixel(x, y);
      drawPixel(x, y, color);
    }
  }
}

int main(void) {
  cout << "Loading Shapes...\n";
  initShapes();

  cout << "Making Window...\n";

  SDL_Event event;
  SDL_Window *window;

  SDL_Init(SDL_INIT_VIDEO);
  SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window,
                              &renderer);

  cout << "Rendering...\n";
  auto start = std::chrono::high_resolution_clock::now();
  render();
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long microseconds =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

  cout << "Done, render took " << microseconds / 1000.0f << "ms\n";

  SDL_RenderPresent(renderer);
  while (1) {
    if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
      break;
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return EXIT_SUCCESS;
}
