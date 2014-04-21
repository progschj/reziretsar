#include <stdint.h>

typedef struct {
    int width, height, pitch;
    float *transform;
    float *depth;
    uint32_t *image;
} rasterizer;

/*
 * rasterizer_init initializes a reasterizer with a width x height render
 * target
 */
int rasterizer_init(rasterizer *r, int width, int height);

/*
 * rasterizer_free deallocates rasterizer resources
 */
void rasterizer_free(rasterizer *r);

/*
 * rasterizer_clear sets all pixels z values to depth and pixel values.
 */
void rasterizer_clear(rasterizer *r, float depth, uint32_t value);

/*
 * rasterizer_draw_array draws an array of triangles
 * 
 * Draws triangles sourced from count vertices.
 * The j-th component of the i-th vertex is fetched from
 * vertices[stride*i+j]. If components<4 the unspecified components
 * default to (0, 0, 0, 1).
 * The values written to the pixels are value+k*increment for the k-th
 * triangle.
 * 
 * parameters:
 *      vertices - pointer to the first vertex
 *      components - specified components per vertex
 *      stride - vertex stride (e.g. distance between the x components)
 *      count - number of vertices (not triangles!)
 *      value - base value for the triangles
 *      increment - value increment. incremented for each triangle.
 *      transform - 4x4 column major matrix transform applied to the vertices
 *
 * return value:
 *      returns the amount of drawn pixels (on screen and passed 
 *      the depth test).
 */

int rasterizer_draw_array(rasterizer *r, const float *vertices, int components, int stride, int count, uint32_t value, uint32_t increment, float *transform);
