#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "rasterizer.h"

int rasterizer_init(rasterizer *r, int width, int height) {
    r->width = width;
    r->pitch = (width+3) & ~3; // make sure actual width is a multiple of 4
    r->height = height;
    if((r->depth = malloc(sizeof(float)*r->pitch*r->height)) == NULL) {
        r->image = NULL;
        return 1;
    }
    if((r->image = malloc(sizeof(uint32_t)*r->pitch*r->height)) == NULL) {
        free(r->depth);
        r->depth = NULL;
        return 1;
    }
    return 0;
}

void rasterizer_free(rasterizer *r) {
    free(r->depth);
    free(r->image);
}

void rasterizer_clear(rasterizer *r, float depth, uint32_t value) {
    for(int i = 0;i<r->pitch*r->height;++i) {
        r->depth[i] = depth;
        r->image[i] = value;
    }
}

static inline int imin(int a, int b) {
    return a<b?a:b;
}
static inline int imax(int a, int b) {
    return a>b?a:b;
}
static float lerp(float a, float b, float s) {
    return a+(b-a)*s;
}

static int draw_triangle(rasterizer *target, const float *v1, const float *v2, const float *v3, uint32_t value) {
    const int tiley = 4;
    const int tilex = 4;
    const int shift = 4;
    const float factor = 1<<shift;

    int width = target->pitch;
    int height = target->height;
    float *depth = target->depth;
    uint32_t *image = target->image;

    int passed = 0;

    float x1 = v1[0];
    float y1 = v1[1];
    float z1 = v1[2];

    float x2 = v2[0];
    float y2 = v2[1];
    float z2 = v2[2];

    float x3 = v3[0];
    float y3 = v3[1];
    float z3 = v3[2];

    float a1 = (y2-y3);
    float b1 = (x3-x2);
    float a2 = (y3-y1);
    float b2 = (x1-x3);
    float invdet = 1.0f/(a1*b2 - a2*b1);
    a1 *= invdet;
    b1 *= invdet;
    a2 *= invdet;
    b2 *= invdet;
    float a3 = -(a1 + a2);
    float b3 = -(b1 + b2);
    
    float dzx = a1*z1 + a2*z2 + a3*z3;
    float dzy = b1*z1 + b2*z2 + b3*z3 - tilex*dzx;

    float dzx1 = 1*dzx;
    float dzx2 = 2*dzx;
    float dzx3 = 3*dzx;
    float dzx4 = 4*dzx;

    int ix1 = floorf(factor*x1);
    int iy1 = floorf(factor*y1);
    int ix2 = floorf(factor*x2);
    int iy2 = floorf(factor*y2);
    int ix3 = floorf(factor*x3);
    int iy3 = floorf(factor*y3);

    int nx1 = iy1-iy2;
    int nx2 = iy2-iy3;
    int nx3 = iy3-iy1;
    int ny1 = ix2-ix1;
    int ny2 = ix3-ix2;
    int ny3 = ix1-ix3;
    
    if((nx1==0 && ny1==0) || (nx2==0 && ny2==0) || (nx3==0 && ny3==0)) {
        return 0;
    }

    int xmin = imax(0, imin(ix1, imin(ix2, ix3))>>shift);
    int ymin = imax(0, imin(iy1, imin(iy2, iy3))>>shift);
    int xmax = imin(width-1, imax(ix1, imax(ix2, ix3))>>shift);
    int ymax = imin(height-1, imax(iy1, imax(iy2, iy3))>>shift);

    xmin &= ~(tilex-1);
    ymin &= ~(tiley-1);
    xmax = (xmax+tilex) & ~(tilex-1);
    ymax = (ymax+tiley) & ~(tiley-1);
    
    ix1 -= 1<<(shift-1);
    ix2 -= 1<<(shift-1);
    ix3 -= 1<<(shift-1);
    iy1 -= 1<<(shift-1);
    iy2 -= 1<<(shift-1);
    iy3 -= 1<<(shift-1);
    
    int c1 = ix1*nx1 + iy1*ny1 + (nx1+ny1==0?(nx1-ny1>0?1:0):(nx1+ny1>0?0:1));
    int c2 = ix2*nx2 + iy2*ny2 + (nx2+ny2==0?(nx2-ny2>0?1:0):(nx2+ny2>0?0:1));
    int c3 = ix3*nx3 + iy3*ny3 + (nx3+ny3==0?(nx3-ny3>0?1:0):(nx3+ny3>0?0:1));

    int twx = (tilex<<shift)-1;
    int twy = (tiley<<shift)-1;
    int in1 =  c1 - ((nx1<=0?twx*nx1:0) + (ny1<=0?twy*ny1:0));
    int in2 =  c2 - ((nx2<=0?twx*nx2:0) + (ny2<=0?twy*ny2:0));
    int in3 =  c3 - ((nx3<=0?twx*nx3:0) + (ny3<=0?twy*ny3:0));
    int out1 = c1 - ((nx1> 0?twx*nx1:0) + (ny1> 0?twy*ny1:0));
    int out2 = c2 - ((nx2> 0?twx*nx2:0) + (ny2> 0?twy*ny2:0));
    int out3 = c3 - ((nx3> 0?twx*nx3:0) + (ny3> 0?twy*ny3:0));

    nx1 <<= shift;
    nx2 <<= shift;
    nx3 <<= shift;
    ny1 <<= shift;
    ny2 <<= shift;
    ny3 <<= shift;

    for(int y0 = ymin;y0<ymax;y0+=tiley) {
        for(int x0 = xmin;x0<xmax;x0+=tilex) {
            int dot1 = nx1*x0 + ny1*y0;
            int dot2 = nx2*x0 + ny2*y0;
            int dot3 = nx3*x0 + ny3*y0;

            if(dot1>=out1 && dot2>=out2 && dot3>=out3) {
                float bary1 = x0*a1 + y0*b1 - (x3*a1 + y3*b1);
                float bary2 = x0*a2 + y0*b2 - (x3*a2 + y3*b2);
                float bary3 = 1.0f - bary1 - bary2;
                float z = bary1*z1 + bary2*z2 + bary3*z3;

                if(dot1>=in1 && dot2>=in2 && dot3>=in3) {
                    /* base version 
                     * sadly not all compiler unroll the inner loop
                    for(int y = y0;y<y0+tiley;++y,z+=dzy) {
                        for(int x = x0;x<x0+tilex;++x,z+=dzx) {
                            if(z<depth[y*width+x]) {
                                depth[y*width+x] = z;
                                image[y*width+x] = value;
                                ++passed;
                            }
                        }
                    }
                    */
                    for(int y = y0;y<y0+tiley;++y,z+=dzy) {
                        for(int x = x0;x<x0+tilex;x+=4,z+=dzx4) {
                            float zz1 = z;
                            if(zz1<depth[y*width+x]) {
                                depth[y*width+x] = zz1;
                                image[y*width+x] = value;
                                ++passed;
                            }
                            float zz2 = z+dzx1;
                            if(zz2<depth[y*width+x+1]) {
                                depth[y*width+x+1] = zz2;
                                image[y*width+x+1] = value;
                                ++passed;
                            }
                            float zz3 = z+dzx2;
                            if(zz3<depth[y*width+x+2]) {
                                depth[y*width+x+2] = zz3;
                                image[y*width+x+2] = value;
                                ++passed;
                            }
                            float zz4 = z+dzx3;
                            if(zz4<depth[y*width+x+3]) {
                                depth[y*width+x+3] = zz4;
                                image[y*width+x+3] = value;
                                ++passed;
                            }
                        }
                    }
                } else {
                    int comp1 = dot1 - c1;
                    int comp2 = dot2 - c2;
                    int comp3 = dot3 - c3;

                    int dc1 = ny1-tilex*nx1;
                    int dc2 = ny2-tilex*nx2;
                    int dc3 = ny3-tilex*nx3;

                    for(int y = y0;y<y0+tiley;++y,z += dzy) {
                        for(int x = x0;x<x0+tilex;++x,z += dzx) {
                            if(comp1 >= 0 && comp2 >= 0 && comp3 >= 0) {
                                if(z<depth[y*width+x]) {
                                    depth[y*width+x] = z;
                                    image[y*width+x] = value;
                                    ++passed;
                                }
                            }
                            comp1 += nx1;
                            comp2 += nx2;
                            comp3 += nx3;
                        }
                        comp1 += dc1;
                        comp2 += dc2;
                        comp3 += dc3;
                    }
                }
            }
        }
    }
    return passed;
}

static void transform_vertex1(float *dest, const float *src, float *A, int components) {
    float in[4];
    in[0] = components>0?src[0]:0;
    in[1] = components>1?src[1]:0;
    in[2] = components>2?src[2]:0;
    in[3] = components>3?src[3]:1;
    dest[0] = 0;
    dest[1] = 0;
    dest[2] = 0;
    dest[3] = 0;
    for(int j = 0;j<4;++j) {
        dest[0] += A[0+4*j]*in[j];
        dest[1] += A[1+4*j]*in[j];
        dest[2] += A[2+4*j]*in[j];
        dest[3] += A[3+4*j]*in[j];
    }
}

static void transform_vertex2(float *dest, int width, int height) {
    float x = dest[0];
    float y = dest[1];
    float z = dest[2];
    float w = dest[3];
    float invw = 1.0f/w;
    dest[0] = (0.5f+x*invw*0.5f)*width;
    dest[1] = (0.5f-y*invw*0.5f)*height;
    dest[2] = z*invw;
    dest[3] = invw;
}

static int clip_polygon(float *dest, const float *src, int n, const float *plane) {
    const float *normal = plane;
    float p = plane[4];
    int out = 0;
    for(int i = 0;i<n;++i) {
        const float *a = src+4*i;
        const float *b = src+4*((i+1)%n);        
        float aa = a[0]*normal[0] + a[1]*normal[1] + a[2]*normal[2] + a[3]*normal[3];
        float bb = b[0]*normal[0] + b[1]*normal[1] + b[2]*normal[2] + b[3]*normal[3];
        
        if(aa>=p) {
            if(bb>=p) {
                dest[4*out+0] = b[0];
                dest[4*out+1] = b[1];
                dest[4*out+2] = b[2];
                dest[4*out+3] = b[3];
                ++out;
            } else {
                float s = (p-aa)/(bb-aa);
                dest[4*out+0] = lerp(a[0], b[0], s);
                dest[4*out+1] = lerp(a[1], b[1], s);
                dest[4*out+2] = lerp(a[2], b[2], s);
                dest[4*out+3] = lerp(a[3], b[3], s);
                ++out;
            }
        } else {
            if(bb>=p) {
                float s = (p-aa)/(bb-aa);
                dest[4*out+0] = lerp(a[0], b[0], s);
                dest[4*out+1] = lerp(a[1], b[1], s);
                dest[4*out+2] = lerp(a[2], b[2], s);
                dest[4*out+3] = lerp(a[3], b[3], s);
                ++out;
                dest[4*out+0] = b[0];
                dest[4*out+1] = b[1];
                dest[4*out+2] = b[2];
                dest[4*out+3] = b[3];
                ++out;
            }
        }
    }
    return out;
}

static const float clip_planes[] = {
    1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
   -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
    0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
    0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
    0.0f,  0.0f,  1.0f,  1.0f,  0.0f,
    0.0f,  0.0f, -1.0f,  1.0f,  0.0f,
    0.0f,  0.0f,  0.0f,  1.0f,  0.0f,
};

int rasterizer_draw_array(rasterizer *r, const float *vertices, int components, int stride, int count, uint32_t value, uint32_t increment, float *transform) {
    float scratch[48];
    float *transformed = scratch;
    float *clipped = scratch+24;
    int passed = 0;
    for(int i = 0;i<count;i+=3) {
        for(int j = 0;j<3;++j) {
            transform_vertex1(transformed+4*j, vertices+stride*(i+j), transform, components);
        }
        int polyverts = 3;
        for(int k = 0;k<7;++k){
            polyverts = clip_polygon(clipped, transformed, polyverts, clip_planes+5*k);
            float *tmp = transformed; transformed = clipped; clipped = tmp;
        }
        for(int j = 0;j<polyverts;++j) {
            transform_vertex2(transformed + 4*j, r->width, r->height);
        }
        for(int j = 2;j<polyverts;++j) {
            passed += draw_triangle(r, transformed+0, transformed+4*j-4, transformed+4*j, value);
        }
        value += increment;
    }
    return passed;
}
