#include "image_preprocessing.h"

void rgb565_to_rgb888(const uint8_t* src_rgb565, uint8_t* dst_rgb888, 
                      int width, int height) {
    const uint16_t* src = (const uint16_t*)src_rgb565;
    
    for (int i = 0; i < width * height; i++) {
        uint16_t pixel = src[i];
        
        // RGB565: RRRRRGGGGGGBBBBB
        uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits
        uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits
        uint8_t b = pixel & 0x1F;          // 5 bits
        
        // Auf 8 bit erweitern
        dst_rgb888[i * 3 + 0] = (r << 3) | (r >> 2);  // R
        dst_rgb888[i * 3 + 1] = (g << 2) | (g >> 4);  // G
        dst_rgb888[i * 3 + 2] = (b << 3) | (b >> 2);  // B
    }
}

void resize_rgb888_bilinear(const uint8_t* src, uint8_t* dst,
                            int src_w, int src_h, int dst_w, int dst_h) {
    float x_ratio = (float)(src_w - 1) / dst_w;
    float y_ratio = (float)(src_h - 1) / dst_h;
    
    for (int y = 0; y < dst_h; y++) {
        for (int x = 0; x < dst_w; x++) {
            float gx = x * x_ratio;
            float gy = y * y_ratio;
            int gxi = (int)gx;
            int gyi = (int)gy;
            float fx = gx - gxi;
            float fy = gy - gyi;
            
            for (int c = 0; c < 3; c++) {  // RGB Kanäle
                // 4 Nachbar-Pixel holen
                float c00 = src[(gyi * src_w + gxi) * 3 + c];
                float c10 = src[(gyi * src_w + (gxi + 1)) * 3 + c];
                float c01 = src[((gyi + 1) * src_w + gxi) * 3 + c];
                float c11 = src[((gyi + 1) * src_w + (gxi + 1)) * 3 + c];
                
                // Bilineare Interpolation
                float value = c00 * (1 - fx) * (1 - fy) +
                              c10 * fx * (1 - fy) +
                              c01 * (1 - fx) * fy +
                              c11 * fx * fy;
                
                dst[(y * dst_w + x) * 3 + c] = (uint8_t)value;
            }
        }
    }
}