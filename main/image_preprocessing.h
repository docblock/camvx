#ifndef IMAGE_PREPROCESSING_H
#define IMAGE_PREPROCESSING_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Konvertiert RGB565 zu RGB888
// src_rgb565: Quellbild in RGB565 Format
// dst_rgb888: Zielbild in RGB888 Format (muss 1.5x so groß sein!)
// width, height: Bildabmessungen
void rgb565_to_rgb888(const uint8_t* src_rgb565, uint8_t* dst_rgb888, 
                      int width, int height);

// Skaliert ein RGB888 Bild mit bilinearer Interpolation
// src: Quellbild
// dst: Zielbild
// src_w, src_h: Quellabmessungen
// dst_w, dst_h: Zielabmessungen
void resize_rgb888_bilinear(const uint8_t* src, uint8_t* dst,
                            int src_w, int src_h, int dst_w, int dst_h);

#ifdef __cplusplus
}
#endif

#endif