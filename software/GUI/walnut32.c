#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_WALNUT32
#define LV_ATTRIBUTE_IMG_WALNUT32
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_WALNUT32 uint8_t walnut32_map[] = {
  0xff, 0xff, 0xff, 0xfc, 	/*Color of index 0*/
  0x00, 0x00, 0x00, 0xfe, 	/*Color of index 1*/

  0x00, 0x0d, 0x2f, 0x3f, 0xbf, 0xfe, 0x00, 0x00, 
  0x00, 0x1b, 0xa7, 0x3f, 0xbf, 0xff, 0xc0, 0x00, 
  0x01, 0x3f, 0xf7, 0x33, 0x2f, 0xff, 0xf0, 0x00, 
  0x03, 0xbf, 0xf3, 0x83, 0x2f, 0xff, 0xfc, 0x00, 
  0x05, 0xff, 0xf3, 0xc6, 0x2f, 0xff, 0xff, 0x00, 
  0x0f, 0xff, 0xfb, 0xf6, 0x3f, 0xff, 0xff, 0x80, 
  0x0f, 0xc7, 0xfb, 0xfe, 0x7f, 0xff, 0xff, 0xe0, 
  0x8f, 0xc7, 0xfc, 0x3e, 0x5f, 0xff, 0xff, 0xf0, 
  0x86, 0x07, 0xfc, 0x04, 0x5f, 0xff, 0xff, 0xf8, 
  0x06, 0x07, 0xfc, 0x0f, 0xff, 0xff, 0xff, 0xf8, 
  0x03, 0x0f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xf0, 
  0x01, 0x83, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 
  0x01, 0x83, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 
  0x01, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 
  0x03, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 
  0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 
  0x14, 0x7f, 0xff, 0xff, 0xf8, 0x07, 0xff, 0xe0, 
  0x3f, 0xdf, 0xff, 0xff, 0x99, 0x81, 0xff, 0xf0, 
  0x10, 0xbf, 0xff, 0xf3, 0x98, 0xfc, 0x7f, 0xf8, 
  0x37, 0x0f, 0xfb, 0xff, 0xbd, 0xfe, 0xff, 0xf8, 
  0x37, 0xe3, 0xfb, 0xff, 0xfc, 0xfc, 0xff, 0xfc, 
  0x17, 0xfb, 0x7f, 0xff, 0xfe, 0xfc, 0xff, 0xfc, 
  0x17, 0xfb, 0x7f, 0xff, 0xf6, 0xfd, 0xff, 0xc0, 
  0x33, 0xf7, 0xff, 0xff, 0xf2, 0x79, 0xff, 0x80, 
  0x1b, 0xe7, 0xff, 0xff, 0xf3, 0x7b, 0xff, 0x80, 
  0x1b, 0xe7, 0xf9, 0xff, 0xe1, 0x03, 0xff, 0xc0, 
  0x09, 0xe7, 0xff, 0xff, 0xcf, 0xc7, 0xff, 0xe0, 
  0x0d, 0xcf, 0xfb, 0xff, 0xfe, 0x7f, 0xff, 0xe0, 
  0x0c, 0x0f, 0xf3, 0xff, 0xfe, 0x67, 0xff, 0xf0, 
  0x07, 0x9f, 0xf6, 0xfe, 0x64, 0xf3, 0xff, 0xf8, 
  0x03, 0xff, 0xe6, 0x78, 0x64, 0xe3, 0xff, 0xfc, 
  0x1c, 0xef, 0xf6, 0x40, 0x67, 0xe3, 0xff, 0xfe, 
  0x39, 0xff, 0xf6, 0x40, 0x65, 0xc3, 0xff, 0xff, 
  0x79, 0xdf, 0xe2, 0x40, 0x65, 0xc3, 0xff, 0xff, 
  0xf9, 0xdf, 0xe3, 0x80, 0x26, 0x43, 0xff, 0xff, 
  0xf8, 0xf7, 0xe0, 0x00, 0x1b, 0xe7, 0xff, 0xff, 
  0xf8, 0x1f, 0xe0, 0x01, 0x02, 0x3f, 0xff, 0xff, 
  0xf9, 0xfd, 0xe0, 0x1f, 0x86, 0x9f, 0xff, 0xf8, 
  0xfd, 0xfc, 0xe0, 0x08, 0x8d, 0xf7, 0xff, 0xc0, 
  0xfc, 0x3e, 0xe0, 0x08, 0x8d, 0xc3, 0xff, 0xe0, 
  0xfc, 0x91, 0x70, 0x07, 0x8b, 0xe3, 0xff, 0xf0, 
  0xf9, 0xf5, 0x1c, 0x00, 0x09, 0x8f, 0xff, 0xf0, 
  0xfb, 0xfc, 0x9f, 0xc0, 0x09, 0x3f, 0xff, 0xf8, 
  0xfb, 0xfc, 0xdf, 0xfd, 0xf8, 0x7f, 0xff, 0xfc, 
  0xf3, 0xfd, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 
  0xf7, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 
  0xf7, 0x83, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 
  0xf0, 0x27, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0x03, 0xff, 0xff, 0xf9, 0xff, 0xff, 0xff, 0xff, 
  0x00, 0x7f, 0xff, 0xf9, 0xff, 0xff, 0xff, 0xff, 
  0x00, 0x1f, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 
  0x00, 0x1f, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xff, 
  0x00, 0x3f, 0xff, 0xf0, 0x7f, 0xff, 0xff, 0xff, 
  0x00, 0x3f, 0xff, 0xe6, 0x1f, 0xff, 0xef, 0xff, 
  0x00, 0x7f, 0xff, 0xf6, 0x7f, 0xff, 0xcf, 0xff, 
};

const lv_img_dsc_t walnut32 = {
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 64,
  .header.h = 64,
  .data_size = 520,
  .data = walnut32_map,
};