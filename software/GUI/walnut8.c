#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_WALNUT8
#define LV_ATTRIBUTE_IMG_WALNUT8
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_WALNUT8 uint8_t walnut8_map[] = {
  0xff, 0xff, 0xff, 0xfc, 	/*Color of index 0*/
  0x00, 0x00, 0x00, 0xfe, 	/*Color of index 1*/

  0x00, 0x00, 0x1f, 0xf5, 0xbe, 0x74, 0x00, 0x00, 
  0x00, 0x01, 0xff, 0xf5, 0xfe, 0x74, 0xf8, 0x00, 
  0x00, 0x07, 0xff, 0xf4, 0xee, 0x67, 0xec, 0x00, 
  0x00, 0x1f, 0xff, 0xfc, 0xec, 0x67, 0xfc, 0xc0, 
  0x00, 0x7f, 0xff, 0xfe, 0x60, 0xef, 0xf3, 0xe0, 
  0x00, 0xff, 0xff, 0xfa, 0x63, 0xef, 0xff, 0xb0, 
  0x01, 0xff, 0xff, 0xfa, 0x7f, 0xce, 0xff, 0xf0, 
  0x07, 0xff, 0xff, 0xfa, 0x3f, 0xdf, 0xe3, 0xf0, 
  0x0f, 0xff, 0xff, 0xfe, 0x3c, 0x1f, 0xe3, 0x70, 
  0x0f, 0xff, 0xff, 0xff, 0x30, 0x3f, 0xe0, 0x60, 
  0x0f, 0xff, 0xff, 0xff, 0xfe, 0x3f, 0xf0, 0x60, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xc0, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x80, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x80, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xc0, 
  0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf, 0xc0, 
  0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xc0, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 
  0x0f, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xfc, 0x00, 
  0x1f, 0xff, 0xbf, 0xfd, 0xff, 0xff, 0xfc, 0x00, 
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xfc, 0x00, 
  0x3f, 0xff, 0xe1, 0xff, 0xff, 0xff, 0xfe, 0x00, 
  0x1f, 0xfc, 0x01, 0xff, 0xff, 0xff, 0xff, 0xc0, 
  0x01, 0xfc, 0xf9, 0xef, 0xff, 0xff, 0xe0, 0x60, 
  0x01, 0xfd, 0xfd, 0xc7, 0xff, 0xff, 0x00, 0x30, 
  0x01, 0xfd, 0xf9, 0xa7, 0xff, 0xbe, 0x7f, 0xd0, 
  0x03, 0xfc, 0xf2, 0x33, 0xff, 0xde, 0x7f, 0xd0, 
  0x07, 0xfc, 0xf6, 0x07, 0xff, 0xe6, 0x7f, 0x90, 
  0x0f, 0xfe, 0xe6, 0x01, 0xfe, 0x02, 0x7f, 0xb0, 
  0x1f, 0xfe, 0xec, 0x00, 0xff, 0x07, 0xff, 0x30, 
  0x1f, 0xfe, 0x0c, 0x00, 0x3f, 0xc7, 0x7f, 0x20, 
  0x7f, 0xff, 0x78, 0x01, 0x0f, 0x87, 0x7e, 0x60, 
  0xff, 0xff, 0xcf, 0x80, 0x00, 0x1f, 0xfe, 0xc0, 
  0xff, 0xfc, 0xc9, 0xff, 0x03, 0xff, 0xb8, 0x80, 
  0xff, 0xf9, 0xec, 0x7c, 0x03, 0xe7, 0x83, 0xf0, 
  0xff, 0xf8, 0xe4, 0x00, 0x00, 0x07, 0xff, 0xf0, 
  0xff, 0xf8, 0xfc, 0x00, 0x00, 0x03, 0xb3, 0xc0, 
  0x7f, 0xf8, 0x66, 0x00, 0xf8, 0x03, 0xb0, 0xe0, 
  0x0f, 0xf8, 0x76, 0x01, 0x08, 0x07, 0x99, 0xf0, 
  0x0f, 0xf8, 0x0f, 0x01, 0x08, 0x07, 0x1a, 0x38, 
  0x1f, 0xfc, 0x7f, 0x80, 0x10, 0x0e, 0x47, 0x9c, 
  0x1f, 0xfc, 0xe6, 0x00, 0xe0, 0x1c, 0xc1, 0xc4, 
  0x3f, 0xff, 0x86, 0x00, 0x00, 0xf9, 0xfd, 0xe6, 
  0x3f, 0xff, 0x37, 0x40, 0x0f, 0xf3, 0xfc, 0xc6, 
  0x7f, 0xff, 0x35, 0xff, 0xff, 0xff, 0xce, 0x1f, 
  0x7f, 0xff, 0x7d, 0xff, 0xff, 0xff, 0xc0, 0xff, 
  0xff, 0xff, 0x7d, 0xff, 0xff, 0xff, 0x9c, 0xff, 
  0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0xbe, 0xff, 
  0xff, 0xff, 0x5f, 0xff, 0xff, 0xff, 0x3e, 0x7f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3e, 0x7f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 
  0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x3f, 0x3f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xbf, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3e, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 
  0xff, 0xff, 0xff, 0xff, 0xbf, 0xff, 0xff, 0xf0, 
  0xff, 0xef, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xf0, 
  0x00, 0x07, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xe0, 
  0x38, 0xf7, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0x80, 
  0x80, 0x27, 0xff, 0xff, 0x3f, 0xff, 0xff, 0x00, 
  0xd0, 0x3f, 0xff, 0xf8, 0x0f, 0xff, 0xfc, 0x00, 
  0xff, 0xe7, 0xff, 0xf6, 0xcf, 0xff, 0xfc, 0x00, 
};

const lv_img_dsc_t walnut8 = {
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 64,
  .header.h = 64,
  .data_size = 520,
  .data = walnut8_map,
};