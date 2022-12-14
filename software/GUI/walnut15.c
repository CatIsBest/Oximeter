#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_WALNUT15
#define LV_ATTRIBUTE_IMG_WALNUT15
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_WALNUT15 uint8_t walnut15_map[] = {
  0xff, 0xff, 0xff, 0xfc, 	/*Color of index 0*/
  0x00, 0x00, 0x00, 0xfe, 	/*Color of index 1*/

  0x00, 0x00, 0x7f, 0xfd, 0xfc, 0xf4, 0xf0, 0x00, 
  0x00, 0x03, 0xff, 0xf5, 0xfc, 0xe5, 0x98, 0x00, 
  0x00, 0x0f, 0xff, 0xf4, 0xdc, 0xef, 0x7c, 0x00, 
  0x00, 0x3f, 0xff, 0xf4, 0xc0, 0xcf, 0xf5, 0xc0, 
  0x00, 0xff, 0xff, 0xfc, 0x61, 0xcf, 0xf7, 0xe0, 
  0x01, 0xff, 0xff, 0xfe, 0x6f, 0xcf, 0xff, 0xb0, 
  0x07, 0xff, 0xff, 0xfa, 0x7f, 0xde, 0xe3, 0xf0, 
  0x0f, 0xff, 0xff, 0xfa, 0x7f, 0x1f, 0xe3, 0xf0, 
  0x1f, 0xff, 0xff, 0xfa, 0x30, 0x3f, 0xe0, 0x70, 
  0x1f, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xe0, 0x60, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xe0, 0xc0, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc1, 0x80, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x80, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x80, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xc0, 
  0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xc0, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x0f, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xf8, 0x00, 
  0x1f, 0xff, 0xf8, 0x7a, 0xcf, 0xff, 0xf8, 0x00, 
  0x1f, 0xcf, 0x00, 0x3f, 0xff, 0xdf, 0xff, 0xf0, 
  0x3f, 0x4c, 0x3f, 0x7f, 0xff, 0xff, 0xfc, 0x18, 
  0x3f, 0x4c, 0xff, 0x7f, 0xff, 0xff, 0xe1, 0xc8, 
  0x02, 0x8e, 0xfe, 0x6f, 0xff, 0xff, 0xe7, 0xd8, 
  0x01, 0xde, 0x7e, 0x4f, 0xff, 0xff, 0x9f, 0xd8, 
  0x07, 0xdf, 0x7e, 0xcf, 0xff, 0xff, 0xbf, 0xd8, 
  0x07, 0xbf, 0x3c, 0x87, 0xff, 0x9f, 0x9f, 0xd0, 
  0x1f, 0xbf, 0xbc, 0xfb, 0xff, 0xff, 0xdf, 0xd0, 
  0x12, 0x3f, 0x91, 0xff, 0xff, 0xdf, 0xcf, 0xb0, 
  0x0f, 0xbf, 0xc5, 0x7f, 0xff, 0xcf, 0xcf, 0xb0, 
  0x1f, 0xff, 0xfd, 0xe6, 0x7f, 0x67, 0xef, 0xa0, 
  0x3f, 0xff, 0xb9, 0xc2, 0x1e, 0x27, 0xe6, 0x60, 
  0x7f, 0xff, 0x9d, 0xe6, 0x02, 0x27, 0xe0, 0x40, 
  0xff, 0xff, 0x0c, 0xe2, 0x02, 0x2f, 0xff, 0xc0, 
  0xff, 0xff, 0x1e, 0xc6, 0x02, 0x67, 0xe6, 0x70, 
  0xff, 0xff, 0x1c, 0xc4, 0x01, 0xc7, 0xee, 0x38, 
  0xff, 0xff, 0x0e, 0xb8, 0x00, 0x07, 0xe7, 0x3c, 
  0xff, 0xff, 0x89, 0x81, 0x80, 0x07, 0xe7, 0x3e, 
  0x1f, 0xff, 0x85, 0x81, 0x78, 0x07, 0xf7, 0x3f, 
  0x07, 0xff, 0x9f, 0x81, 0x10, 0x07, 0x10, 0x3f, 
  0x07, 0xff, 0xd0, 0xc1, 0x10, 0x07, 0x3f, 0x3f, 
  0x0f, 0xff, 0xe5, 0x40, 0xe0, 0x1e, 0x7f, 0x7f, 
  0x0f, 0xff, 0xed, 0x60, 0x00, 0x38, 0xf9, 0xff, 
  0x1f, 0xff, 0xed, 0x60, 0x03, 0xf1, 0xf0, 0x7f, 
  0x3f, 0xff, 0xc7, 0x3f, 0xff, 0xfb, 0xe2, 0x3f, 
  0x7f, 0xff, 0xc0, 0x3e, 0x3f, 0xff, 0xce, 0xbf, 
  0x7f, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xce, 0xbf, 
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0x1f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0x9f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0x9f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xdf, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x9f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 
  0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xfc, 
  0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xf8, 
  0xff, 0xff, 0xff, 0x2f, 0x9f, 0xff, 0xff, 0xf0, 
  0xff, 0xff, 0xfe, 0xef, 0xbf, 0xff, 0xff, 0xc0, 
  0xff, 0xff, 0xfe, 0xce, 0x0f, 0xff, 0xff, 0x00, 
  0xff, 0xff, 0xfd, 0xdf, 0x1f, 0xff, 0xfc, 0x00, 
  0xff, 0xff, 0xf9, 0x9f, 0x1f, 0xff, 0xfc, 0x00, 
  0xff, 0xf7, 0xe3, 0xb8, 0x47, 0xff, 0xfc, 0x00, 
  0xff, 0xf3, 0xe7, 0xbe, 0x6f, 0xff, 0xfe, 0x00, 
};

const lv_img_dsc_t walnut15 = {
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 64,
  .header.h = 64,
  .data_size = 520,
  .data = walnut15_map,
};
