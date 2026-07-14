#include "hid_app.h"
#include "type.h"

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct {
//     uint8_t jyoutai;
    
//     uint8_t L_x;
//     uint8_t L_y;
//     uint8_t R_x;
//     uint8_t R_y;

//     uint8_t L2;
//     uint8_t R2;

//     //左から、Triangle,Circle,Cross,Square,Left,Down,Right,Up
//     uint8_t key;//方向キーと記号キー
//     //左から、L1,R1,L3,R3,Share,Option,PS,T-Pad_click
//     uint8_t boton;//その他ボタン

//     uint8_t checsam;
// }ds4_data;

ds4_data setDeta(uint8_t const* report, uint16_t len);

bool changeData(int* output , ds4_data rewdata);

bool set_LED(int* output,uint8_t red,uint8_t green,uint8_t bure );

#ifdef __cplusplus
}
#endif