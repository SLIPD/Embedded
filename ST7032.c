#include "ST7032.h"
#include <stdint.h>

char* StringConvert(char* message)
{
    char characters[] = {'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z',
                         'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z',
                         '0','1','2','3','4','5','6','7','8','9',':',';','<','=','>','?','@','[',']','^','_','{','|','}'};
    
    uint8_t values[] = {CHAR_a, CHAR_b, CHAR_c, CHAR_d, CHAR_e, CHAR_f, CHAR_g, CHAR_h, CHAR_i, CHAR_j, CHAR_k, CHAR_l, CHAR_m,
                        CHAR_n, CHAR_o, CHAR_p, CHAR_q, CHAR_r, CHAR_s, CHAR_t, CHAR_u, CHAR_v, CHAR_w, CHAR_x, CHAR_y, CHAR_z,
                        CHAR_A, CHAR_B, CHAR_C, CHAR_D, CHAR_E, CHAR_F, CHAR_G, CHAR_H, CHAR_I, CHAR_J, CHAR_K, CHAR_L, CHAR_M,
                        CHAR_N, CHAR_O, CHAR_P, CHAR_Q, CHAR_R, CHAR_S, CHAR_T, CHAR_U, CHAR_V, CHAR_W, CHAR_X, CHAR_Y, CHAR_Z,
                        CHAR_0, CHAR_1, CHAR_2, CHAR_3, CHAR_4, CHAR_5, CHAR_6, CHAR_7, CHAR_8, CHAR_9, CHAR_COLON, CHAR_SEMI_COLON,
                        CHAR_OPEN_CHEV, CHAR_EQUALS, CHAR_CLOSE_CHEV, CHAR_QUESTION, CHAR_AT, CHAR_OPEN_BRACES, CHAR_CLOSE_BRACES,
                        CHAR_CARET, CHAR_UNDERSCORE, CHAR_OPEN_CURLY, CHAR_V_BAR, CHAR_CLOSE_CURLY};
    

}