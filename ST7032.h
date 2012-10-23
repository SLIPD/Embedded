/* 
 * File:   ST7032.h
 * Author: GCHAU
 *
 * Created on 23 October 2012, 01:55
 */

#ifndef ST7032_H
#define	ST7032_H

#ifdef	__cplusplus
extern "C" {
#endif


//      ST7032 Address
    
#define ST7032_ADDR_WRITE       0x7C
#define ST7032_ADDR_READ        0x7D
    
//      Control Byte
    
// Another control byte will follow after data byte
#define CONTROL_BYTE_INSTR      0x80
#define CONTROL_BYTE_DATA       0xC0
    
// Last control byte to be sent, only data bytes will follow
#define LAST_CONTROL_INSTR      0x00
#define LAST_CONTROL_DATA       0x40
    
// Instruction Codes
    
#define CLEAR_DISPLAY           0x01
#define RETURN_HOME             0x02
#define DISPLAY_ON              0x0C
#define DISPLAY_OFF             0x08
#define DISPLAY_CURSOR          0x0E
#define DISPLAY_CURSOR_POS      0x0D
#define DISPLAY_ALL             0x0F
    
    
// Character mappings

#define CHAR_SPACE              0x20
#define CHAR_EXC                0x21
#define CHAR_QUOTE              0x22
#define CHAR_HASH               0x23
#define CHAR_$                  0x24
#define CHAR_PER                0x25
#define CHAR_AMPER              0x26
#define CHAR_APOS               0x27
#define CHAR_OPEN_BRACKET       0x28
#define CHAR_CLOSE_BRACKET      0x29
#define CHAR_MUL                0x2A
#define CHAR_PLUS               0x2B
#define CHAR_COMMA              0x2C
#define CHAR_MINUS              0x2D
#define CHAR_FULL_STOP          0x2E
#define CHAR_FOR_SLASH          0x2F
    
#define CHAR_0                  0x30
#define CHAR_1                  0x31
#define CHAR_2                  0x32
#define CHAR_3                  0x33
#define CHAR_4                  0x34
#define CHAR_5                  0x35
#define CHAR_6                  0x36
#define CHAR_7                  0x37
#define CHAR_8                  0x38
#define CHAR_9                  0x39
#define CHAR_COLON              0x3A
#define CHAR_SEMI_COLON         0x3B
#define CHAR_OPEN_CHEV          0x3C
#define CHAR_EQUALS             0x3D
#define CHAR_CLOSE_CHEV         0x3E
#define CHAR_QUESTION           0x3F

#define CHAR_AT                 0x04
#define CHAR_A                  0x14
#define CHAR_B                  0x24
#define CHAR_C                  0x34
#define CHAR_D                  0x44
#define CHAR_E                  0x54
#define CHAR_F                  0x64
#define CHAR_G                  0x74
#define CHAR_H                  0x84
#define CHAR_I                  0x94
#define CHAR_J                  0xA4
#define CHAR_K                  0xB4
#define CHAR_L                  0xC4
#define CHAR_M                  0xD4
#define CHAR_N                  0xE4
#define CHAR_O                  0xF4
    
#define CHAR_P                  0x50
#define CHAR_Q                  0x51
#define CHAR_R                  0x52
#define CHAR_S                  0x53
#define CHAR_T                  0x54
#define CHAR_U                  0x55
#define CHAR_V                  0x56
#define CHAR_W                  0x57
#define CHAR_X                  0x58
#define CHAR_Y                  0x59
#define CHAR_Z                  0x5A
#define CHAR_OPEN_BRACES        0x5B
#define CHAR_YEN                0x5C
#define CHAR_CLOSE_BRACES       0x5D
#define CHAR_CARET              0x5E
#define CHAR_UNDERSCORE         0x5F
    
#define CHAR_GRAVE_ACCENT       0x60
#define CHAR_a                  0x61
#define CHAR_b                  0x62
#define CHAR_c                  0x63
#define CHAR_d                  0x64
#define CHAR_e                  0x65
#define CHAR_f                  0x66
#define CHAR_g                  0x67
#define CHAR_h                  0x68
#define CHAR_i                  0x69
#define CHAR_j                  0x6A
#define CHAR_k                  0x6B
#define CHAR_l                  0x6C
#define CHAR_m                  0x6D
#define CHAR_n                  0x6E
#define CHAR_o                  0x6F
        
#define CHAR_p                  0x70
#define CHAR_q                  0x71
#define CHAR_r                  0x72
#define CHAR_s                  0x73
#define CHAR_t                  0x74
#define CHAR_u                  0x75
#define CHAR_v                  0x76
#define CHAR_w                  0x77
#define CHAR_x                  0x78
#define CHAR_y                  0x79
#define CHAR_z                  0x7A
#define CHAR_OPEN_CURLY         0x7B
#define CHAR_V_BAR              0x7C
#define CHAR_CLOSE_CURLY        0x7D
#define CHAR_RIGHT              0x7E
#define CHAR_LEFT               0x7F
    
#define CHAR_MULTIPLY           0xF7
#define CHAR_DIVIDE             0xF8

#define CHAR_LESS_THAN          0xF9
#define CHAR_GREATER_THAN       0xFA
    
    
    
    char* StringConvert(char* message);
    
    

#ifdef	__cplusplus
}
#endif

#endif	/* ST7032_H */

