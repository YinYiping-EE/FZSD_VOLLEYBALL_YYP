/**
 * @file lcd.c
 * @brief LCD (ST7789V) SPI 彩色屏驱动实现
 *
 * 移植自 CtrBoard-H7_ALL 工程 Bsp/lcd.c。
 * GPIO 初始化 (RES/BLK) 在 LCD_Init() 内完成, 不依赖 CubeMX 生成的 gpio.c。
 * 使用硬件 SPI1 (hspi1), SPI 模式 3, ~17MHz。
 */

#include "lcd.h"
#include "lcdfont.h"

/**
 * @brief  向 LCD 总线写入一个字节
 * @param  dat - 要写入的数据
 */
void LCD_Writ_Bus(uint8_t dat)
{
    LCD_CS_Clr();
#if USE_ANALOG_SPI
    for (uint8_t i = 0; i < 8; i++) {
        LCD_SCLK_Clr();
        if (dat & 0x80) {
            LCD_MOSI_Set();
        } else {
            LCD_MOSI_Clr();
        }
        LCD_SCLK_Set();
        dat <<= 1;
    }
#else
    HAL_SPI_Transmit(&hspi1, &dat, 1, 0xffff);
#endif
    LCD_CS_Set();
}

/**
 * @brief  向 LCD 写入 8 位数据
 * @param  dat - 要写入的数据
 */
void LCD_WR_DATA8(uint8_t dat)
{
    LCD_Writ_Bus(dat);
}

/**
 * @brief  向 LCD 写入 16 位数据
 * @param  dat - 要写入的数据
 */
void LCD_WR_DATA(uint16_t dat)
{
    LCD_Writ_Bus(dat >> 8);
    LCD_Writ_Bus(dat);
}

/**
 * @brief  向 LCD 写入寄存器地址
 * @param  dat - 寄存器地址
 */
void LCD_WR_REG(uint8_t dat)
{
    LCD_DC_Clr();   /* 写命令 */
    LCD_Writ_Bus(dat);
    LCD_DC_Set();   /* 写数据 */
}

/**
 * @brief  设置 LCD 显示区域的坐标范围
 * @param  x1, y1 - 起始坐标
 * @param  x2, y2 - 结束坐标
 */
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (USE_HORIZONTAL == 0) {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA(y1 + 20);
        LCD_WR_DATA(y2 + 20);
        LCD_WR_REG(0x2c);
    } else if (USE_HORIZONTAL == 1) {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA(y1 + 20);
        LCD_WR_DATA(y2 + 20);
        LCD_WR_REG(0x2c);
    } else if (USE_HORIZONTAL == 2) {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA(x1 + 20);
        LCD_WR_DATA(x2 + 20);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);
    } else {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA(x1 + 20);
        LCD_WR_DATA(x2 + 20);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);
    }
}

/**
 * @brief  在指定区域填充颜色
 * @param  xsta, ysta - 区域左上角
 * @param  xend, yend - 区域右下角
 * @param  color       - 填充颜色 (RGB565)
 * @note   全屏填充 (280x240) 约 80ms, 会阻塞当前任务
 */
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint16_t i, j;
    LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);
    for (i = ysta; i < yend; i++) {
        for (j = xsta; j < xend; j++) {
            LCD_WR_DATA(color);
        }
    }
}

/**
 * @brief  在指定位置画一个点
 * @param  x, y - 坐标
 * @param  color - 颜色 (RGB565)
 */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_WR_DATA(color);
}

/**
 * @brief  在 LCD 上画一条直线 (Bresenham 算法)
 * @param  x1, y1 - 起点坐标
 * @param  x2, y2 - 终点坐标
 * @param  color   - 线的颜色
 */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0) incx = 1;
    else if (delta_x == 0) incx = 0;
    else { incx = -1; delta_x = -delta_x; }
    if (delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0;
    else { incy = -1; delta_y = -delta_y; }
    if (delta_x > delta_y) distance = delta_x;
    else distance = delta_y;
    for (t = 0; t < distance + 1; t++) {
        LCD_DrawPoint(uRow, uCol, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

/**
 * @brief  在 LCD 上画矩形
 * @param  x1, y1 - 矩形左上角坐标
 * @param  x2, y2 - 矩形右下角坐标
 * @param  color   - 矩形的颜色
 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

/**
 * @brief  在 LCD 上画圆 (中点画圆算法)
 * @param  x0, y0 - 圆心坐标
 * @param  r      - 圆的半径
 * @param  color  - 圆的颜色
 */
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a, b;
    a = 0; b = r;
    while (a <= b) {
        LCD_DrawPoint(x0 - b, y0 - a, color);
        LCD_DrawPoint(x0 + b, y0 - a, color);
        LCD_DrawPoint(x0 - a, y0 + b, color);
        LCD_DrawPoint(x0 - a, y0 - b, color);
        LCD_DrawPoint(x0 + b, y0 + a, color);
        LCD_DrawPoint(x0 + a, y0 - b, color);
        LCD_DrawPoint(x0 + a, y0 + b, color);
        LCD_DrawPoint(x0 - b, y0 + a, color);
        a++;
        if ((a * a + b * b) > (r * r)) {
            b--;
        }
    }
}

/**
 * @brief  在 LCD 上显示汉字 (自动选择尺寸)
 * @param  x, y  - 起始坐标
 * @param  s     - 汉字字符串 (GB2312编码, 每字2字节, 以'\0'结尾)
 * @param  fc    - 前景色
 * @param  bc    - 背景色
 * @param  sizey - 字体高度: 12/16/24/32
 * @param  mode  - 显示模式: 0=非叠加, 1=叠加
 */
void LCD_ShowChinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    while (*s != 0) {
        if (sizey == 12) LCD_ShowChinese12x12(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 16) LCD_ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 24) LCD_ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 32) LCD_ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
        else return;
        s += 2;
        x += sizey;
    }
}

/**
 * @brief  显示 12x12 汉字
 */
void LCD_ShowChinese12x12(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;
    uint16_t TypefaceNum;
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;

    HZnum = sizeof(tfont12) / sizeof(typFNT_GB12);
    for (k = 0; k < HZnum; k++) {
        if ((tfont12[k].Index[0] == *(s)) && (tfont12[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) {
                        if (tfont12[k].Msk[i] & (0x01 << j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if (m % sizey == 0) {
                            m = 0;
                            break;
                        }
                    } else {
                        if (tfont12[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;
    }
}

/**
 * @brief  显示 16x16 汉字
 */
void LCD_ShowChinese16x16(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;
    uint16_t TypefaceNum;
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont16) / sizeof(typFNT_GB16);
    for (k = 0; k < HZnum; k++) {
        if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) {
                        if (tfont16[k].Msk[i] & (0x01 << j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if (m % sizey == 0) {
                            m = 0;
                            break;
                        }
                    } else {
                        if (tfont16[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;
    }
}

/**
 * @brief  显示 24x24 汉字
 */
void LCD_ShowChinese24x24(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;
    uint16_t TypefaceNum;
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont24) / sizeof(typFNT_GB24);
    for (k = 0; k < HZnum; k++) {
        if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) {
                        if (tfont24[k].Msk[i] & (0x01 << j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if (m % sizey == 0) {
                            m = 0;
                            break;
                        }
                    } else {
                        if (tfont24[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;
    }
}

/**
 * @brief  显示 32x32 汉字
 */
void LCD_ShowChinese32x32(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;
    uint16_t TypefaceNum;
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont32) / sizeof(typFNT_GB32);
    for (k = 0; k < HZnum; k++) {
        if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) {
                        if (tfont32[k].Msk[i] & (0x01 << j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if (m % sizey == 0) {
                            m = 0;
                            break;
                        }
                    } else {
                        if (tfont32[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;
    }
}

/**
 * @brief  在 LCD 上显示一个 ASCII 字符
 * @param  x, y  - 起始坐标
 * @param  num   - 要显示的字符 (ASCII)
 * @param  fc    - 前景色
 * @param  bc    - 背景色
 * @param  sizey - 字体高度: 12/16/24/32 (对应宽度 6/8/12/16)
 * @param  mode  - 显示模式: 0=非叠加, 1=叠加
 */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, TypefaceNum;
    uint16_t x0 = x;
    sizex = sizey / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';
    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);
    for (i = 0; i < TypefaceNum; i++) {
        if (sizey == 12) temp = ascii_1206[num][i];
        else if (sizey == 16) temp = ascii_1608[num][i];
        else if (sizey == 24) temp = ascii_2412[num][i];
        else if (sizey == 32) temp = ascii_3216[num][i];
        else return;
        for (t = 0; t < 8; t++) {
            if (!mode) {
                if (temp & (0x01 << t)) LCD_WR_DATA(fc);
                else LCD_WR_DATA(bc);
                m++;
                if (m % sizex == 0) {
                    m = 0;
                    break;
                }
            } else {
                if (temp & (0x01 << t)) LCD_DrawPoint(x, y, fc);
                x++;
                if ((x - x0) == sizex) {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
}

/**
 * @brief  在 LCD 上显示字符串
 * @param  x, y  - 起始坐标
 * @param  p     - 要显示的字符串 (以'\0'结尾)
 * @param  fc    - 前景色
 * @param  bc    - 背景色
 * @param  sizey - 字体高度
 * @param  mode  - 显示模式: 0=非叠加, 1=叠加
 */
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    while (*p != '\0') {
        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
        x += sizey / 2;
        p++;
    }
}

/**
 * @brief  求 m 的 n 次方
 */
uint32_t mypow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/**
 * @brief  在 LCD 上显示整数
 * @param  x, y  - 坐标
 * @param  num   - 整数
 * @param  len   - 显示位数
 * @param  fc    - 文字颜色
 * @param  bc    - 背景颜色
 * @param  sizey - 字体大小
 */
void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t sizex = sizey / 2;
    for (t = 0; t < len; t++) {
        temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
                continue;
            } else enshow = 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

/**
 * @brief  在 LCD 上显示带符号的浮点数
 * @param  x, y    - 坐标
 * @param  num     - 浮点数
 * @param  len     - 整数位数
 * @param  decimal - 小数位数
 * @param  fc      - 文字颜色
 * @param  bc      - 背景颜色
 * @param  sizey   - 字体大小
 */
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    int16_t num_int;
    uint8_t t, temp, sizex;
    sizex = sizey / 2;
    num_int = num * mypow(10, decimal);

    if (num < 0) {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);
        num_int = -num_int;
        x += sizex;
        len++;
    } else {
        LCD_ShowChar(x, y, ' ', fc, bc, sizey, 0);
        x += sizex;
        len++;
    }

    /* 刷新显示区域背景 */
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    for (t = 0; t < len; t++) {
        if (t == (len - decimal)) {
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}

/**
 * @brief  在 LCD 上显示正的浮点数 (无符号)
 * @param  x, y    - 坐标
 * @param  num     - 浮点数 (必须 >= 0)
 * @param  len     - 整数位数
 * @param  decimal - 小数位数
 * @param  fc      - 文字颜色
 * @param  bc      - 背景颜色
 * @param  sizey   - 字体大小
 */
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    int16_t num_int;
    uint8_t t, temp, sizex;
    sizex = sizey / 2;
    num_int = num * mypow(10, decimal);

    x += sizex;
    len++;

    /* 刷新显示区域背景 */
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    for (t = 0; t < len; t++) {
        if (t == (len - decimal)) {
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}

/**
 * @brief  在 LCD 上显示图片 (RGB565 格式)
 * @param  x, y   - 起始坐标
 * @param  length - 图片宽度
 * @param  width  - 图片高度
 * @param  pic    - 图片数据指针 (RGB565, 逐像素存储, 每像素2字节 MSB first)
 */
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
    uint16_t i, j;
    uint32_t k = 0;
    LCD_Address_Set(x, y, x + length - 1, y + width - 1);
    for (i = 0; i < length; i++) {
        for (j = 0; j < width; j++) {
            LCD_WR_DATA8(pic[k * 2]);
            LCD_WR_DATA8(pic[k * 2 + 1]);
            k++;
        }
    }
}

/**
 * @brief  LCD 初始化
 *
 * 初始化 RES/BLK GPIO (PI10/PB11), 然后执行 ST7789V 初始化序列。
 * @note  RES/BLK 的 GPIO 在此函数内初始化, 不依赖 CubeMX 的 gpio.c。
 *        若使用了 I2C2 (也使用 PB10/PB11), 此函数会覆盖其引脚配置。
 */
void LCD_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* ---- 初始化 LCD_RES (PB11) ---- */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = LCD_RES_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_RES_GPIO_Port, &GPIO_InitStruct);

    /* ---- 初始化 LCD_BLK (PB10) ---- */
    GPIO_InitStruct.Pin = LCD_BLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_BLK_GPIO_Port, &GPIO_InitStruct);

    /* ---- 硬件复位 ---- */
    LCD_RES_Clr();
    HAL_Delay(100);
    LCD_RES_Set();
    HAL_Delay(100);

    /* ---- 开启背光 ---- */
    LCD_BLK_Set();
    HAL_Delay(100);

    /* ============ ST7789V 初始化序列 ============ */
    LCD_WR_REG(0x11);       /* Sleep out */
    HAL_Delay(120);

    LCD_WR_REG(0x36);       /* MADCTL: 内存访问控制 */
    if (USE_HORIZONTAL == 0) LCD_WR_DATA8(0x00);
    else if (USE_HORIZONTAL == 1) LCD_WR_DATA8(0xC0);
    else if (USE_HORIZONTAL == 2) LCD_WR_DATA8(0x70);
    else LCD_WR_DATA8(0xA0);

    LCD_WR_REG(0x3A);       /* COLMOD: 像素格式 */
    LCD_WR_DATA8(0x05);     /* 16-bit RGB565 */

    LCD_WR_REG(0xB2);       /* PORCTRL */
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);

    LCD_WR_REG(0xB7);       /* GCTRL: 门控制 */
    LCD_WR_DATA8(0x35);

    LCD_WR_REG(0xBB);       /* VCOMS: VCOM 设置 */
    LCD_WR_DATA8(0x32);     /* Vcom = 1.35V */

    LCD_WR_REG(0xC2);       /* LCMCTRL */
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0xC3);       /* VDVVRHEN */
    LCD_WR_DATA8(0x15);     /* GVDD = 4.8V */

    LCD_WR_REG(0xC4);       /* VDV */
    LCD_WR_DATA8(0x20);     /* VDV = 0V */

    LCD_WR_REG(0xC6);       /* FRCTRL2: 帧率控制 */
    LCD_WR_DATA8(0x0F);     /* 60Hz */

    LCD_WR_REG(0xD0);       /* PWCTRL1 */
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1);

    LCD_WR_REG(0xE0);       /* PVGAMCTRL: 正极性 gamma */
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x17);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x34);

    LCD_WR_REG(0xE1);       /* NVGAMCTRL: 负极性 gamma */
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x17);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x34);

    LCD_WR_REG(0x21);       /* INVON: 显示反转 */

    LCD_WR_REG(0x29);       /* DISPON: 开启显示 */
}
