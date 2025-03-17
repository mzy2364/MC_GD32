/**
 * @file lv_port_disp_templ.c
 *
 */

 /*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp_mcgd32.h"
#include "lv_conf.h"
#include "lcd.h"
/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_disp_drv_t * my_disp_drv = NULL;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/

    /* LittlevGL requires a buffer where it draws the objects. The buffer's has to be greater than 1 display row
     *
     * There are three buffering configurations:
     * 1. Create ONE buffer with some rows: 
     *      LittlevGL will draw the display's content here and writes it to your display
     * 
     * 2. Create TWO buffer with some rows: 
     *      LittlevGL will draw the display's content to a buffer and writes it your display.
     *      You should use DMA to write the buffer's content to the display.
     *      It will enable LittlevGL to draw the next part of the screen to the other buffer while
     *      the data is being sent form the first buffer. It makes rendering and flushing parallel.
     * 
     * 3. Create TWO screen-sized buffer: 
     *      Similar to 2) but the buffer have to be screen sized. When LittlevGL is ready it will give the
     *      whole frame to display. This way you only need to change the frame buffer's address instead of
     *      copying the pixels.
     * */

    /* Example for 1) */
//    static lv_disp_buf_t disp_buf_1;
//    static lv_color_t buf1_1[LV_HOR_RES_MAX * 10];                      /*A buffer for 10 rows*/
//    lv_disp_buf_init(&disp_buf_1, buf1_1, NULL, LV_HOR_RES_MAX * 10);   /*Initialize the display buffer*/

    /* Example for 2) */
    static lv_disp_buf_t disp_buf_2;
    static lv_color_t buf2_1[LV_HOR_RES_MAX * 10];                        /*A buffer for 10 rows*/
    static lv_color_t buf2_2[LV_HOR_RES_MAX * 10];                        /*An other buffer for 10 rows*/
    lv_disp_buf_init(&disp_buf_2, buf2_1, buf2_2, LV_HOR_RES_MAX * 10);   /*Initialize the display buffer*/

    /* Example for 3) */
//    static lv_disp_buf_t disp_buf_3;
//    static lv_color_t buf3_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];            /*A screen sized buffer*/
//    static lv_color_t buf3_2[LV_HOR_RES_MAX * LV_VER_RES_MAX];            /*An other screen sized buffer*/
//    lv_disp_buf_init(&disp_buf_3, buf3_1, buf3_2, LV_HOR_RES_MAX * LV_VER_RES_MAX);   /*Initialize the display buffer*/


    /*-----------------------------------
     * Register the display in LittlevGL
     *----------------------------------*/

    lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.buffer = &disp_buf_2;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lcd_spi_dma_transmit(void* buf, uint32_t size)
{
	/* 设置SPI为16位模式 */
	spi_disable(SPI2);
	spi_i2s_data_frame_format_config(SPI2, SPI_FRAMESIZE_16BIT);
	spi_enable(SPI2);

    /* 开启DMA传输 */
	dma_channel_disable(DMA1, DMA_CH1);
	dma_transfer_number_config(DMA1, DMA_CH1,size);
	dma_memory_address_config(DMA1, DMA_CH1,(uint32_t)buf);
	dma_channel_enable(DMA1, DMA_CH1);

}

/* Initialize your display and the required peripherals. */
static void disp_init(void)
{
    /*You code here*/
}

/* Flush the content of the internal buffer the specific area on the display
 * You can use DMA or any hardware acceleration to do this operation in the background but
 * 'lv_disp_flush_ready()' has to be called when finished. */
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/

    int16_t w = (area->x2 - area->x1 + 1);
    int16_t h = (area->y2 - area->y1 + 1);
    uint32_t size = w * h;

    my_disp_drv = disp_drv;

    /* 设置SPI为8位模式 */
	spi_disable(SPI2);
	spi_i2s_data_frame_format_config(SPI2, SPI_FRAMESIZE_8BIT);
	spi_enable(SPI2);
    LCD_Address_Set(area->x1,area->y1,area->x2,area->y2);		//设置地址

    LCD_CS_Clr();	//片选
    LCD_DC_Set();   //写数据

    lcd_spi_dma_transmit(color_p, size);	//启动DMA传输
    
//	while(1)
//	{
//		if(dma_flag_get(DMA1, DMA_CH1,DMA_FLAG_FTF)!=RESET)//等待通道4传输完成
//		{
//            dma_flag_clear(DMA1, DMA_CH1,DMA_FLAG_FTF);
//			break;
//		}
//	}
//    LCD_CS_Set();	//片选取消

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    //lv_disp_flush_ready(disp_drv);
}

void DMA1_Channel1_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA1,DMA_CH1,DMA_INT_FLAG_FTF) != RESET)
    {
        dma_interrupt_flag_clear(DMA1,DMA_CH1,DMA_INT_FLAG_FTF);
		LCD_CS_Set();	//片选取消
        if(my_disp_drv)
        {
            lv_disp_flush_ready(my_disp_drv);/* tell lvgl that flushing is done */
            my_disp_drv = NULL;
        }
    }
}

#else /* Enable this file at the top */

/* This dummy typedef exists purely to silence -Wpedantic. */
typedef int keep_pedantic_happy;
#endif
