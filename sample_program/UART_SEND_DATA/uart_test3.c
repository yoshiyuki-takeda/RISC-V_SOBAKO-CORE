/* Tnag Nano 1k Soc用 LED変色プログラム
   
   タイマ割り込みを使用して、１秒毎にLED発光色を変更する

   割込み処理を実行するために
   CPUコアのINT_VECTORパラメータを
   32'h0000_0090へ変更する必要がある。
*/

#define TMR_RLD_REG_ADDR  ((unsigned long*)(0x00010000)) /* タイマ リロード レジスタ */
#define TMR_CTRL_REG_ADDR ((unsigned long*)(0x00010008)) /* タイマ コントロール レジスタ */
#define LED_ADDR ((unsigned long*)(0x00010010)) /* LED 制御 レジスタ */
#define SW_ADDR  ((unsigned long*)(0x00010014)) /* スイッチ 読み出し レジスタ */

#define TX_DATA_ADDR   ((unsigned long*)(0x00010020))
#define TX_STATUS_ADDR ((unsigned long*)(0x00010024))
#define RX_DATA_ADDR   ((unsigned long*)(0x00010028))
#define RX_STATUS_ADDR ((unsigned long*)(0x0001002c))

#define BUF_NUM 64

 unsigned char int_flag;
 unsigned long *led,*sw,*tmr_ctrl,*tmr_rld;
 unsigned long *tx,*rx,*tx_status,*rx_status;

 unsigned char buf[BUF_NUM];
 unsigned char buf_count,buf_top,buf_end;

short put_char(char in)
{
    if(buf_count < BUF_NUM)
    {
        buf[buf_end] = in;
        buf_end = (buf_end+1) & (BUF_NUM-1);
        buf_count++;
        return 0;
    }
    else
        return -1;
}

char num2hex(char in)
{
    char a = in & 0xf;
    if(a>9)
        return 'a' - 10 + a;
    else
        return '0' + a;
}

int main()
{
    unsigned long led_loop;
    unsigned char loop_count = 0;

    tx = TX_DATA_ADDR;
    rx = RX_DATA_ADDR;
    tx_status = TX_STATUS_ADDR;
    rx_status = RX_STATUS_ADDR;

    led = LED_ADDR;

    *led = 7;
    led_loop = 0;
    buf_count = 0;
    buf_top = 0;
    buf_end = 0;
    while(1){ // メインループ
        if( *tx_status&1 )
        {
            if(buf_count > 0)
            {
                *tx = buf[buf_top];
                buf_top = (buf_top + 1) & (BUF_NUM-1);
                buf_count--;
            }
        }
        if( *rx_status&1 )
        {
            put_char(*rx+1);
        }

        if(led_loop == 600000)
        {
            led_loop = 0;
            *led = (*led&7)^4;
            put_char( num2hex(loop_count>>4) );
            put_char( num2hex(loop_count) );
            put_char( '\n' );
            loop_count++;
        }

        led_loop++;
    }
}


