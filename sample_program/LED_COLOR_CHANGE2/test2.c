/* Tnag Nano 1k Soc用 LED変色プログラム
   
   タイマ割り込みを使用して、１秒毎にLED発光色を変更する
*/

#define TMR_RLD_REG_ADDR ((unsigned long*)(0x00010000)) /* タイマ リロード レジスタ */
#define TMR_CTRL_REG_ADDR ((unsigned long*)(0x00010008)) /* タイマ コントロール レジスタ */
#define LED_ADDR ((unsigned long*)(0x00010010)) /* LED 制御 レジスタ */
#define SW_ADDR ((unsigned long*)(0x00010014)) /* スイッチ 読み出し レジスタ */
unsigned char int_flag;
long    loop_count;
unsigned long *led,*sw,*tmr_ctrl,*tmr_rld;

void interrupt_hander(void) __attribute__ ((interrupt("machine")));

void interrupt_hander()
{
    int_flag = 0; /*割り込み待ちフラグの解除*/
    *tmr_ctrl = 0x7; /* タイマ割り込みフラグのクリア */
}

int main()
{
    unsigned long mie,dummy,pre_sw,last_sw;
    long updown;

    /* 初期化 */
    led = LED_ADDR;
    sw = SW_ADDR;
    tmr_ctrl = TMR_CTRL_REG_ADDR;
    tmr_rld = TMR_RLD_REG_ADDR;

    *led = 0x7; /* LED消灯 */
    *tmr_rld = 52735; /* タイマカウント値 */
    int_flag = 1;
    loop_count = 0;
    mie = 0b100000000000; /* Machine external interrupt bit */
    updown = 1;
    last_sw = pre_sw = (*sw)&1;

    asm volatile("csrrs %0,mtvec,%1" : "=r"(dummy) : "r"(interrupt_hander) ); /**/
    asm volatile("csrrsi zero,mstatus,0b1000"); /*グローバル割り込みの許可*/
    asm volatile("csrrs %0,mie,%1" : "=r"(dummy) : "r"(mie) ); /*マシン権限の外部割込み許可*/

    *tmr_ctrl = 0xf; /* タイマー開始、タイマ割り込み許可 */

    while(1){ // メインループ
        while(int_flag){} /* 割り込み待ち */

        int_flag = 1; /*割り込み待ちフラグのセット*/

        pre_sw = last_sw;
        last_sw = (*sw)&1;

        if( last_sw < pre_sw ) /*SW-Aの押下げ検出*/
            updown = -updown; /*色変更方向の切替*/

        loop_count = (loop_count+1) & 0x1ff; /* 512 * 52735 / 27Hz ≒ 1s */

        if(loop_count == 0)
        {
            *led = ((*led)&0x7) - updown; /*LEDの発光色の変更*/
        }
    }
}


