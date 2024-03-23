
        .section .text.init
        .globl _start
_start:
  li sp, 0xffc /* スタックポインタをメインメモリの最終番地に設定 */
  j main       /* メイン関数呼び出し */
