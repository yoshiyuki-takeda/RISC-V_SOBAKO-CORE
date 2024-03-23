
        .section .text.init;
        .globl _start;
_start: /* 汎用レジスタの初期化 */
  li sp, 0xffc  /* スタックポインタをメインメモリの最終番地に設定 */
  call main     /* メイン関数呼び出し */
