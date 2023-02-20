
        .section .text.init
        .globl _start
_start: /* 汎用レジスタの初期化 */
  li x2, 0xffc /* スタックポインタをメインメモリの最終番地に設定 */
  j main
