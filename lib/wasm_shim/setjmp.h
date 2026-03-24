// Minimal setjmp/longjmp for WASM — libtess2 uses these for error recovery.
// On WASM we implement them via __builtin_wasm_throw/catch if available,
// or fall back to a simple flag-based approach since libtess2 only uses
// setjmp once (at the top of tessTesselate) and longjmp to bail out.

#ifndef _SETJMP_H
#define _SETJMP_H

typedef int jmp_buf[6];

int setjmp(jmp_buf env);
_Noreturn void longjmp(jmp_buf env, int val);

#endif
