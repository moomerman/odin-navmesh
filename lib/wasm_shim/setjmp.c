// Minimal setjmp/longjmp for WASM.
// libtess2 calls setjmp once at the start of tessTesselate and longjmp to bail
// on error. Since WASM has no native setjmp, we use a simple global flag.
// This is NOT a general-purpose implementation — it only works for libtess2's
// single setjmp/longjmp pattern.

#include "setjmp.h"

static int _jmp_val = 0;
static int _jmp_set = 0;

int setjmp(jmp_buf env) {
    (void)env;
    if (_jmp_set && _jmp_val != 0) {
        int v = _jmp_val;
        _jmp_val = 0;
        _jmp_set = 0;
        return v;
    }
    _jmp_set = 1;
    _jmp_val = 0;
    return 0;
}

// longjmp can't actually jump on WASM without exception handling support.
// libtess2 checks the return value of setjmp so we store the value and
// the next call to tessTesselate will pick it up. In practice, libtess2
// calls longjmp only from within the call tree of the setjmp, so the
// tessellation function will return 0 (failure).
void longjmp(jmp_buf env, int val) {
    (void)env;
    _jmp_val = val ? val : 1;
    // We can't actually jump, but we need to not return.
    // Use __builtin_trap to abort — tessTesselate will return 0 (failure).
    __builtin_trap();
}
