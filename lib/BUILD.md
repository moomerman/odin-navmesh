# Building libtess2

Pre-built libraries for libtess2 (https://github.com/memononen/libtess2).

## Get the source

```sh
git clone --depth 1 https://github.com/memononen/libtess2.git /tmp/libtess2
```

## macOS (arm64)

```sh
cd /tmp/libtess2
cc -c -O2 -I Include Source/*.c
ar rcs libtess2.a *.o
cp libtess2.a <project>/nav/lib/
```

## WASM

Requires LLVM with wasm32 target support. Apple clang does NOT support wasm —
use Homebrew LLVM (`brew install llvm@18`).

```sh
LLVM=$(brew --prefix llvm@18)
ODIN_ROOT=$(odin root)

cd /tmp/libtess2
$LLVM/bin/clang -c -Os --target=wasm32 \
  --sysroot=$ODIN_ROOT/vendor/libc-shim \
  -isystem <project>/nav/lib/wasm_shim \
  -I Include \
  Source/*.c \
  <project>/nav/lib/wasm_shim/setjmp.c

$LLVM/bin/wasm-ld -r *.o -o libtess2_wasm.o
cp libtess2_wasm.o <project>/nav/lib/
```

The `wasm_shim/setjmp.h` and `wasm_shim/setjmp.c` provide a minimal setjmp/longjmp
stub for WASM. libtess2 uses setjmp/longjmp for error recovery — the shim traps on
longjmp, causing tessellation to return failure rather than crashing.

## Verify

```sh
# Native
odin test nav/

# WASM
odin check nav/ -no-entry-point -target:js_wasm32
```
