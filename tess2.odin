package nav

// Odin bindings for libtess2 (Mikko Mononen's polygon tessellation library).
// Based on bindings from github.com/varomix/ohcad.

import "core:c"

when ODIN_ARCH == .wasm32 || ODIN_ARCH == .wasm64p32 {
	foreign import libtess2 "lib/libtess2_wasm.o"
} else when ODIN_OS == .Darwin && ODIN_ARCH == .arm64 {
	foreign import libtess2 "lib/libtess2_darwin_arm64.a"
} else when ODIN_OS == .Darwin && ODIN_ARCH == .amd64 {
	foreign import libtess2 "lib/libtess2_darwin_amd64.a"
} else when ODIN_OS == .Linux {
	foreign import libtess2 "lib/libtess2_linux_amd64.a"
} else when ODIN_OS == .Windows {
	foreign import libtess2 "lib/libtess2_windows_amd64.lib"
} else {
	foreign import libtess2 "system:tess2"
}

Tess_Winding_Rule :: enum c.int {
	Odd,
	Nonzero,
	Positive,
	Negative,
	Abs_Geq_Two,
}

Tess_Element_Type :: enum c.int {
	Polygons,
	Connected_Polygons,
	Boundary_Contours,
}

Tess_Option :: enum c.int {
	Constrained_Delaunay_Triangulation,
	Reverse_Contours,
}

Tess_Status :: enum c.int {
	Ok,
	Out_Of_Memory,
	Invalid_Input,
}

TESS_UNDEF :: ~c.int(0)

@(default_calling_convention = "c", link_prefix = "tess")
foreign libtess2 {
	NewTess :: proc(alloc: rawptr) -> rawptr ---
	DeleteTess :: proc(tess: rawptr) ---
	AddContour :: proc(tess: rawptr, size: c.int, pointer: rawptr, stride: c.int, count: c.int) ---
	SetOption :: proc(tess: rawptr, option: c.int, value: c.int) ---
	Tesselate :: proc(tess: rawptr, winding_rule: c.int, element_type: c.int, poly_size: c.int, vertex_size: c.int, normal: [^]f32) -> c.int ---
	GetVertexCount :: proc(tess: rawptr) -> c.int ---
	GetVertices :: proc(tess: rawptr) -> [^]f32 ---
	GetVertexIndices :: proc(tess: rawptr) -> [^]c.int ---
	GetElementCount :: proc(tess: rawptr) -> c.int ---
	GetElements :: proc(tess: rawptr) -> [^]c.int ---
	GetStatus :: proc(tess: rawptr) -> Tess_Status ---
}
