package navmesh

// 2D Navigation Mesh for Point-and-Click Adventure Games
//
// Uses libtess2 for robust polygon tessellation with hole support,
// then A* + Simple Stupid Funnel for runtime pathfinding.
//
// Usage:
//   outer := [][2]f32{{0,0}, {800,0}, {800,600}, {0,600}}
//   holes := [][][2]f32{
//       {{300,200}, {500,200}, {500,400}, {300,400}},
//   }
//   mesh := nav.bake(outer, holes) or_else panic("bake failed")
//   defer nav.destroy(&mesh)
//
//   path := nav.find_path(&mesh, {50, 300}, {750, 300})
//   defer delete(path)

import "core:c"
import "core:math"

Vec2 :: [2]f32
Triangle :: [3]i32

Nav_Mesh :: struct {
	vertices:  []Vec2,
	triangles: []Triangle,
	adjacency: [][3]i32,
}

Bake_Error :: enum {
	None,
	Too_Few_Vertices,
	Tessellation_Failed,
}

// Bake a navigation mesh from an outer polygon boundary and optional holes.
//
// `outer` must be counter-clockwise wound.
// Each hole must be clockwise wound.
bake :: proc(
	outer: []Vec2,
	holes: [][]Vec2 = nil,
	allocator := context.allocator,
) -> (
	mesh: Nav_Mesh,
	err: Bake_Error,
) {
	if len(outer) < 3 {
		err = .Too_Few_Vertices
		return
	}

	tess := NewTess(nil)
	if tess == nil {
		err = .Tessellation_Failed
		return
	}
	defer DeleteTess(tess)

	// Enable constrained Delaunay for better triangle quality.
	SetOption(tess, c.int(Tess_Option.Constrained_Delaunay_Triangulation), 1)

	// Add the outer boundary contour.
	AddContour(tess, 2, raw_data(outer), size_of(Vec2), c.int(len(outer)))

	// Add each hole as a separate contour.
	for hole in holes {
		if len(hole) >= 3 {
			AddContour(tess, 2, raw_data(hole), size_of(Vec2), c.int(len(hole)))
		}
	}

	// Tessellate with CONNECTED_POLYGONS to get adjacency info.
	normal := [3]f32{0, 0, 1}
	result := Tesselate(
		tess,
		c.int(Tess_Winding_Rule.Odd),
		c.int(Tess_Element_Type.Connected_Polygons),
		3, // triangles
		2, // 2D
		raw_data(&normal),
	)

	if result == 0 {
		err = .Tessellation_Failed
		return
	}

	vert_count := int(GetVertexCount(tess))
	tri_count := int(GetElementCount(tess))
	verts_ptr := GetVertices(tess)
	elems_ptr := GetElements(tess)

	if verts_ptr == nil || elems_ptr == nil || tri_count == 0 {
		err = .Tessellation_Failed
		return
	}

	// Copy vertices from libtess2 output.
	vertices := make([]Vec2, vert_count, allocator)
	for i in 0 ..< vert_count {
		vertices[i] = {verts_ptr[i * 2], verts_ptr[i * 2 + 1]}
	}

	// Copy triangles and adjacency from CONNECTED_POLYGONS output.
	// Each element is: [v0, v1, v2, n0, n1, n2] (6 ints per triangle).
	triangles := make([]Triangle, tri_count, allocator)
	adjacency := make([][3]i32, tri_count, allocator)

	for i in 0 ..< tri_count {
		base := i * 6
		triangles[i] = {
			i32(elems_ptr[base + 0]),
			i32(elems_ptr[base + 1]),
			i32(elems_ptr[base + 2]),
		}
		for e in 0 ..< 3 {
			nb := elems_ptr[base + 3 + e]
			adjacency[i][e] = nb == TESS_UNDEF ? -1 : i32(nb)
		}
	}

	mesh = Nav_Mesh {
		vertices  = vertices,
		triangles = triangles,
		adjacency = adjacency,
	}
	return
}

destroy :: proc(mesh: ^Nav_Mesh) {
	delete(mesh.vertices)
	delete(mesh.triangles)
	delete(mesh.adjacency)
	mesh^ = {}
}

// Find a smoothed path from `start` to `goal` within the nav mesh.
find_path :: proc(mesh: ^Nav_Mesh, start, goal: Vec2, allocator := context.allocator) -> []Vec2 {
	actual_goal := goal
	goal_tri := _find_containing_triangle(mesh, goal)
	if goal_tri < 0 {
		actual_goal = _nearest_point_on_mesh(mesh, goal)
		goal_tri = _find_containing_triangle(mesh, actual_goal)
		if goal_tri < 0 {
			return nil
		}
	}

	start_tri := _find_containing_triangle(mesh, start)
	if start_tri < 0 {
		return nil
	}

	if start_tri == goal_tri {
		result := make([]Vec2, 2, allocator)
		result[0] = start
		result[1] = actual_goal
		return result
	}

	tri_path := _astar(mesh, start_tri, goal_tri, start, actual_goal, allocator)
	if tri_path == nil {
		return nil
	}
	defer delete(tri_path)

	return _funnel(mesh, tri_path, start, actual_goal, allocator)
}

// Test if a point is inside the walkable area of the nav mesh.
point_in_mesh :: proc(mesh: ^Nav_Mesh, p: Vec2) -> bool {
	return _find_containing_triangle(mesh, p) >= 0
}

// Find the closest point on the mesh boundary to a given point.
nearest_point_on_mesh_boundary :: proc(mesh: ^Nav_Mesh, p: Vec2) -> Vec2 {
	return _nearest_point_on_mesh(mesh, p)
}


// -- Geometry utilities -------------------------------------------------------

cross2d :: proc(a, b: Vec2) -> f32 {
	return a.x * b.y - a.y * b.x
}

dot2d :: proc(a, b: Vec2) -> f32 {
	return a.x * b.x + a.y * b.y
}

dist2d :: proc(a, b: Vec2) -> f32 {
	d := b - a
	return math.sqrt(d.x * d.x + d.y * d.y)
}

dist_sq :: proc(a, b: Vec2) -> f32 {
	d := b - a
	return d.x * d.x + d.y * d.y
}

closest_point_on_segment :: proc(a, b, p: Vec2) -> Vec2 {
	ab := b - a
	len_sq := dot2d(ab, ab)
	if len_sq < 1e-12 do return a
	t := clamp(dot2d(p - a, ab) / len_sq, 0, 1)
	return a + ab * t
}
