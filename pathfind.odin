package navmesh

// Runtime pathfinding: A*, funnel algorithm, mesh queries.

import "core:math"

// -- Triangle lookup ----------------------------------------------------------

@(private)
_find_containing_triangle :: proc(mesh: ^Nav_Mesh, p: Vec2) -> i32 {
	for ti in 0 ..< i32(len(mesh.triangles)) {
		tri := mesh.triangles[ti]
		a := mesh.vertices[tri[0]]
		b := mesh.vertices[tri[1]]
		c := mesh.vertices[tri[2]]
		if _point_in_triangle(p, a, b, c) do return ti
	}
	// Point may be on an edge due to floating point — find the nearest triangle.
	return _nearest_triangle(mesh, p)
}

@(private)
_nearest_triangle :: proc(mesh: ^Nav_Mesh, p: Vec2) -> i32 {
	best: i32 = -1
	best_dist: f32 = math.F32_MAX
	for ti in 0 ..< i32(len(mesh.triangles)) {
		tri := mesh.triangles[ti]
		a := mesh.vertices[tri[0]]
		b := mesh.vertices[tri[1]]
		c := mesh.vertices[tri[2]]
		// Distance to nearest edge of this triangle.
		d := min(
			_dist_sq_to_segment(p, a, b),
			_dist_sq_to_segment(p, b, c),
			_dist_sq_to_segment(p, c, a),
		)
		if d < best_dist {
			best_dist = d
			best = ti
		}
	}
	// Only snap if very close (within 5px) — otherwise truly outside.
	if best_dist > 25.0 {
		return -1
	}
	return best
}

@(private)
_dist_sq_to_segment :: proc(p, a, b: Vec2) -> f32 {
	return dist_sq(p, closest_point_on_segment(a, b, p))
}

@(private)
_point_in_triangle :: proc(p, a, b, c_: Vec2) -> bool {
	d1 := cross2d(b - a, p - a)
	d2 := cross2d(c_ - b, p - b)
	d3 := cross2d(a - c_, p - c_)
	has_neg := (d1 < 0) || (d2 < 0) || (d3 < 0)
	has_pos := (d1 > 0) || (d2 > 0) || (d3 > 0)
	return !(has_neg && has_pos)
}


// -- Nearest point on mesh boundary -------------------------------------------

@(private)
_nearest_point_on_mesh :: proc(mesh: ^Nav_Mesh, p: Vec2) -> Vec2 {
	best_point := Vec2{0, 0}
	best_dist: f32 = math.F32_MAX

	for ti in 0 ..< len(mesh.triangles) {
		tri := mesh.triangles[ti]
		for e in 0 ..< 3 {
			if mesh.adjacency[ti][e] >= 0 do continue
			a := mesh.vertices[tri[e]]
			b := mesh.vertices[tri[(e + 1) % 3]]
			cp := closest_point_on_segment(a, b, p)
			d := dist_sq(p, cp)
			if d < best_dist {
				best_dist = d
				best_point = cp
			}
		}
	}
	return best_point
}


// -- A* over triangle graph ---------------------------------------------------

@(private)
_astar :: proc(
	mesh: ^Nav_Mesh,
	start_tri, goal_tri: i32,
	start_pos, goal_pos: Vec2,
	allocator := context.allocator,
) -> []i32 {
	n := i32(len(mesh.triangles))

	g_score := make([]f32, n, allocator)
	defer delete(g_score)
	f_score := make([]f32, n, allocator)
	defer delete(f_score)
	came_from := make([]i32, n, allocator)
	defer delete(came_from)
	// Track the point we "entered" each triangle through (edge midpoint).
	entry_point := make([]Vec2, n, allocator)
	defer delete(entry_point)
	in_open := make([]bool, n, allocator)
	defer delete(in_open)
	closed := make([]bool, n, allocator)
	defer delete(closed)

	for i in 0 ..< n {
		g_score[i] = math.F32_MAX
		f_score[i] = math.F32_MAX
		came_from[i] = -1
	}

	g_score[start_tri] = 0
	f_score[start_tri] = dist2d(start_pos, goal_pos)
	entry_point[start_tri] = start_pos

	open := make([dynamic]i32, allocator)
	defer delete(open)
	append(&open, start_tri)
	in_open[start_tri] = true

	for len(open) > 0 {
		best_oi := 0
		best_f: f32 = math.F32_MAX
		for oi in 0 ..< len(open) {
			f := f_score[open[oi]]
			if f < best_f {
				best_f = f
				best_oi = oi
			}
		}

		current := open[best_oi]
		if current == goal_tri {
			path := make([dynamic]i32, allocator)
			node := current
			for node >= 0 {
				inject_at(&path, 0, node)
				node = came_from[node]
			}
			return path[:]
		}

		ordered_remove(&open, best_oi)
		in_open[current] = false
		closed[current] = true

		cur_entry := entry_point[current]

		for edge in 0 ..< 3 {
			neighbor := mesh.adjacency[current][edge]
			if neighbor < 0 || closed[neighbor] do continue

			// Use the point on the shared edge closest to where we are now
			// as the crossing point. This gives an accurate g-cost (minimum
			// distance to reach this edge) and a reasonable h-cost estimate.
			edge_pt := _edge_crossing(mesh, current, edge, cur_entry)
			tentative_g := g_score[current] + dist2d(cur_entry, edge_pt)

			if tentative_g < g_score[neighbor] {
				came_from[neighbor] = current
				g_score[neighbor] = tentative_g
				f_score[neighbor] = tentative_g + dist2d(edge_pt, goal_pos)
				entry_point[neighbor] = edge_pt

				if !in_open[neighbor] {
					append(&open, neighbor)
					in_open[neighbor] = true
				}
			}
		}
	}

	return nil
}

@(private)
_edge_crossing :: proc(mesh: ^Nav_Mesh, ti: i32, edge: int, target: Vec2) -> Vec2 {
	tri := mesh.triangles[ti]
	a := mesh.vertices[tri[edge]]
	b := mesh.vertices[tri[(edge + 1) % 3]]
	return closest_point_on_segment(a, b, target)
}


// -- Funnel algorithm (Simple Stupid Funnel) ----------------------------------

@(private)
_funnel :: proc(
	mesh: ^Nav_Mesh,
	tri_path: []i32,
	start, goal: Vec2,
	allocator := context.allocator,
) -> []Vec2 {
	if len(tri_path) == 0 do return nil

	if len(tri_path) == 1 {
		result := make([]Vec2, 2, allocator)
		result[0] = start
		result[1] = goal
		return result
	}

	Portal :: struct {
		left:  Vec2,
		right: Vec2,
	}

	portals := make([dynamic]Portal, allocator)
	defer delete(portals)

	append(&portals, Portal{left = start, right = start})

	for i in 0 ..< len(tri_path) - 1 {
		ti_a := tri_path[i]
		ti_b := tri_path[i + 1]
		l, r := _find_shared_edge(mesh, ti_a, ti_b)
		append(&portals, Portal{left = l, right = r})
	}

	append(&portals, Portal{left = goal, right = goal})

	// Simple Stupid Funnel Algorithm.
	path := make([dynamic]Vec2, allocator)
	append(&path, start)

	apex := start
	left := start
	right := start
	apex_idx := 0
	left_idx := 0
	right_idx := 0

	i := 1
	for i < len(portals) {
		pl := portals[i].left
		pr := portals[i].right

		// Tighten right side.
		if cross2d(pr - apex, right - apex) <= 0 {
			if apex == right || cross2d(pr - apex, left - apex) > 0 {
				right = pr
				right_idx = i
			} else {
				append(&path, left)
				apex = left
				apex_idx = left_idx
				left = apex
				right = apex
				left_idx = apex_idx
				right_idx = apex_idx
				i = apex_idx + 1
				continue
			}
		}

		// Tighten left side.
		if cross2d(pl - apex, left - apex) >= 0 {
			if apex == left || cross2d(pl - apex, right - apex) < 0 {
				left = pl
				left_idx = i
			} else {
				append(&path, right)
				apex = right
				apex_idx = right_idx
				left = apex
				right = apex
				left_idx = apex_idx
				right_idx = apex_idx
				i = apex_idx + 1
				continue
			}
		}

		i += 1
	}

	last := path[len(path) - 1]
	if last.x != goal.x || last.y != goal.y {
		append(&path, goal)
	}

	return path[:]
}

@(private = "file")
_find_shared_edge :: proc(mesh: ^Nav_Mesh, tri_a, tri_b: i32) -> (left, right: Vec2) {
	ta := mesh.triangles[tri_a]
	tb := mesh.triangles[tri_b]

	shared: [2]i32
	count := 0
	for i in 0 ..< 3 {
		for j in 0 ..< 3 {
			if ta[i] == tb[j] {
				if count < 2 {
					shared[count] = ta[i]
					count += 1
				}
			}
		}
	}

	if count < 2 {
		return mesh.vertices[ta[0]], mesh.vertices[ta[1]]
	}

	s0 := mesh.vertices[shared[0]]
	s1 := mesh.vertices[shared[1]]

	// Find the third vertex of tri_a (not on the shared edge).
	third_vi: i32 = -1
	for i in 0 ..< 3 {
		if ta[i] != shared[0] && ta[i] != shared[1] {
			third_vi = ta[i]
			break
		}
	}

	third := mesh.vertices[third_vi]
	if cross2d(s1 - s0, third - s0) > 0 {
		return s1, s0
	}
	return s0, s1
}
