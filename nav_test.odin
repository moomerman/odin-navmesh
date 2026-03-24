#+build !js
package nav

import "core:testing"

@(test)
test_bake_simple_square :: proc(t: ^testing.T) {
	outer := []Vec2{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
	mesh, err := bake(outer)
	defer destroy(&mesh)

	testing.expect(t, err == .None, "bake should succeed")
	testing.expect(t, len(mesh.triangles) >= 2, "square should produce at least 2 triangles")
	testing.expect(t, len(mesh.vertices) >= 4, "should have at least 4 vertices")
}

@(test)
test_bake_with_hole :: proc(t: ^testing.T) {
	outer := []Vec2{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
	holes := [][]Vec2{{{300, 200}, {500, 200}, {500, 400}, {300, 400}}}
	mesh, err := bake(outer, holes)
	defer destroy(&mesh)

	testing.expect(t, err == .None, "bake with hole should succeed")
	testing.expect(t, len(mesh.triangles) > 2, "hole should produce more triangles")
}

@(test)
test_point_in_mesh :: proc(t: ^testing.T) {
	outer := []Vec2{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
	holes := [][]Vec2{{{300, 200}, {500, 200}, {500, 400}, {300, 400}}}
	mesh, err := bake(outer, holes)
	defer destroy(&mesh)
	testing.expect(t, err == .None)

	// Point in walkable area should be in mesh.
	testing.expect(t, point_in_mesh(&mesh, {50, 50}), "corner should be in mesh")
	testing.expect(t, point_in_mesh(&mesh, {750, 300}), "right side should be in mesh")

	// Point inside the hole should NOT be in mesh.
	testing.expect(t, !point_in_mesh(&mesh, {400, 300}), "hole center should not be in mesh")

	// Point outside the boundary should NOT be in mesh.
	testing.expect(t, !point_in_mesh(&mesh, {-50, 300}), "outside should not be in mesh")
}

@(test)
test_find_path_simple :: proc(t: ^testing.T) {
	outer := []Vec2{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
	mesh, err := bake(outer)
	defer destroy(&mesh)
	testing.expect(t, err == .None)

	path := find_path(&mesh, {50, 300}, {750, 300})
	defer delete(path)

	testing.expect(t, path != nil, "path should exist")
	testing.expect(t, len(path) >= 2, "path should have at least start and goal")
	testing.expect(t, path[0] == Vec2{50, 300}, "path should start at start")
	testing.expect(t, path[len(path) - 1] == Vec2{750, 300}, "path should end at goal")
}

@(test)
test_find_path_around_obstacle :: proc(t: ^testing.T) {
	outer := []Vec2{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
	holes := [][]Vec2{{{300, 200}, {500, 200}, {500, 400}, {300, 400}}}
	mesh, err := bake(outer, holes)
	defer destroy(&mesh)
	testing.expect(t, err == .None)

	path := find_path(&mesh, {100, 300}, {700, 300})
	defer delete(path)

	testing.expect(t, path != nil, "path around obstacle should exist")
	testing.expect(t, len(path) >= 3, "path should have waypoints to go around obstacle")

	// Verify no path point is inside the hole.
	for pt in path {
		in_hole := pt.x > 300 && pt.x < 500 && pt.y > 200 && pt.y < 400
		testing.expect(t, !in_hole, "path should not go through obstacle")
	}
}

@(test)
test_find_path_multiple_holes :: proc(t: ^testing.T) {
	outer := []Vec2{{0, 0}, {1000, 0}, {1000, 600}, {0, 600}}
	holes := [][]Vec2 {
		{{200, 100}, {400, 100}, {400, 500}, {200, 500}}, // left wall
		{{600, 100}, {800, 100}, {800, 500}, {600, 500}}, // right wall
	}
	mesh, err := bake(outer, holes)
	defer destroy(&mesh)
	testing.expect(t, err == .None)

	path := find_path(&mesh, {100, 300}, {900, 300})
	defer delete(path)

	testing.expect(t, path != nil, "path through corridor should exist")
	testing.expect(t, len(path) >= 2, "path should have waypoints")
}

@(test)
test_goal_outside_mesh :: proc(t: ^testing.T) {
	outer := []Vec2{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
	mesh, err := bake(outer)
	defer destroy(&mesh)
	testing.expect(t, err == .None)

	// Goal outside the mesh should be clamped to boundary.
	path := find_path(&mesh, {400, 300}, {1000, 300})
	defer delete(path)

	testing.expect(t, path != nil, "path to outside goal should snap to boundary")
	last := path[len(path) - 1]
	testing.expect(t, last.x <= 800, "goal should be clamped to mesh boundary")
}

@(test)
test_path_quality_demo_scene :: proc(t: ^testing.T) {
	// The actual demo scene: room with 5 obstacles.
	room := []Vec2{{40, 40}, {1240, 40}, {1240, 680}, {40, 680}}
	desk := []Vec2{{200, 120}, {200, 250}, {400, 250}, {400, 120}}
	shelf := []Vec2{{750, 80}, {750, 280}, {870, 280}, {870, 80}}
	couch := []Vec2{{500, 350}, {500, 490}, {800, 490}, {800, 350}}
	table := []Vec2{{120, 480}, {120, 610}, {360, 610}, {360, 480}}
	plant := []Vec2{{950, 500}, {950, 620}, {1050, 620}, {1050, 500}}

	mesh, err := bake(room, {desk, shelf, couch, table, plant})
	defer destroy(&mesh)
	testing.expect(t, err == .None)

	// Left to right across the room at mid-height.
	// Direct distance is ~1120. A reasonable path should not exceed 1.2x.
	{
		path := find_path(&mesh, {80, 360}, {1200, 360})
		defer delete(path)
		testing.expect(t, path != nil, "left-to-right path should exist")
		path_len: f32 = 0
		for i in 0 ..< len(path) - 1 {
			path_len += dist2d(path[i], path[i + 1])
		}
		direct := dist2d({80, 360}, {1200, 360})
		testing.expectf(
			t,
			path_len / direct < 1.2,
			"left-to-right path too long (len=%.0f, direct=%.0f, ratio=%.2f)",
			path_len,
			direct,
			path_len / direct,
		)
	}

	// Left of desk to right of couch — should not detour via top of room.
	{
		path := find_path(&mesh, {150, 300}, {900, 400})
		defer delete(path)
		testing.expect(t, path != nil, "desk-to-couch path should exist")
		path_len: f32 = 0
		for i in 0 ..< len(path) - 1 {
			path_len += dist2d(path[i], path[i + 1])
		}
		direct := dist2d({150, 300}, {900, 400})
		testing.expectf(
			t,
			path_len / direct < 1.2,
			"desk-to-couch path too long (len=%.0f, direct=%.0f, ratio=%.2f)",
			path_len,
			direct,
			path_len / direct,
		)
	}

	// Around shelf — should go over the top, not around the bottom.
	{
		path := find_path(&mesh, {730, 150}, {900, 70})
		defer delete(path)
		testing.expect(t, path != nil, "shelf path should exist")
		went_below := false
		for pt in path {
			if pt.y > 280 {went_below = true}
		}
		testing.expect(t, !went_below, "shelf path should go over top, not around bottom")
	}
}

@(test)
test_path_symmetric :: proc(t: ^testing.T) {
	// Symmetric obstacle in the center. Paths from left-center to right-center
	// should have similar length regardless of direction.
	outer := []Vec2{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
	holes := [][]Vec2{{{350, 200}, {450, 200}, {450, 400}, {350, 400}}}
	mesh, err := bake(outer, holes)
	defer destroy(&mesh)
	testing.expect(t, err == .None)

	path_lr := find_path(&mesh, {100, 300}, {700, 300})
	defer delete(path_lr)
	path_rl := find_path(&mesh, {700, 300}, {100, 300})
	defer delete(path_rl)

	testing.expect(t, path_lr != nil, "left-to-right path should exist")
	testing.expect(t, path_rl != nil, "right-to-left path should exist")

	len_lr: f32 = 0
	for i in 0 ..< len(path_lr) - 1 {
		len_lr += dist2d(path_lr[i], path_lr[i + 1])
	}
	len_rl: f32 = 0
	for i in 0 ..< len(path_rl) - 1 {
		len_rl += dist2d(path_rl[i], path_rl[i + 1])
	}

	// Both paths should be similar length (within 20%).
	ratio := len_lr / len_rl if len_lr > len_rl else len_rl / len_lr
	testing.expectf(
		t,
		ratio < 1.2,
		"paths should be similar length (L->R=%.0f, R->L=%.0f, ratio=%.2f)",
		len_lr,
		len_rl,
		ratio,
	)
}

@(test)
test_nearest_point_on_boundary :: proc(t: ^testing.T) {
	outer := []Vec2{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
	mesh, err := bake(outer)
	defer destroy(&mesh)
	testing.expect(t, err == .None)

	nearest := nearest_point_on_mesh_boundary(&mesh, {900, 300})
	testing.expect(t, nearest.x <= 800 + 1, "nearest should be on right boundary")

	nearest2 := nearest_point_on_mesh_boundary(&mesh, {-100, 300})
	testing.expect(t, nearest2.x >= -1, "nearest should be on left boundary")
}
