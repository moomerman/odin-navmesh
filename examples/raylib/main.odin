// Nav2D raylib demo — click to pathfind around obstacles.
// Press M to toggle navmesh wireframe.
//
// Build:  odin run examples/raylib/
package main

import "core:math"

import rl "vendor:raylib"

import nav "../.."

SPEED :: 250.0
ARRIVE_DIST :: 3.0

mesh: nav.Nav_Mesh
bake_ok: bool
char_pos: [2]f32
current_path: [][2]f32
path_idx: int
show_mesh: bool

main :: proc() {
	rl.InitWindow(1280, 720, "Nav2D Demo (raylib)")
	defer rl.CloseWindow()

	rl.SetTargetFPS(60)

	init()
	defer shutdown()

	for !rl.WindowShouldClose() {
		frame(rl.GetFrameTime())
	}
}

init :: proc() {
	show_mesh = true

	// Room boundary (counter-clockwise winding).
	room := [][2]f32{{40, 40}, {1240, 40}, {1240, 680}, {40, 680}}

	// Obstacles (clockwise winding).
	desk := [][2]f32{{200, 120}, {200, 250}, {400, 250}, {400, 120}}
	shelf := [][2]f32{{750, 80}, {750, 280}, {870, 280}, {870, 80}}
	couch := [][2]f32{{500, 350}, {500, 490}, {800, 490}, {800, 350}}
	table := [][2]f32{{120, 480}, {120, 610}, {360, 610}, {360, 480}}
	plant := [][2]f32{{950, 500}, {950, 620}, {1050, 620}, {1050, 500}}

	err: nav.Bake_Error
	mesh, err = nav.bake(room, {desk, shelf, couch, table, plant})
	bake_ok = err == .None

	char_pos = {80, 360}
}

frame :: proc(dt: f32) {
	if !bake_ok {
		rl.BeginDrawing()
		rl.ClearBackground({25, 25, 35, 255})
		rl.DrawText("Navmesh bake failed!", 100, 350, 24, rl.RED)
		rl.EndDrawing()
		return
	}

	if rl.IsKeyPressed(.M) do show_mesh = !show_mesh

	// Click to move.
	if rl.IsMouseButtonPressed(.LEFT) {
		click := rl.GetMousePosition()
		target: [2]f32 = {click.x, click.y}

		if !nav.point_in_mesh(&mesh, target) {
			target = nav.nearest_point_on_mesh_boundary(&mesh, target)
		}

		if current_path != nil do delete(current_path)
		current_path = nav.find_path(&mesh, char_pos, target)
		path_idx = 0
	}

	// Follow path.
	if current_path != nil && path_idx < len(current_path) {
		wp := current_path[path_idx]
		dir := wp - char_pos
		dist := math.sqrt(nav.dot2d(dir, dir))

		if dist < ARRIVE_DIST {
			path_idx += 1
		} else {
			step := min(SPEED * dt, dist)
			char_pos += (dir / dist) * step
		}
	}

	// -- Draw --
	rl.BeginDrawing()
	defer rl.EndDrawing()

	rl.ClearBackground({25, 25, 35, 255})

	// Room floor.
	rl.DrawRectangle(40, 40, 1200, 640, {42, 46, 56, 255})

	// Navmesh wireframe.
	if show_mesh {
		for tri in mesh.triangles {
			a := mesh.vertices[tri[0]]
			b := mesh.vertices[tri[1]]
			c := mesh.vertices[tri[2]]
			mesh_color := rl.Color{58, 64, 82, 255}
			rl.DrawLine(i32(a.x), i32(a.y), i32(b.x), i32(b.y), mesh_color)
			rl.DrawLine(i32(b.x), i32(b.y), i32(c.x), i32(c.y), mesh_color)
			rl.DrawLine(i32(c.x), i32(c.y), i32(a.x), i32(a.y), mesh_color)
		}
	}

	// Obstacles.
	_draw_obstacle({200, 120, 200, 130}, "desk")
	_draw_obstacle({750, 80, 120, 200}, "shelf")
	_draw_obstacle({500, 350, 300, 140}, "couch")
	_draw_obstacle({120, 480, 240, 130}, "table")
	_draw_obstacle({950, 500, 100, 120}, "plant")

	// Room border.
	rl.DrawRectangleLines(40, 40, 1200, 640, {90, 98, 118, 255})

	// Path visualization.
	if current_path != nil && len(current_path) > 0 {
		path_color := rl.Color{90, 180, 240, 180}

		// Line from character to next waypoint.
		if path_idx < len(current_path) {
			p := current_path[path_idx]
			rl.DrawLine(i32(char_pos.x), i32(char_pos.y), i32(p.x), i32(p.y), path_color)
		}

		// Remaining path segments.
		for i in path_idx ..< len(current_path) - 1 {
			p0 := current_path[i]
			p1 := current_path[i + 1]
			rl.DrawLine(i32(p0.x), i32(p0.y), i32(p1.x), i32(p1.y), path_color)
		}

		// Waypoint dots.
		for i in path_idx ..< len(current_path) {
			p := current_path[i]
			rl.DrawRectangle(i32(p.x) - 3, i32(p.y) - 3, 6, 6, {90, 180, 240, 255})
		}

		// Target crosshair.
		goal := current_path[len(current_path) - 1]
		cross_color := rl.Color{255, 200, 50, 255}
		rl.DrawLine(i32(goal.x) - 10, i32(goal.y), i32(goal.x) + 10, i32(goal.y), cross_color)
		rl.DrawLine(i32(goal.x), i32(goal.y) - 10, i32(goal.x), i32(goal.y) + 10, cross_color)
	}

	// Character.
	rl.DrawRectangle(i32(char_pos.x) - 10, i32(char_pos.y) - 10, 20, 20, {210, 70, 50, 255})
	rl.DrawRectangle(i32(char_pos.x) - 8, i32(char_pos.y) - 8, 16, 16, {240, 110, 75, 255})

	// Mouse hover — green dot if walkable, red if not.
	mouse := rl.GetMousePosition()
	mouse_pos: [2]f32 = {mouse.x, mouse.y}
	walkable := nav.point_in_mesh(&mesh, mouse_pos)
	dot_color := walkable ? rl.Color{80, 200, 100, 200} : rl.Color{200, 80, 80, 200}
	rl.DrawRectangle(i32(mouse.x) - 2, i32(mouse.y) - 2, 4, 4, dot_color)

	// HUD.
	rl.DrawText("Click to move  |  M: toggle mesh  |  ESC: quit", 50, 15, 14, {120, 125, 140, 200})
	rl.DrawFPS(1200, 15)
}

shutdown :: proc() {
	if current_path != nil do delete(current_path)
	nav.destroy(&mesh)
}

@(private = "file")
_draw_obstacle :: proc(rect: rl.Rectangle, label: cstring) {
	rl.DrawRectangleRec(rect, {32, 35, 45, 255})
	rl.DrawRectangleLinesEx(rect, 2, {72, 78, 98, 255})
	font_size: i32 = 14
	text_w := rl.MeasureText(label, font_size)
	rl.DrawText(
		label,
		i32(rect.x) + (i32(rect.width) - text_w) / 2,
		i32(rect.y) + (i32(rect.height) - font_size) / 2,
		font_size,
		{85, 90, 108, 255},
	)
}
