# odin-navmesh

2D navigation mesh library for Odin. Uses [libtess2](https://github.com/memononen/libtess2) for polygon tessellation and A* + funnel algorithm for pathfinding.

Pre-built libraries are included for macOS (arm64/amd64), Linux (amd64), Windows (amd64), and WASM.

<img width="1310" height="778" alt="image" src="https://github.com/user-attachments/assets/8b807074-a9b0-4823-ba19-574045aecfef" />

## Usage

```odin
import nav "path/to/odin-navmesh"

// Define walkable area (counter-clockwise) and obstacles (clockwise).
room := [][2]f32{{0, 0}, {800, 0}, {800, 600}, {0, 600}}
obstacle := [][2]f32{{300, 200}, {500, 200}, {500, 400}, {300, 400}}

// Bake the navmesh.
mesh := nav.bake(room, {obstacle}) or_else panic("bake failed")
defer nav.destroy(&mesh)

// Find a path.
path := nav.find_path(&mesh, {50, 300}, {750, 300})
defer delete(path)
```

## API

```odin
bake(outer: []Vec2, holes: [][]Vec2) -> (Nav_Mesh, Bake_Error)
```
Tessellate a walkable polygon with optional holes into a navigation mesh. `outer` must be counter-clockwise, holes must be clockwise.

```odin
destroy(mesh: ^Nav_Mesh)
```
Free the mesh.

```odin
find_path(mesh: ^Nav_Mesh, start, goal: Vec2) -> []Vec2
```
Find a smoothed path between two points. If `goal` is outside the mesh it snaps to the nearest boundary point. Returns `nil` if no path exists. Caller must `delete` the result.

```odin
point_in_mesh(mesh: ^Nav_Mesh, p: Vec2) -> bool
```
Test if a point is inside the walkable area.

```odin
nearest_point_on_mesh_boundary(mesh: ^Nav_Mesh, p: Vec2) -> Vec2
```
Project a point onto the nearest mesh boundary edge.

## Raylib Example

A minimal click-to-move demo:

```odin
package main

import rl "vendor:raylib"
import nav "path/to/odin-navmesh"

main :: proc() {
    rl.InitWindow(800, 600, "navmesh")
    defer rl.CloseWindow()
    rl.SetTargetFPS(60)

    room := [][2]f32{{10, 10}, {790, 10}, {790, 590}, {10, 590}}
    box := [][2]f32{{300, 200}, {300, 400}, {500, 400}, {500, 200}}

    mesh := nav.bake(room, {box}) or_else panic("bake failed")
    defer nav.destroy(&mesh)

    pos: [2]f32 = {50, 300}
    path: [][2]f32
    idx: int

    for !rl.WindowShouldClose() {
        dt := rl.GetFrameTime()

        if rl.IsMouseButtonPressed(.LEFT) {
            m := rl.GetMousePosition()
            target: [2]f32 = {m.x, m.y}
            if !nav.point_in_mesh(&mesh, target) {
                target = nav.nearest_point_on_mesh_boundary(&mesh, target)
            }
            if path != nil do delete(path)
            path = nav.find_path(&mesh, pos, target)
            idx = 0
        }

        if path != nil && idx < len(path) {
            dir := path[idx] - pos
            dist := rl.Vector2Length({dir.x, dir.y})
            if dist < 2 {
                idx += 1
            } else {
                pos += (dir / dist) * min(200 * dt, dist)
            }
        }

        rl.BeginDrawing()
        rl.ClearBackground(rl.BLACK)
        rl.DrawRectangle(300, 200, 200, 200, rl.DARKGRAY)
        rl.DrawCircle(i32(pos.x), i32(pos.y), 8, rl.RED)
        rl.EndDrawing()
    }

    if path != nil do delete(path)
}
```

Run the included example with:
```
odin run examples/raylib/
```

## Building the libraries

The pre-built libtess2 binaries in `lib/` are committed to the repo so you can use the package immediately. To rebuild them (e.g. after an upstream libtess2 update), run the **Build libs** workflow from the GitHub Actions tab, or via the CLI:

```
gh workflow run build-libs.yml
```

This builds libtess2 for all platforms and commits the updated binaries to `lib/`.

## Running tests

```
odin test .
```
