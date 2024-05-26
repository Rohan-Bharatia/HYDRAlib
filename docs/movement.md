# Movement
## Overview
The ```Vertex``` struct and ```Path``` class are used to facilitate robot movement by representing points on a coordinate system.

## ```Vertex```
```cpp
struct Vertex
{
    float x;
    float y;
    float theta;
}
```

### Parameters
 * ```x```: The X position of the vertex.
 * ```y```: The Y position of the vertex.
 * ```theta```: The angle of the vertex.

### Implementation
```cpp
Vertex vert(0, 0, 90);
```

### Description / Extra Notes
Verticies covers basic geometric movement through linear motion. Verticies also support basic arithmatic logic like:
1. Addition by another vertex.
2. Subtraction by another vertex.
3. Multiplication by another vertex.
4. Subtraction by a scalar.
5. Division by another vertex.
6. Subtraction by a scalar.

## ```Path```
```cpp
Path(Vertex a, Vertex b);
```

### Parameters
 * ```a```: The starting vertex of a path.
 * ```b```: The ending vertex of a path.

### Implementation
```cpp
Vertex start(0, 0, 90);
Vertex end(100, 100, 90);
Path path(start, end);
```

### Description / Extra Notes
This class covers a more complex system through bézier curves rather than straight lines.
