# Case

The case is modeled in [build123d](https://github.com/gumyr/build123d).

| Path | What |
|---|---|
| `bottom.py` | Bottom case of the left half |
| `v0.1.0/` | The v0.1.0 prototype, modeled in FreeCAD (`bottom.FCStd`, `top.FCStd` and the DXF sketches they use) |

`bottom.py` is a rewrite of `v0.1.0/bottom.FCStd` (Body001, the left half). It
produces the same shape as the FreeCAD model. The right half is its mirror
image.

## Usage

The project is managed with [uv](https://docs.astral.sh/uv/).

```sh
cd case
uv sync
uv run python bottom.py          # writes out/bottom-left.step and .stl
uv run python bottom.py --show   # also shows the part in the OCP CAD Viewer
```
