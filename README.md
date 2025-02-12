# molgif

`molgif` generates beautiful GIFs of rotating molecules using SDF as input.

![Caffeine](./resources/caffeine.gif)

short intro:

```bash
go build -o molgif main.go
```

```bash
./molgif --help

Usage of ./molgif:
  -ambient float
        Ambient light coefficient (default 0.7)
  -diffuse float
        Diffuse light coefficient (default 1)
  -fov float
        Vertical field of view (radians) (default 1.0471975511965979)
  -frames int
        Total number of frames in the animation (default 120)
  -height int
        Image height in pixels (default 1080)
  -in string
        Path to the input SDF file (required)
  -out string
        Path to the output GIF file (default "output.gif")
  -shininess float
        Shininess coefficient for specular highlights (default 20)
  -specular float
        Specular light coefficient (default 0.7)
  -tilt float
        Tilt angle (radians) applied about the X–axis
  -width int
        Image width in pixels (default 1920)
  -zoffset float
        Camera translation along Z axis (default 2.25)
```

Basic usage:

```bash
./molgif -in resources/caffeine.sdf
```

have fun
