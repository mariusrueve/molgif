// main.go
package main

import (
	"bufio"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/color/palette"
	"image/gif"
	"log"
	"math"
	"os"
	"strconv"
	"strings"
	"sync"
)

// Vec3 is a simple 3D vector.
type Vec3 struct {
	X, Y, Z float64
}

func (v Vec3) Add(o Vec3) Vec3    { return Vec3{v.X + o.X, v.Y + o.Y, v.Z + o.Z} }
func (v Vec3) Sub(o Vec3) Vec3    { return Vec3{v.X - o.X, v.Y - o.Y, v.Z - o.Z} }
func (v Vec3) Mul(s float64) Vec3 { return Vec3{v.X * s, v.Y * s, v.Z * s} }
func (v Vec3) Dot(o Vec3) float64 { return v.X*o.X + v.Y*o.Y + v.Z*o.Z }
func (v Vec3) Length() float64    { return math.Sqrt(v.Dot(v)) }
func (v Vec3) Normalize() Vec3 {
	l := v.Length()
	if l == 0 {
		return v
	}
	return v.Mul(1 / l)
}
func (v Vec3) Cross(o Vec3) Vec3 {
	return Vec3{
		X: v.Y*o.Z - v.Z*o.Y,
		Y: v.Z*o.X - v.X*o.Z,
		Z: v.X*o.Y - v.Y*o.X,
	}
}
func (v Vec3) String() string { return fmt.Sprintf("(%f,%f,%f)", v.X, v.Y, v.Z) }

// Sphere represents an atom (rendered as a sphere).
type Sphere struct {
	center Vec3
	radius float64
	color  color.RGBA
}

// Cylinder represents a bond (rendered as a finite cylinder).
type Cylinder struct {
	p1, p2 Vec3
	radius float64
	color  color.RGBA
}

// reflect returns the reflection of I about the normal N.
func reflect(I, N Vec3) Vec3 {
	return I.Sub(N.Mul(2 * I.Dot(N)))
}

// intersectSphere returns the distance t along the ray (origin, dir)
// where it first hits the sphere (if any). Assumes dir is normalized.
func intersectSphere(origin, dir Vec3, sphere Sphere) (float64, bool) {
	oc := origin.Sub(sphere.center)
	// Since dir is normalized, a = 1.
	b := 2 * oc.Dot(dir)
	c := oc.Dot(oc) - sphere.radius*sphere.radius
	discriminant := b*b - 4*c
	if discriminant < 0 {
		return 0, false
	}
	sqrtDisc := math.Sqrt(discriminant)
	t1 := (-b - sqrtDisc) / 2
	t2 := (-b + sqrtDisc) / 2
	epsilon := 1e-4
	if t1 > epsilon {
		return t1, true
	}
	if t2 > epsilon {
		return t2, true
	}
	return 0, false
}

// intersectCylinder computes the intersection of a ray with a finite cylinder (including caps).
// It returns the smallest positive t (if any) and the surface normal at the hit.
func intersectCylinder(origin, dir Vec3, cyl Cylinder) (float64, bool, Vec3) {
	v := cyl.p2.Sub(cyl.p1)
	L := v.Length()
	if L == 0 {
		return 0, false, Vec3{}
	}
	vNorm := v.Mul(1 / L)
	dp := origin.Sub(cyl.p1)

	// Lateral surface intersection.
	dDotV := dir.Dot(vNorm)
	dPerp := dir.Sub(vNorm.Mul(dDotV))
	dpDotV := dp.Dot(vNorm)
	dpPerp := dp.Sub(vNorm.Mul(dpDotV))
	A := dPerp.Dot(dPerp)
	B := 2 * dPerp.Dot(dpPerp)
	C := dpPerp.Dot(dpPerp) - cyl.radius*cyl.radius

	epsilon := 1e-4
	tMin := math.Inf(1)
	hit := false
	var hitNormal Vec3

	if math.Abs(A) > epsilon {
		disc := B*B - 4*A*C
		if disc >= 0 {
			sqrtDisc := math.Sqrt(disc)
			t1 := (-B - sqrtDisc) / (2 * A)
			t2 := (-B + sqrtDisc) / (2 * A)
			for _, tCandidate := range []float64{t1, t2} {
				if tCandidate > epsilon {
					hitPoint := origin.Add(dir.Mul(tCandidate))
					proj := hitPoint.Sub(cyl.p1).Dot(vNorm)
					if proj >= 0 && proj <= L {
						if tCandidate < tMin {
							tMin = tCandidate
							hit = true
							projectionPoint := cyl.p1.Add(vNorm.Mul(proj))
							hitNormal = hitPoint.Sub(projectionPoint).Normalize()
						}
					}
				}
			}
		}
	}

	// Cap intersections.
	if math.Abs(dir.Dot(vNorm)) > epsilon {
		tCap := -(dp.Dot(vNorm)) / dir.Dot(vNorm)
		if tCap > epsilon && tCap < tMin {
			hitPoint := origin.Add(dir.Mul(tCap))
			if hitPoint.Sub(cyl.p1).Sub(vNorm.Mul(hitPoint.Sub(cyl.p1).Dot(vNorm))).Length() <= cyl.radius {
				tMin = tCap
				hit = true
				hitNormal = vNorm.Mul(-1)
			}
		}
	}
	{
		dp2 := origin.Sub(cyl.p2)
		if math.Abs(dir.Dot(vNorm)) > epsilon {
			tCap := -(dp2.Dot(vNorm)) / dir.Dot(vNorm)
			if tCap > epsilon && tCap < tMin {
				hitPoint := origin.Add(dir.Mul(tCap))
				if hitPoint.Sub(cyl.p2).Sub(vNorm.Mul(hitPoint.Sub(cyl.p2).Dot(vNorm))).Length() <= cyl.radius {
					tMin = tCap
					hit = true
					hitNormal = vNorm
				}
			}
		}
	}
	return tMin, hit, hitNormal
}

// rotateY rotates vector v about the Y–axis by angle theta (in radians).
func rotateY(v Vec3, theta float64) Vec3 {
	cosT := math.Cos(theta)
	sinT := math.Sin(theta)
	return Vec3{
		X: cosT*v.X + sinT*v.Z,
		Y: v.Y,
		Z: -sinT*v.X + cosT*v.Z,
	}
}

// rotateX rotates vector v about the X–axis by angle theta (in radians).
func rotateX(v Vec3, theta float64) Vec3 {
	cosT := math.Cos(theta)
	sinT := math.Sin(theta)
	return Vec3{
		X: v.X,
		Y: cosT*v.Y - sinT*v.Z,
		Z: sinT*v.Y + cosT*v.Z,
	}
}

// precomputeRays computes the normalized ray directions for every pixel.
func precomputeRays(width, height int, fov, aspect float64) []Vec3 {
	halfHeight := math.Tan(fov / 2)
	halfWidth := aspect * halfHeight
	rays := make([]Vec3, width*height)
	for j := 0; j < height; j++ {
		for i := 0; i < width; i++ {
			u := (float64(i) + 0.5) / float64(width)
			v := (float64(j) + 0.5) / float64(height)
			x := (2*u - 1) * halfWidth
			y := (1 - 2*v) * halfHeight
			rays[j*width+i] = Vec3{x, y, 1}.Normalize()
		}
	}
	return rays
}

// computeBoundingSphere computes a sphere that bounds all frame objects.
func computeBoundingSphere(spheres []Sphere, bonds []Cylinder) (Vec3, float64) {
	minPt := Vec3{math.Inf(1), math.Inf(1), math.Inf(1)}
	maxPt := Vec3{math.Inf(-1), math.Inf(-1), math.Inf(-1)}

	// Include spheres.
	for _, s := range spheres {
		for _, offset := range []Vec3{{s.radius, s.radius, s.radius}, {-s.radius, -s.radius, -s.radius}} {
			pt := s.center.Add(offset)
			if pt.X < minPt.X {
				minPt.X = pt.X
			}
			if pt.Y < minPt.Y {
				minPt.Y = pt.Y
			}
			if pt.Z < minPt.Z {
				minPt.Z = pt.Z
			}
			if pt.X > maxPt.X {
				maxPt.X = pt.X
			}
			if pt.Y > maxPt.Y {
				maxPt.Y = pt.Y
			}
			if pt.Z > maxPt.Z {
				maxPt.Z = pt.Z
			}
		}
	}
	// Include bonds.
	for _, b := range bonds {
		for _, pt := range []Vec3{b.p1, b.p2} {
			for _, offset := range []Vec3{{b.radius, b.radius, b.radius}, {-b.radius, -b.radius, -b.radius}} {
				candidate := pt.Add(offset)
				if candidate.X < minPt.X {
					minPt.X = candidate.X
				}
				if candidate.Y < minPt.Y {
					minPt.Y = candidate.Y
				}
				if candidate.Z < minPt.Z {
					minPt.Z = candidate.Z
				}
				if candidate.X > maxPt.X {
					maxPt.X = candidate.X
				}
				if candidate.Y > maxPt.Y {
					maxPt.Y = candidate.Y
				}
				if candidate.Z > maxPt.Z {
					maxPt.Z = candidate.Z
				}
			}
		}
	}
	center := Vec3{
		X: (minPt.X + maxPt.X) / 2,
		Y: (minPt.Y + maxPt.Y) / 2,
		Z: (minPt.Z + maxPt.Z) / 2,
	}
	radius := maxPt.Sub(center).Length()
	return center, radius
}

// rayColor casts a ray from the camera origin through the scene and returns the shaded color.
func rayColor(origin, dir Vec3, spheres []Sphere, cylinders []Cylinder, lightDir Vec3,
	ambient, diffuseCoeff, specularCoeff, shininess float64, bsCenter Vec3, bsRadius float64) color.Color {

	// Early exit if the ray misses the scene’s bounding sphere.
	if t, ok := intersectSphere(origin, dir, Sphere{center: bsCenter, radius: bsRadius}); !ok || t < 0 {
		return color.RGBA{255, 255, 255, 255}
	}

	closestT := math.Inf(1)
	hitFound := false
	var hitColor color.RGBA
	var hitNormal Vec3

	// Test sphere intersections.
	for _, sphere := range spheres {
		if t, ok := intersectSphere(origin, dir, sphere); ok && t < closestT {
			closestT = t
			hitFound = true
			hitPoint := origin.Add(dir.Mul(t))
			hitNormal = hitPoint.Sub(sphere.center).Normalize()
			hitColor = sphere.color
		}
	}
	// Test cylinder intersections.
	for _, cyl := range cylinders {
		if t, ok, normal := intersectCylinder(origin, dir, cyl); ok && t < closestT {
			closestT = t
			hitFound = true
			hitNormal = normal
			hitColor = cyl.color
		}
	}

	if !hitFound {
		return color.RGBA{255, 255, 255, 255} // Background color.
	}

	hitPoint := origin.Add(dir.Mul(closestT))
	shadowOrigin := hitPoint.Add(hitNormal.Mul(1e-4))
	inShadow := false
	for _, sphere := range spheres {
		if t, ok := intersectSphere(shadowOrigin, lightDir, sphere); ok && t > 1e-4 {
			inShadow = true
			break
		}
	}
	if !inShadow {
		for _, cyl := range cylinders {
			if t, ok, _ := intersectCylinder(shadowOrigin, lightDir, cyl); ok && t > 1e-4 {
				inShadow = true
				break
			}
		}
	}

	viewDir := origin.Sub(hitPoint).Normalize()
	var diffuse, specular float64
	if !inShadow {
		diff := math.Max(hitNormal.Dot(lightDir), 0)
		diffuse = diffuseCoeff * diff
		reflectDir := reflect(lightDir.Mul(-1), hitNormal)
		spec := math.Pow(math.Max(viewDir.Dot(reflectDir), 0), shininess)
		specular = specularCoeff * spec
	}
	intensity := ambient + diffuse
	r := float64(hitColor.R)*intensity + 255*specular
	g := float64(hitColor.G)*intensity + 255*specular
	b := float64(hitColor.B)*intensity + 255*specular
	if r > 255 {
		r = 255
	}
	if g > 255 {
		g = 255
	}
	if b > 255 {
		b = 255
	}
	return color.RGBA{uint8(r), uint8(g), uint8(b), 255}
}

// renderFrame renders one image via ray–tracing the scene.
func renderFrame(spheres []Sphere, cylinders []Cylinder, width, height int, fov, aspect float64, lightDir Vec3,
	ambient, diffuseCoeff, specularCoeff, shininess float64, camRays []Vec3) *image.RGBA {

	img := image.NewRGBA(image.Rect(0, 0, width, height))
	cameraOrigin := Vec3{0, 0, 0}
	bsCenter, bsRadius := computeBoundingSphere(spheres, cylinders)

	var wg sync.WaitGroup
	for j := 0; j < height; j++ {
		wg.Add(1)
		go func(j int) {
			defer wg.Done()
			for i := 0; i < width; i++ {
				idx := j*width + i
				rayDir := camRays[idx]
				col := rayColor(cameraOrigin, rayDir, spheres, cylinders, lightDir,
					ambient, diffuseCoeff, specularCoeff, shininess, bsCenter, bsRadius)
				img.Set(i, j, col)
			}
		}(j)
	}
	wg.Wait()
	return img
}

// imageToPaletted converts an RGBA image into a paletted image using the provided palette.
func imageToPaletted(img *image.RGBA, pal color.Palette) *image.Paletted {
	bounds := img.Bounds()
	palImg := image.NewPaletted(bounds, pal)
	for y := bounds.Min.Y; y < bounds.Max.Y; y++ {
		for x := bounds.Min.X; x < bounds.Max.X; x++ {
			palImg.Set(x, y, img.At(x, y))
		}
	}
	return palImg
}

// parseSDF reads a molecule from an SDF file in standard V2000 format.
func parseSDF(filename string) ([]Sphere, []Cylinder, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, nil, err
	}
	defer file.Close()

	var lines []string
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		lines = append(lines, scanner.Text())
	}
	if err := scanner.Err(); err != nil {
		return nil, nil, err
	}
	if len(lines) < 4 {
		return nil, nil, fmt.Errorf("file too short to be a valid SDF")
	}

	// Counts line is line 4 (index 3).
	countsLine := lines[3]
	fields := strings.Fields(countsLine)
	if len(fields) < 2 {
		return nil, nil, fmt.Errorf("invalid counts line in SDF")
	}
	nAtoms, err := strconv.Atoi(fields[0])
	if err != nil {
		return nil, nil, err
	}
	nBonds, err := strconv.Atoi(fields[1])
	if err != nil {
		return nil, nil, err
	}

	// Read atom block.
	type AtomRec struct {
		element string
		pos     Vec3
	}
	var atoms []AtomRec
	atomStart := 4
	atomEnd := atomStart + nAtoms
	if len(lines) < atomEnd {
		return nil, nil, fmt.Errorf("not enough atom lines")
	}

	var minX, minY, minZ, maxX, maxY, maxZ float64
	first := true
	for i := atomStart; i < atomEnd; i++ {
		line := lines[i]
		fields := strings.Fields(line)
		if len(fields) < 4 {
			continue
		}
		x, err1 := strconv.ParseFloat(fields[0], 64)
		y, err2 := strconv.ParseFloat(fields[1], 64)
		z, err3 := strconv.ParseFloat(fields[2], 64)
		if err1 != nil || err2 != nil || err3 != nil {
			continue
		}
		element := fields[3]
		atoms = append(atoms, AtomRec{element, Vec3{x, y, z}})
		if first {
			minX, maxX = x, x
			minY, maxY = y, y
			minZ, maxZ = z, z
			first = false
		} else {
			if x < minX {
				minX = x
			}
			if x > maxX {
				maxX = x
			}
			if y < minY {
				minY = y
			}
			if y > maxY {
				maxY = y
			}
			if z < minZ {
				minZ = z
			}
			if z > maxZ {
				maxZ = z
			}
		}
	}

	// Read bond block.
	type BondRec struct {
		a, b  int // 0-indexed atom indices.
		order int
	}
	var bondsRec []BondRec
	bondStart := atomEnd
	bondEnd := bondStart + nBonds
	if len(lines) < bondEnd {
		return nil, nil, fmt.Errorf("not enough bond lines")
	}
	for i := bondStart; i < bondEnd; i++ {
		line := lines[i]
		fields := strings.Fields(line)
		if len(fields) < 3 {
			continue
		}
		a, err1 := strconv.Atoi(fields[0])
		b, err2 := strconv.Atoi(fields[1])
		order, err3 := strconv.Atoi(fields[2])
		if err1 != nil || err2 != nil || err3 != nil {
			continue
		}
		bondsRec = append(bondsRec, BondRec{a - 1, b - 1, order})
	}

	// Center and scale molecule.
	center := Vec3{(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2}
	maxDist := 0.0
	for _, a := range atoms {
		d := a.pos.Sub(center).Length()
		if d > maxDist {
			maxDist = d
		}
	}
	scale := 1.0
	if maxDist > 0 {
		scale = 1.0 / maxDist
	}

	const atomScaleFactor = 0.6

	// Covalent radii and element colors.
	radiiMap := map[string]float64{
		"H":  0.31,
		"C":  0.76,
		"N":  0.71,
		"O":  0.66,
		"F":  0.57,
		"P":  1.07,
		"S":  1.05,
		"Cl": 1.02,
	}
	colorMap := map[string]color.RGBA{
		"H":  {255, 255, 255, 255},
		"C":  {50, 50, 50, 255},
		"N":  {50, 50, 255, 255},
		"O":  {255, 50, 50, 255},
		"F":  {0, 255, 0, 255},
		"P":  {255, 165, 0, 255},
		"S":  {255, 255, 0, 255},
		"Cl": {0, 255, 0, 255},
	}

	var spheres []Sphere
	for _, a := range atoms {
		pos := a.pos.Sub(center).Mul(scale)
		rad, ok := radiiMap[a.element]
		if !ok {
			rad = 0.70
		}
		rad = rad * scale * atomScaleFactor
		col, ok := colorMap[a.element]
		if !ok {
			col = color.RGBA{200, 100, 200, 255}
		}
		spheres = append(spheres, Sphere{pos, rad, col})
	}

	// Create bonds as cylinders.
	bonds := []Cylinder{}
	bondColor := color.RGBA{150, 150, 150, 255}
	for _, bRec := range bondsRec {
		if bRec.a < 0 || bRec.a >= len(spheres) || bRec.b < 0 || bRec.b >= len(spheres) {
			continue
		}
		p1 := spheres[bRec.a].center
		p2 := spheres[bRec.b].center
		d := p2.Sub(p1)
		if d.Length() == 0 {
			continue
		}
		dNorm := d.Normalize()
		// Choose an arbitrary reference vector.
		ref := Vec3{0, 0, 1}
		if math.Abs(dNorm.Dot(ref)) > 0.9 {
			ref = Vec3{0, 1, 0}
		}
		offsetVec := dNorm.Cross(ref).Normalize()
		r1 := spheres[bRec.a].radius
		r2 := spheres[bRec.b].radius
		bondRadius := 0.225 * math.Min(r1, r2)

		switch bRec.order {
		case 1:
			baseP1 := p1.Add(dNorm.Mul(r1))
			baseP2 := p2.Sub(dNorm.Mul(r2))
			bonds = append(bonds, Cylinder{baseP1, baseP2, bondRadius, bondColor})
		case 2:
			const bondFactor = 0.8
			offMag := 0.3 * math.Min(r1, r2)
			for _, sign := range []float64{1, -1} {
				offset := offsetVec.Mul(sign * offMag)
				t1 := bondFactor * math.Sqrt(math.Max(0, r1*r1-offset.Dot(offset)))
				t2 := bondFactor * math.Sqrt(math.Max(0, r2*r2-offset.Dot(offset)))
				p1off := p1.Add(offset).Add(dNorm.Mul(t1))
				p2off := p2.Add(offset).Sub(dNorm.Mul(t2))
				bonds = append(bonds, Cylinder{p1off, p2off, bondRadius, bondColor})
			}
		case 3:
			const bondFactor = 0.8
			baseP1 := p1.Add(dNorm.Mul(r1 * bondFactor))
			baseP2 := p2.Sub(dNorm.Mul(r2 * bondFactor))
			bonds = append(bonds, Cylinder{baseP1, baseP2, bondRadius, bondColor})
			offMag := 0.3 * math.Min(r1, r2)
			for _, sign := range []float64{1, -1} {
				offset := offsetVec.Mul(sign * offMag)
				t1 := bondFactor * math.Sqrt(math.Max(0, r1*r1-offset.Dot(offset)))
				t2 := bondFactor * math.Sqrt(math.Max(0, r2*r2-offset.Dot(offset)))
				p1off := p1.Add(offset).Add(dNorm.Mul(t1))
				p2off := p2.Add(offset).Sub(dNorm.Mul(t2))
				bonds = append(bonds, Cylinder{p1off, p2off, bondRadius, bondColor})
			}
		default:
			baseP1 := p1.Add(dNorm.Mul(r1))
			baseP2 := p2.Sub(dNorm.Mul(r2))
			bonds = append(bonds, Cylinder{baseP1, baseP2, bondRadius, bondColor})
		}
	}

	return spheres, bonds, nil
}

// Config holds the rendering and animation settings.
type Config struct {
	InputFile   string
	OutputFile  string
	Width       int
	Height      int
	FOV         float64
	TotalFrames int
	Tilt        float64
	ZOffset     float64
	Ambient     float64
	Diffuse     float64
	Specular    float64
	Shininess   float64
}

// parseFlags processes command–line arguments and returns a Config.
func parseFlags() Config {
	cfg := Config{}
	flag.StringVar(&cfg.InputFile, "in", "", "Path to the input SDF file (required)")
	flag.StringVar(&cfg.OutputFile, "out", "output.gif", "Path to the output GIF file")
	flag.IntVar(&cfg.Width, "width", 1920, "Image width in pixels")
	flag.IntVar(&cfg.Height, "height", 1080, "Image height in pixels")
	flag.Float64Var(&cfg.FOV, "fov", math.Pi/3, "Vertical field of view (radians)")
	flag.IntVar(&cfg.TotalFrames, "frames", 120, "Total number of frames in the animation")
	flag.Float64Var(&cfg.Tilt, "tilt", 0.0, "Tilt angle (radians) applied about the X–axis")
	flag.Float64Var(&cfg.ZOffset, "zoffset", 2.25, "Camera translation along Z axis")
	flag.Float64Var(&cfg.Ambient, "ambient", 0.7, "Ambient light coefficient")
	flag.Float64Var(&cfg.Diffuse, "diffuse", 1.0, "Diffuse light coefficient")
	flag.Float64Var(&cfg.Specular, "specular", 0.7, "Specular light coefficient")
	flag.Float64Var(&cfg.Shininess, "shininess", 20.0, "Shininess coefficient for specular highlights")
	flag.Parse()

	if cfg.InputFile == "" {
		flag.Usage()
		log.Fatal("Error: input SDF file must be provided using -in flag.")
	}
	return cfg
}

// run executes the main program logic.
func run(cfg Config) error {
	// Check input file exists.
	if _, err := os.Stat(cfg.InputFile); os.IsNotExist(err) {
		return fmt.Errorf("input file %q does not exist", cfg.InputFile)
	}

	spheres, bonds, err := parseSDF(cfg.InputFile)
	if err != nil {
		return fmt.Errorf("error parsing SDF: %w", err)
	}

	aspect := float64(cfg.Width) / float64(cfg.Height)
	lightDir := Vec3{-1, 1, 1}.Normalize()
	camRays := precomputeRays(cfg.Width, cfg.Height, cfg.FOV, aspect)

	images := []*image.Paletted{}
	delays := []int{}

	// Render each frame.
	for frame := 0; frame < cfg.TotalFrames; frame++ {
		theta := float64(frame) * 2 * math.Pi / float64(cfg.TotalFrames)
		// Rotate and translate spheres.
		frameSpheres := make([]Sphere, len(spheres))
		for i, s := range spheres {
			rotated := rotateY(s.center, theta)
			rotated = rotateX(rotated, cfg.Tilt)
			rotated = rotated.Add(Vec3{0, 0, cfg.ZOffset})
			frameSpheres[i] = Sphere{rotated, s.radius, s.color}
		}
		// Rotate and translate bonds.
		frameBonds := make([]Cylinder, len(bonds))
		for i, b := range bonds {
			rotP1 := rotateY(b.p1, theta)
			rotP1 = rotateX(rotP1, cfg.Tilt)
			rotP1 = rotP1.Add(Vec3{0, 0, cfg.ZOffset})
			rotP2 := rotateY(b.p2, theta)
			rotP2 = rotateX(rotP2, cfg.Tilt)
			rotP2 = rotP2.Add(Vec3{0, 0, cfg.ZOffset})
			frameBonds[i] = Cylinder{rotP1, rotP2, b.radius, b.color}
		}
		// Render the frame.
		img := renderFrame(frameSpheres, frameBonds, cfg.Width, cfg.Height, cfg.FOV, aspect, lightDir,
			cfg.Ambient, cfg.Diffuse, cfg.Specular, cfg.Shininess, camRays)
		palImg := imageToPaletted(img, palette.Plan9)
		images = append(images, palImg)
		delays = append(delays, 5) // Delay in 10ms units.
		log.Printf("Rendered frame %d/%d", frame+1, cfg.TotalFrames)
	}

	// Assemble and write the animated GIF.
	outGif := &gif.GIF{
		Image:     images,
		Delay:     delays,
		LoopCount: 0, // Infinite loop.
	}
	outFile, err := os.Create(cfg.OutputFile)
	if err != nil {
		return fmt.Errorf("error creating output file: %w", err)
	}
	defer outFile.Close()

	if err := gif.EncodeAll(outFile, outGif); err != nil {
		return fmt.Errorf("error encoding GIF: %w", err)
	}
	log.Printf("Successfully wrote %q", cfg.OutputFile)
	return nil
}

func main() {
	cfg := parseFlags()
	if err := run(cfg); err != nil {
		log.Fatalf("Error: %v", err)
	}
}
