# grid_map_filters_drs
Custom grid map filters for DRS use, so we can easily setup stuff using filter chains.

The package includes the filters in [`src/grid_map_filters_drs`](src/grid_map_filters_drs) and a simple node to load a filter chain [`elevation_map_filter`](src/elevation_map_filter/elevation_map_filter.cpp).

## Filters

### Base Height Threshold
Applies a `threshold` on height relative to a `target_frame`, given the TF tree, converting the values above the threshold to `set_to_upper` and the ones below to `set_to_lower`.

```yaml
- name: height_filter
    type: gridMapFiltersDrs/BaseHeightThresholdFilter
    params:
      input_layer: elevation
      output_layer: height_thresholded
      target_frame: base
      threshold: -0.3
      set_to_upper: 0.0
      set_to_lower: 1.0
```

### Change Normals Frame
Applies a transformation to a set of normal layers with a given prefix (e.g., `normal_`) and applies a rotation to match the indicated `target_frame`, asumming the normals are defined in the fixed frame of the grid map.

```yaml
- name: rotate_surface_normals
  type: gridMapFiltersDrs/ChangeNormalsFrameFilter
  params:
    input_layers_prefix: normal_
    target_frame: base_link
```
For the previous example, if the grid map is defined in `odom`, it will apply a transformation so the normals are expressed in the `base_link` frame, hence they should change when the robot rotates the base (or torso).

### Delete All But (the given layers)
Deletes all the layers **except** the ones specified in `layers`. 
```yaml
  - name: delete
    type: gridMapFiltersDrs/DeleteAllButFilter
    params:
      layers: [elevation, traversability] # List of layers.
```
It will remove all the alyers in the grid map and will just keep `elevation` and `traversability`.
### Denoise and Inpaint
It applies inpainting using OpenCV algorithms. Since the inpainting converts the grid map to an image, it also allows to apply denoising methods before inpainting to mitigate spikes or other artifacts.

```yaml
- name: denoise_inpaint
  type: gridMapFiltersDrs/DenoiseAndInpaintFilter
  params:
    input_layer: elevation
    output_layer: elevation_inpainted
    radius: 0.1 # in m
    inpainting_type: ns # 'ns' or 'telea' allowed
    pre_denoising: false # enables denoising before inpainting
    denoising_radius: 0.15 # m
    denoising_type: total_variation # 'total_variation', 'non_local', 'gaussian', and 'median' supported
```
[Supported algorithms for denoising](https://docs.opencv.org/4.2.0/df/d3d/tutorial_py_inpainting.html) are Navier-Stokes (`ns`) or Telea's method (`telea`).
Denoising supports Gaussian (`gaussian`) or median (`median`) filters, as well as Non-local means denoising (`non_local`) or total variation (`total_variation`).

### Fast Normals
In contrast to the normals computation method available in the `grid_map` package, here we adopt a simpler approach assuming that normals always point in the Z axis, and we use computer vision techniques to compute the derivatives (with Sobel operators). It also allows to do some pre filtering to tthe input layer, and post filtering to the normal layers.

```yaml
- name: surface_normals_fast
  type: gridMapFiltersDrs/FastNormalsVectorFilter
  params:
    input_layer: elevation
    output_layers_prefix: normal_
    input_smoothing_radius: 0.15 # spatial gaussian filter (in meters)
    normals_smoothing_radius: 0.1 # spatial median filter (in meters)
```

### Gaussian Process Inpaint
This filter uses a subsample of the map to apply a Gaussian Process regression, generating a super smooth surface. **It depends on the [limbo library](https://github.com/ori-drs/limbo)**.

```yaml
- name: gp_inpainting
  type: gridMapFiltersDrs/GaussianProcessInpaintFilter
  params:
    input_layer: elevation
    output_layer: elevation_gp_inpainted
    subsample_skip: 20 # skips n cells to reduce computation
```

### Geodesic Distance Field 2D (GDF)
Computes the geodesic distance using the Fast Marching Method (FMM). It requires an `input_layer` with values in the [0,1] interval. It also requires an attractor point for the FMM given by a `PoseWithCovarianceStamped` message.

It also provides the gradients in X and Y as separate layers (in the example below, it will generate the layers `geodesic_gradient_x` and `geodesic_gradient_y`).

```yaml
- name: geodesic
  type: gridMapFiltersDrs/GeodesicDistanceField2dFilter
  params:
    input_layer: traversability
    output_layer: geodesic
    normalize_gradients: true
    attractor_topic: /goal
    threshold: 0.5 # Only applicable when using binarization
    use_binarization: true
```

### Learned Motion Costs
Generates motion costs (risk, energy, time) based on the method by [Yang et al. (2021)](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/491442/yang2021icra.pdf?sequence=1). It requires the `gpu_path_optimizer` package to execute a service call given by `service_name`.

```yaml
- name: learned_risk
    type: gridMapFiltersDrs/LearnedMotionCostsFilter
    params:
      input_layer: elevation
      output_layer: risk_learned
      cost_layer: risk
      service_name: /traversability_server/compute_traversability
```

### Nan Filler Filter
It fills the NaN values in a grid map using a simple criterion (mean, max, mean, or fixed_value).

```yaml
- name: fill_nans
  type: gridMapFiltersDrs/NanFillerFilter
  params:
    input_layer: elevation
    output_layer: elevation_no_nans
    set_to: mean # min, max, mean, fixed_value
    value: 0.0 # ignored if not 'fixed_value'
```

### Nan Mask
It generates a new layer, with a binary mask indicating where the grid map has NaN values.

```yaml
- name: nan_mask
  type: gridMapFiltersDrs/NanMaskFilter
  params:
    input_layer: elevation
    output_layer: nan_mask
```

### Signed Distance Field (SDF)
Computes a 2D signed distance field from a [0,1] layer, similarly to the GDF. It also provides the X and Y gradients as separate layers (in the example below, it will generate the layers `sdf_gradient_x` and `sdf_gradient_y`).

```yaml
- name: sdf
  type: gridMapFiltersDrs/SignedDistanceField2dFilter
  params:
    input_layer: traversability
    output_layer: sdf
    normalize_gradients: true
    threshold: 0.5 # lower thresholds are more relaxed
```