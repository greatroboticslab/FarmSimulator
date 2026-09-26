# Sky

`kloofendal_48d_partly_cloudy_farm_4k.hdr` is the farm's sky (Tools > Install HDRI Sky).

Source: "Kloofendal 48d Partly Cloudy (Pure Sky)" by Greg Zaal, Poly Haven,
CC0 (no attribution required).
https://polyhaven.com/a/kloofendal_48d_partly_cloudy_puresky
Downloaded 2026-09-26 as the 4k .hdr (20.7 MB):
https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/4k/kloofendal_48d_partly_cloudy_puresky_4k.hdr

Prepared with `tools/hdri_prepare.py` (source HDR not kept in the repo):

- the sun disc is clamped to luminance 30, because the scene's directional light
  is the sun and Unity derives ambient light from the skybox; left in, the sun
  would be counted twice
- the lower hemisphere (a blurred mirror of the sky in pure-sky HDRIs) is
  replaced with the radiance of farm ground lit by this sky, so ambient light
  from below and the lower half of metal reflections are green-brown, not blue

Measured on the original with `tools/hdri_sun.py`: sun elevation 47.88 deg,
irradiance facing the sun 4.51 / 4.55 / 4.15 (light intensity 1.447, colour
0.992 / 1.000 / 0.912 in linear units at sky exposure 1), 70% of the light on
flat ground is direct sun. `Assets/Editor/SkySetup.cs` carries these numbers.
