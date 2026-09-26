# Sky

The farm's time-of-day and weather presets (Tools > Install HDRI Sky, SkyPresets on
the scene's sun). All three skies are Poly Haven pure skies, CC0 (no attribution
required), downloaded 2026-09-26 as 4k .hdr files:

| file | preset | source |
|---|---|---|
| `kloofendal_48d_partly_cloudy_farm_4k.hdr` | Midday | "Kloofendal 48d Partly Cloudy (Pure Sky)", Greg Zaal, https://polyhaven.com/a/kloofendal_48d_partly_cloudy_puresky (20.7 MB) |
| `syferfontein_18d_clear_farm_4k.hdr` | Late afternoon | "Syferfontein 18d Clear (Pure Sky)", https://polyhaven.com/a/syferfontein_18d_clear_puresky (15.9 MB) |
| `overcast_soil_farm_4k.hdr` | Overcast | "Overcast Soil (Pure Sky)", https://polyhaven.com/a/overcast_soil_puresky (16.9 MB) |

Each is prepared with `tools/hdri_prepare.py` (sources not kept in the repo):

- the sun disc is clamped to luminance 30, because the scene's directional light
  is the sun and Unity derives ambient light from the skybox; left in, the sun
  would be counted twice
- the lower hemisphere (a blurred mirror of the sky in pure-sky HDRIs) is
  replaced with the radiance of farm ground lit by that sky, so ambient light
  from below and the lower half of metal reflections are green-brown, not blue

Measured on the originals with `tools/hdri_sun.py` (sun elevation; light
intensity E_sun / pi and colour in linear units at sky exposure 1; share of
the light on flat ground that is direct sun):

| sky | sun | light | colour | direct |
|---|---|---|---|---|
| kloofendal 48d | 47.9 deg | 1.447 | 0.99 / 1.00 / 0.91 | 70% |
| syferfontein 18d | 18.4 deg | 4.920 | 1.00 / 0.87 / 0.54 | 89% |
| overcast soil | none | 0 | | 0% |

The skies are not calibrated against each other, so each preset has an
exposure (1, 0.45, 0.55); `Assets/Editor/SkySetup.cs` explains the choices and
carries these numbers.
