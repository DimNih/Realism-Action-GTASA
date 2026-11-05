# BetaRealismAction

A native Android Mod Loader (AML) plugin for **GTA: San Andreas Android**. It adds NPC responses to nearby dead pedestrians and hard vehicle crashes, equips the player with a helmet on powered motorcycles, gives nearby pedestrians umbrellas during rain, and adds ambient beach behavior and truck air-brake sounds.

## Screenshots

| Dead-pedestrian response                                   | Motorcycle helmet                                               |
| ---------------------------------------------------------- | --------------------------------------------------------------- |
| ![A responder near a dead pedestrian](review/review1.jpeg) | ![Player wearing a helmet on a motorcycle](review/review2.jpeg) |

## Features

- Detects nearby dead pedestrians and selects a corpse for the current response.
- Assigns one nearby responder to approach the corpse and call for an ambulance.
- Gives other eligible nearby pedestrians a brief, one-time reaction.
- After a hard vehicle impact, sends up to eight nearby pedestrians running to the scene. Gang members play an angry reaction; other pedestrians show confusion and talk with the player.
- Up to four occupants of nearby stationary vehicles can exit, check the scene, and return to their original driver or passenger seat.
- Applies one of two native player helmet clothing variants on powered motorcycles and restores the saved clothing descriptor after dismounting.
- During rain, some nearby on-foot NPCs hold a cane-model umbrella and play the `WUZI/payung` animation. When the rain stops or they leave the area, their prior weapon and normal AI task are restored.
- In selected Los Santos beach zones, occasionally gives nearby NPCs a short swim, sunbathe, chat idle, or stroll task. Most beach pedestrians keep their original game AI; one mod-managed NPC is assigned per scan.
- In supported trucks, plays a brake squeal when braking above 8 km/h, followed by a compressed-air release about 150 ms after the squeal ends.
- When the player is hit by a bullet, a round blood stain appears on the body at the impact point, facing the side that was hit (front, back, left, right). Stains follow the skeleton, merge when hit repeatedly, soak in, dry darker, and fade after about four minutes.
- Replaces the native `vehiclelights128` vehicle-light pool with dynamic lighting. Each front lamp follows its own GTA damage state, so a broken left/right lamp disables only that side; lights also follow `bLightsOn`. The GLES pass uses distance-based attenuation, brighter ground response, softer wall reflections, and a broad cone for upper building surfaces. Fallback native lighting is bounded to nearby vehicles and the engine's 32-light pool.
- Adds optional `dynamic_light=1` capture for non-vehicle GTA lights. `CPointLights::AddLight` and `CCoronas::RegisterCorona` feed nearby lamp posts, traffic lights, object lights, and effects into a ranked three-light world pool, with `world=0.30` controlling the supplemental strength. Vehicle lights remain handled by the per-lamp system.
- Adds realtime sun shadows for nearby buildings and map objects. Shadow direction and length follow the in-game clock, fade at dawn/dusk, weaken in rain, and are projected onto GTA ground polygons through the native shadow renderer. Configure `shadow_building`, `shadow_building_strength`, and `shadow_building_range`.

## Requirements

- GTA: San Andreas for Android, version 2.10
- Android Mod Loader (AML)
- ARM64 Android device

## Installation

1. Build the plugin or obtain the matching ARM64 `.so` release.
2. Copy `libBetaRealismAction.so` into the AML mods directory used by your GTA SA installation.
3. Configure XkiLoader to replace weapon model ID `326` (`gun_cane.dff`) with the umbrella DFF.
4. Make the `WUZI` animation block available to the game streaming system, with animation `payung` from `wuzi.ifp` (the animation is 3.7 seconds; the mod loops it while rain continues).
5. Launch the game.

### Vehicle-light implementation notes

The native light path follows the GTA SA Android 2.10 `CPointLights::AddLight` behavior: it only appends while `NumLights < 32`, rejects lights outside a camera-centered sphere of `Range + 15`, and attenuates color after 75% of that camera range. The mod caps headlights at 24 slots to reserve up to eight entries for two surface bounces on each of four nearby cars. In `VehicleLight.log`, `surfaceLights` counts bounce lights accepted by the engine, `surfaceCulled` counts AddLight camera/range rejections, and `surfacePoolFull` counts rejected attempts when all 32 slots are occupied.

These are real GTA point lights, not GTA IV/V deferred lighting or SSR. They light world/object geometry through GTA's normal dynamic-light receiver path; the stock renderer does not expose a modern per-pixel material/shader system through `CPointLights::AddLight`.

For smoother headlight coverage, `vehicle_light_gl.cpp` adds an optional GLES3 full-screen lighting pass immediately before `eglSwapBuffers`, after GTA finishes rendering the scene; it does not wait for a sentinel texture or game draw call. It copies the completed scene depth buffer from the framebuffer currently bound by the game (which may be a nonzero EGL-backed FBO), reconstructs visible positions and approximate normals per pixel, evaluates the vehicle spotlights in GLSL, and uses short screen-space depth marches to reduce light across visible occluders. Matrix capture filters out HUD/orthographic programs and keeps fresh perspective camera matrices from scene draws. This path does not draw the visible beam/pool as a mesh. It requires successful EGL/GLES hooks, depth-buffer blitting, and valid game perspective matrices. The depth-copy format follows the live framebuffer (for example, D16 or D24), and transient blit errors retry on later frames rather than permanently disabling the shader. The collision-clipped Im3D splats remain as a compatibility fallback. Screen-space shadows can still miss off-screen occluders or show artifacts at depth discontinuities, and the result requires on-device testing on the target GTA build/GPU.

### Sun shadows for everything (`screen_shadow_gl.cpp`)

Bayangan matahari real-time untuk **semua** yang terlihat kamera: bangunan, objek, pohon, tiang, kendaraan, dan ped sebagai *caster*, dan semuanya juga sebagai *receiver* (tanah, dinding, mobil, badan ped). Tidak ada lagi bayangan "tembus": bayangan dihitung **per pixel dari depth buffer asli**, jadi selalu menempel di permukaan yang tergambar.

Cara kerja (satu pass bersama dengan lampu kendaraan, satu salinan depth):

1. Depth scene direkonstruksi menjadi mesh dari grid depth (segitiga antar sampel bertetangga, diputus di tepi siluet) lalu digambar dengan matriks ortografis matahari ke dua cascade depth map (dekat ±18 m, jauh ±`sun_shadow_range` m). Karena sumbernya adalah apa yang benar-benar dirender, bentuk bayangan mengikuti model asli, termasuk animasi ped, kartu daun pohon, dan objek mod. Setiap permukaan dianggap padat sedalam `sun_shadow_thick` ke belakang agar tiang/tembok tipis tidak bocor.
2. Pass receiver full-screen membaca depth lagi, membandingkan tiap pixel dengan depth map (PCF contact-hardening: tajam di dekat objek, lembut makin jauh), menambah contact shadow screen-space 8 langkah, dan menggelapkan sisi yang membelakangi matahari. Hasilnya mengalikan warna scene dengan warna bayangan sejuk (`sun_shadow_ambient`).
3. Map di-snap ke texel dunia sehingga tepi bayangan tidak berkedip saat kamera bergerak.

Selama pass ini aktif, decal tanah lama (`complex_shadow`), registered shadow mesh, static shadow, dan blob shadow bawaan engine (kendaraan/ped/tiang) tidak dirender sehingga tidak ada double shadow. `sun_shadow_native` dipertahankan sebagai compatibility key tetapi tidak mengaktifkan native shadow saat `sun_shadow=1`. Jika shader gagal berulang, mode shader tidak mengganti dirinya dengan native shadow; ini sengaja supaya tidak terjadi shadow dobel.

Batasan yang perlu diketahui: caster hanya yang masuk di depth kamera. Objek yang sepenuhnya di luar layar atau tertutup objek lain dari sudut kamera tidak ikut melempar bayangan (mis. dinding yang tertutup pohon), dan objek lebih tipis dari ~1 sel grid di kejauhan bisa hilang (turunkan `sun_shadow_density`). Pada uji host (Mesa llvmpipe, scene sintetis vs ray-trace analitik) salah-gelap < 1% dan sisanya adalah bayangan dari caster yang tidak terlihat kamera.

Tes host (tanpa game): `g++ -std=c++17 -O2 test_screen_shadow_core.cpp -o t && ./t` (matematika cascade/snapping) dan `g++ -std=c++17 -O2 test_screen_shadow_gl.cpp screen_shadow_gl.cpp -o t -lEGL -lGLESv2 && EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 ./t out_dir quick` (menjalankan shader nyata di GLES 3.2 software dan membandingkan dengan ray-trace). Belum diuji di perangkat/GPU Android: cek `VehicleLight.log` untuk baris `[SunShadow]` dan `sunShadow=1` pada status `[GL]`. Kalau arah bayangan tidak cocok dengan matahari yang terlihat, coba `sun_source=1` (log mencetak kedua vektor matahari); `sun_shadow_debug=1..4` menampilkan visibilitas map, cascade, normal, dan contact shadow.

### Dynamic vehicle lights

Bloom jarak jauh default aktif secara halus pada `bloom=0.18`: efeknya hanya muncul saat kamera berada di depan sumber lampu, meningkat pada jarak menengah-jauh, dan memakai depth test agar tidak menembus kendaraan atau bangunan. Nilai ini dapat diatur di `VehicleLight.ini` pada rentang `0.0` sampai `0.60`.

### Shadow building terpisah

`complex_shadow_system.cpp` menghitung arah matahari dari jam game dan menyimpan quad bayangan lembut untuk building/objek melalui `CShadows::StoreShadowToBeRendered1`. Mesh shadow bawaan GTA diprioritaskan bila tersedia, lalu DFF/collision menjadi fallback. Pool building, object, dan dummy/LOD ikut dipindai; bentuk memakai panjang, lebar, dan koordinat tengah proyeksi model, bukan satu box global. Bayangan mengikuti posisi matahari, memudar menjelang malam, melemah saat hujan, dan dibatasi maksimal 18 caster serta 30 slot agar tidak flicker/double dan tetap terukur. `shadow_building=0` mematikannya. `sun_shadow=1` memilih jalur shader/depth receiver: semua permukaan GTA yang terlihat kamera—termasuk NPC, kendaraan dan CJ—bisa menerima bayangan sesuai depth tanpa menembus dinding, serta menggantikan decal kompleks agar tidak dobel. Fitur `dynamic_light` tetap terpisah di `dynamic_light_system.cpp` untuk lampu dunia GTA.

Pengaturan sun shadow ringan: `shadow_building=1`, `shadow_building_strength=1.10`, dan `shadow_building_range=82`. Untuk mode shader interaktif, aktifkan `sun_shadow=1`; mode ini lebih detail tetapi lebih berat GPU.

Pool putih bawaan GTA (proyeksi vertikal CShadows) tembus tembok. Saat pass GLSL aktif, pool stok diblokir dan shader depth-reconstruction menjadi renderer utama; jika syarat GLES3/depth/matriks tidak terpenuhi, fallback (`vehicle_light_beam_core.h`) memakai grid ray 17×13 dan Hann splats yang dipotong pada batas permukaan, plus shaft Im3D tipis. Tes host `test_light_beam_core.cpp` memeriksa batas tembok/tiang, cakupan tanah, dan keseragaman kernel fallback; tes ini tidak masuk ke shared library Android.

### Vehicle light update (v2)

- **Source vehicle never shadows itself.** The screen-space occlusion test now treats the lamp's own vehicle box as transparent (and ignores geometry hugging the lamp). This removes the faint ghost silhouette that appeared behind/around the car, and restores reflections when a lamp is pressed against a wall or building.
- **Per-lamp damage.** Each front lamp reads its own CDamageManager state (layout is validated first, so a wrong offset can never switch lights off). Left smashed -> only the right beam; right smashed -> only the left beam. Rear glow follows the rear lamp state. Intact lamps fade in smoothly; a smashed lamp goes dark immediately.
- **Distance behaviour.** Reflection gets clearly brighter as a surface gets closer to the lamp and fainter with distance (soft shoulder tone-map, no flat clipped white).
- **Whiter light.** New additive white-lift pass (`white=`): the road pool ahead of the car is whiter like `vehiclelights128`, the beam core is whiter and the edge slightly warmer, so it is no longer a flat orange-tinted tint.
- **Upper reflection.** The beam has a long upward lobe plus a road-pool bounce that climbs facades, so building walls and upper floors receive reflected light. A close-range spill keeps walls lit when the lamp is almost touching them (`bounce=`).
- **Sun-like distance bloom.** Soft core + halo + faint horizontal streak when the camera is in front of a lamp's beam. Multi-tap linear-distance depth test (hidden by buildings/cars, never by the source car). Subtle by default (`bloom=0.18`).
- Normals are snapped on flat roads/walls with a wider baseline, removing the blocky patches caused by depth quantisation.

`VehicleLight.ini` (next to `VehicleLight.log`, reloaded every few seconds): `intensity`, `range`, `bloom` (0-0.8), `white` (0-1.5), `bounce` (0-2), `shadow`, `rear`, `albedo`, `volumetric`, `debug`. Check `VehicleLight.log` for `first GLES headlight source ... selfBox`, `front lamp N smashed`, and `damage manager layout implausible` (the last means lamp-damage offsets need checking on that build).

### Truck air-brake sounds

The build includes two PCM WAV effects in `assets/audio`. The brake squeal lasts two seconds and the compressed-air hiss lasts five seconds. The hiss starts shortly after the squeal finishes during the same braking event. Copy both files to the AML Android data directory, under `AudioBetaRealismAction`:

```text
<Android data path>/AudioBetaRealismAction/air_brake_squeal.wav
<Android data path>/AudioBetaRealismAction/air_brake_hiss.wav
```

For example, if AML reports `/storage/emulated/0/Android_unprotected/data/com.rockstargames.gtasa/files` as its data path, use `/storage/emulated/0/Android_unprotected/data/com.rockstargames.gtasa/files/AudioBetaRealismAction/air_brake_squeal.wav` and `/storage/emulated/0/Android_unprotected/data/com.rockstargames.gtasa/files/AudioBetaRealismAction/air_brake_hiss.wav`. The squeal starts on brake press while moving; the hiss follows about 150 ms after the squeal and plays once per braking event. Supported models are Linerunner, Trashmaster, Mule, Packer, Flatbed, Yankee, Boxville, Benson, Tanker, Roadtrain, Cement Truck, Towtruck, Tractor, Utility Van, and DFT-30.

### Beach zones and world props

The NPC behavior is bounded to the stock Los Santos beach zones. When you are on foot and fewer than five NPCs are nearby, the mod can add up to six temporary beach visitors using stock beach-visitor models, and removes those peds when you leave the beach or move far away. Coordinates below are GTA world XY bounds, not GPS coordinates; they are taken from the common `info.zon`/SA map-zone table. Swim destinations are short offshore targets near each beach.

| Zone              | Approximate XY bounds                | Swim target XY |
| ----------------- | ------------------------------------ | -------------- |
| Santa Maria Beach | X 72.65–647.71, Y -2173.29–-1684.65  | 50, -1910      |
| Verona Beach      | X 647.71–930.22, Y -2173.29–-1804.21 | 760, -2200     |

Useful stock object IDs when placing beach activity areas in an IPL/map mod:

| Object ID       | Model                         | Suggested use                                                  |
| --------------- | ----------------------------- | -------------------------------------------------------------- |
| 1255            | `lounger`                     | NPC sunbathing / resting anchor                                |
| 1646            | `lounge_towel_up`             | towel / beach-rest area                                        |
| 1640–1643       | `beachtowel01`–`beachtowel04` | beach blanket anchors                                          |
| 642             | `kb_canopy_test`              | canopy/tent anchor; verify appearance in the target game build |
| 1639            | `od_vbnet`                    | volleyball net for an IPL or custom map                        |
| 1598            | `beachball`                   | volleyball/beach-ball prop                                     |
| 2404–2406, 2410 | `CJ_SURF_BOARD*`              | surf activity props                                            |

The stock object catalog reports the volleyball net and ball as unplaced beta props in the standard map, so they need custom IPL placements before they can form a real volleyball court. The current ambient system does not create props or simulate volleyball/bicycle riding; cyclists already generated by GTA's beach traffic retain their default AI. The implemented tasks are swimming, sunbathing animation, chat idle, and strolling. The zone reference and prop IDs are listed in [the SA map-zone table](https://github.com/kristoisberg/samp-map-zones/blob/master/map-zones.inc) and [the GTA SA beach object catalog](https://citwiki.com/CITtools/Objects/category/beach/).

### Bullet blood stains

`blood_stain_system.cpp` hooks `Fx_c::AddBlood` and snaps its collision-sphere impact point to the visible torso surface. Tier 1 builds the decal from nearby player-mesh triangles and re-skins those vertices each frame with live bone matrices. If the optional RenderWare skin API cannot be resolved or validated, Tier 2 falls back to an ellipse attached to the live pelvis/spine/torso/neck frame. Both paths render after the player ped, use the generated blood texture, and retain the bullet-hit and health/armour checks. Tier 2 uses a 100% body profile and a 3 mm normal lift; it deliberately does not emulate BulletWound's 12 mm depth bias by translating the patch toward the camera, because that can make the patch cross the body surface.

The supplied `BulletWound.zip` contains a 32-bit-only `libBulletWound.so`, configuration files, and PNG textures, but no source code. That binary cannot be loaded into this 64-bit ARM build. However, the supplied decompilation shows that BulletWound looked up the skin functions by their plain export names and also resolved `RpHAnimHierarchyGetMatrixArray`. `blood_stain_system.cpp` now tries those exact export names before its mangled-name fallbacks. Verify on device whether Tier 1 becomes available; if not, the library may omit those plugin exports in this game build.

The implementation has not yet been verified in GTA SA Android 2.10 on an ARM64 device. The RenderWare skin symbol names and `RpHAnimHierarchy` layout therefore remain runtime assumptions; do not treat PC math tests as a substitute for an NDK build or in-game validation. `test_decal_core.cpp` is a standalone host-side math harness and is intentionally not part of the Android shared-library build.

Blood-stain diagnostics are written to `<AML Android data path>/BetaRealismAction_BloodStain.log` and Android logcat under the `BetaBloodStain` tag. Check `Symbol ... resolved/NOT found` for optional skin API availability, `Tier1 validation: OK/FAILED` for selected tier validation, `Stain created [Tier1 mesh]` for `hitToSkin`, and `Tier2 calib` for fallback profile calibration. The log also records hook, damage, rejected-hit, texture, and render diagnostics.

The umbrella feature requires the umbrella DFF replacement and the `WUZI/payung` IFP animation to be installed and streamable by the game. If either asset is missing, NPC umbrella behavior waits for it and logs the asset state.

### Update: Complex Shadow + anti-kedip + lebih ringan

- **Complex Shadow (`complex_shadow_system.cpp`).** Bayangan building dan objek peta kini berbentuk dari **DFF modelnya sendiri**: setiap triangle `RpGeometry` dari atomic yang sedang dirender (dengan LTM frame-nya) diproyeksikan ke tanah searah matahari, lalu dijadikan mask 80x80 untuk quad `CShadows` bawaan game. Hasilnya mengikuti siluet asli model (bangunan L, tiang, papan, kartu daun pohon). Model yang geometrinya tidak bisa dibaca CPU otomatis memakai collision model sebagai cadangan. `complex_shadow_dff=0` memaksa collision saja.
- **Semua building/objek/dummy LOD yang dekat** di sekitar kamera mendapat bayangan real-time (bangunan >= 2.5 m, objek >= 0.8 m, dummy >= 1.0 m). Kendaraan dan ped default mati karena engine sudah punya shadow sendiri; nyalakan dengan `shadow_vehicles=1` / `shadow_peds=1`, atau gunakan `sun_shadow=1` untuk receiver shader yang mencakup kendaraan/ped/CJ.
- **Lebih ringan.** Maks 18 caster kompleks, scan pool tiap 20 frame, mask hanya dibuat ulang bila matahari bergeser >= 2.5 derajat atau objek berputar >= 4 derajat, maksimal 1 rebuild per frame, tekstur 80x80 di-cache per caster. Mode `sun_shadow=1` memakai shader receiver yang lebih detail tetapi lebih berat GPU.
- **Anti-kedip bayangan.** Caster dilacak antar-frame (pilihan lengket, fade in/out), bayangan disimpan ulang kalau list engine dikosongkan di antara dua `RenderStoredShadows`.
- **Anti-kedip lampu dinamis.** Lampu dunia dilacak dengan brightness di-smooth, ditahan 2 frame, 4 slot dipilih dengan histeresis + cross-fade. Lampu kendaraan di-cache per mobil (hold 3 frame, fade 10 frame) dan dipilih berdasarkan jarak dengan histeresis. Sisi GL menahan snapshot terakhir 4 frame sambil memudar bila satu frame tiba-tiba kosong.
- Orientasi tekstur bayangan di engine belum bisa diverifikasi tanpa game; kalau bayangan terlihat terbalik/lepas dari objeknya, ubah `shadow_orient` (0..7).
- Tes host untuk inti rumus: `g++ -std=c++17 -O2 test_complex_shadow_core.cpp -o test_complex_shadow && ./test_complex_shadow`.
- Referensi: gta-reversed (`CRealTimeShadowManager`/`CShadowCamera`, pool shadow real-time engine yang terbatas dan hanya untuk objek fisik), dekompilasi BFBB `xShadow.cpp` (teknik render model ke raster lalu invert), dan gist matriks shadow-view matahari native-m.
