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
