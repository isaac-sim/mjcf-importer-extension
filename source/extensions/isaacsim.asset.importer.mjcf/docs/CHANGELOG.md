# Changelog
## [2.3.6] - 2025-02-19
### Fixed
- Relative path for sublayers

## [2.3.5] - 2025-02-13
### Added
- Support for equality constraints
### Fixed
- Fixed bug where multiple tendons to the same joint pair were overriding each other.
- Fixed appropriate conversion of tendon properties to USD.

## [2.3.4] - 2025-01-28
### Fixed
- Windows signing issue

## [2.3.3] - 2025-01-01
### Changed
- Colliders create a MeshMergeCollision at the collisions prim level on robot assembly.

## [2.3.2] - 2024-12-13
### Fixed
- In-place ensuring that meshes, visuals and colliders scopes are invisible.
- Add collision group for colliders out of default prim

## [2.3.1] - 2024-12-09
### Fixed
- Crash when selecting the default density.

## [2.3.0] - 2024-12-09
### Changed
- Ensured Colliders and Visuals do a second stack of referencing so the visual and collider can use the same mesh.
### Fixed
- Collider instancing was picking wrong element when imageable was not a mesh.

## [2.2.3] - 2024-11-22
### Changed
- Only create instanceable prims.
- Use Omniverse ASset Converter.
- Change auto-limit default for joints to true
### FIXED
- Ensure tags are imported even when there are multiple elements.
- Allow import to continue even when inertia is not defined. (issues a warning)

## [2.2.2] - 2024-11-20
### Changed
- Update omni.client import

## [2.2.1] - 2024-10-25
### Changed
- Remove test imports from runtime

## [2.2.0] - 2024-10-13
### Changed
- Update Assets transforms into Translate/orient/scale elements

## [2.1.0] - 2024-10-13
### Changed
- Change the import options
### Removed
- standalone window import

## [2.0.0] - 2024-09-20
### Changed
- Changed MJCF importer to export USD to base, physics, sensor, and main stage
- Changed joint importer to import joints to the joint scope, and categorized by joint type

## [1.2.2] - 2024-09-17
### Added
- Action registry for menu item

## [1.2.1] - 2024-09-06
### Changed
- Register the mjcf file icon to asset types.

## [1.2.0] - 2024-08-22
### Changed
- ISIM-1670: Move Import workflow to File->Import.
- ISIM-1673: Display mjcf options in the right panel of the Import Dialog Window.

## [1.1.1] - 2024-07-10
### Changed
- Importer frames on imported asset when done through GUI.

## [1.1.0] - 2023-10-03
### Changed
- Structural and packaging changes

## [1.0.1] - 2023-07-07
### Added
- Support for `autolimits` compiler setting for joints

## [1.0.0] - 2023-06-13
### Changed
- Renamed the extension to omni.importer.mjcf
- Published the extension to the default registry

## [0.5.0] - 2023-05-09
### Added
- Support for ball and free joint
- Support for `<freejoint>` tag
- Support for plane geom type
- Support for intrinsic Euler sequences

### Changed
- Default value for fix_base is now false
- Root bodies no longer have their translation automatically set to the origin
- Visualize collision geom option now sets collision geom's visibility to invisible
- Change prim hierarchy to support multiple world body level prims

### Fixed
- Fix support for full inertia matrix
- Fix collision geom for ellipsoid prim
- Fix zaxis orientation parsing
- Fix 2D texture by enabling UVW projection


## [0.4.1] - 2023-05-02
### Added
- High level code overview in README.md

## [0.4.0] - 2023-03-27
### Added
- Support for sites and spatial tendons
- Support for specifying mesh root directory

## [0.3.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.3.0] - 2022-10-13
### Added
- Added material and texture support

## [0.2.3] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.2.2] - 2022-07-21

### Added
- Add armature to joints

## [0.2.1] - 2022-07-21

### Fixed
- Display Bookmarks when selecting files

## [0.2.0] - 2022-06-30

### Added
- Add instanceable option to importer

## [0.1.3] - 2022-05-17

### Added
- Add joint values API

## [0.1.2] - 2022-05-10

### Changed
- Collision filtering now uses filteredPairsAPI instead of collision groups
- Adding tendons no longer has limitations on the number of joints per tendon and the order of the joints

## [0.1.1] - 2022-04-14

### Added
- Joint name annotation USD attribute for preserving joint names

### Fixed
- Correctly parse distance scaling from UI

## [0.1.0] - 2022-02-07

### Added
- Initial version of MJCF importer extension
