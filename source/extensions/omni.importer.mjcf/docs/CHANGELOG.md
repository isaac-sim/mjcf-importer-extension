# Changelog

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
