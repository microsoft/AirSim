# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Blocks is an Unreal Engine 4.27 environment for AirSim that specializes in electromagnetic jamming simulation. It provides a platform for testing autonomous systems (particularly drones) under electromagnetic interference scenarios with both visual simulation and HTTP API control capabilities.

## Build System

This is an Unreal Engine project. Build through the Unreal Editor or Unreal Build Tool:

- **Engine Version**: 4.27
- **Module**: `Blocks` (Runtime module)
- **Build Configuration**: `Source/Blocks/Blocks.Build.cs`

Dependencies (defined in Blocks.Build.cs):
- Core, CoreUObject, Engine, InputCore
- HTTP, HttpServer (for API endpoints)
- Json, JsonUtilities (for API responses)

Platform notes:
- Windows: Exceptions enabled
- Linux: Exceptions disabled

## Architecture

### Jamming System

The core jamming functionality follows a component-based architecture:

**Base Class**: `AJammerActor` (Source/Blocks/JammerActor.h)
- Implements electromagnetic interference using 1/r² power decay model
- Uses `USphereComponent` for range detection (radius in cm)
- Visual feedback via `UStaticMeshComponent* RangeVisualizer` sphere mesh
- Configurable via UProperties:
  - `JammerPower`: Base power (default 1.0f)
  - `bIsJamming`: Enable/disable jamming

**Derived Classes**:
- `AAntennaJammer`: Specialized for antenna jamming scenarios
- `ADroneJammer`: Specialized for drone interference

Key Blueprint-callable methods:
- `GetJammerPower()`: Returns current jamming power (0 if disabled)
- `GetJammerPowerAtLocation(WorldLocation)`: Computes interference power at a 3D position using distance decay
- `GetRadiusCm()`: Gets the effective jamming range

### HTTP API Service

`AJammerHttpService` (Source/Blocks/JammerHttpService.h) provides RESTful endpoints for external control and monitoring:

- Default port: 18080 (configurable)
- Listener address: 0.0.0.0 (configured in Config/DefaultEngine.ini)

Endpoints:
- `GET /ping` - Health check
- `GET /jammers` - Lists all jammer actors in the world
- `GET /jammer_power` - Query jammer power at specific location

The service uses Unreal's HttpServer module with route-based handlers and JSON serialization.

### AirSim Integration

The project integrates with AirSim via plugin dependency (configured in Blocks.uproject):

- Game mode redirections in Config/DefaultEngine.ini:
  - `TP_FlyingGameMode` → `BlocksGameMode`
  - `TP_FlyingPawn` → `BlocksPawn`
- Default map: `/Game/Maps/Airsim.Airsim`

### Content Structure

The Content directory contains multiple simulation environments:

- **Maps/**: Main maps including `Airsim.umap` (default) and `Simple.umap`
- **FlyingCPP/**: C++ flying example map
- **3D_Radar/**: Radar simulation with blueprints
- **MilitaryDrone/**: Military drone assets
- **LucidBubble/**: Additional environment variant
- **VigilanteContent/**: Naval/ship simulation
- **Models/**: 3D model assets

### Configuration

**Config/DefaultEngine.ini**:
- Editor startup and default map settings
- Game mode: `/Script/Blocks.BlocksGameMode`
- HTTP server listener: `DefaultBindAddress=0.0.0.0`
- Physics: Gravity -980.0 cm/s²

**Config/DefaultGame.ini**:
- Project metadata (Microsoft Research, MIT License)
- Packaging: Development configuration
- Maps to cook: FlyingExampleMap and AirSimAssets

## Development Notes

- The jammer power calculation uses distance squared decay: `Power / (distance_meters * distance_meters)`
- All coordinates in Unreal are centimeters (power calculation converts to meters)
- The HTTP service iterates through world actors to find jammers - performance considerations apply for large maps
- Range visualizer automatically syncs to the sphere component radius in `OnConstruction()`
