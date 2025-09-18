The goal of this doc is to let you quickly grasp the overall file structure and logic flow of the Silk physics library.

## File Structure

```
├── assets                      // markdown assets
├── demo                        // GUI demo
├── docs                        // docs
├── extern                      // git submodules of external libraries
├── misc
│   ├── ccd-sample-queries      // test data
│   ├── hacks                   // patches for external libraries
│   ├── llm_guidelines          // llm prompt boilerplate
│   ├── papers                  // unused
│   ├── physics_scene           // test data
│   └── scripts                 // python debug scripts
├── model                       // sample 3d model files
├── silk                        // core physics library
│   ├── include                 // silk public API
│   └── src                     // silk source code
└── test                        // testing code
```


## Silk library

This is the core physics library.

### ECS System

Silk uses the Entity Component System (ECS). An Entity is a type erased object with many components.

For example, a cloth entity might have a triangle mesh component for its geometry and a cloth config component for its physical parameters.

The typical flow of ECS system is 

1. find an entity with the components you need
2. read/modify those components
3. create/delete components depending on the situation

In silk, ECS system is implemented in `ecs.hpp` and `ecs.cpp`. The `Entity` struct is a lookup table for its components and the real data is stored in the `Registry` class. 

Here's a quick cheatsheet of ECS system in silk.

```c++  
Registry registry; //Assumes this is the global ECS registry (data storage class)  

// This code loops through all entities with the desired component.
for (Entity& e : registry.get_all_entities()) {
    // Query component ClothConfig of the current entity.
    // The get method returns a ptr to the component. nullptr if component doesn't exist.
    auto config = registry.get<ClothConfig>(e);
    
    // If this entity does not have ClothConfig component,
    // It's not the object we are interested in.
    if (!config) {
      continue;
    }

	// If current entity does have ClothConfig component, do some computation on it.
    my_very_smart_computation(config);
    
    // To remove component.
    registry.remove<ClothConfig>(e);
    
    // To add/replace component.
    ClothConfig my_new_config;
    // The set method returns a ptr the to newly added component if successed.
    // Else return nullptr.
    auto config = registry.set<ClothConfig>(e, my_new_config);
}


// Another common pattern is to get a dense vector of a component type
std::vector<ClothConfig>& configs = registry.get_all<ClothConfig>();
```

### What does this component do?

| Component type     | Description                                               |
| ------------------ | --------------------------------------------------------- |
| ClothConfig        | Physical parameters for cloth                             |
| CollisionConfig    | Collision settings                                        |
| TriMesh            | Triangular mesh                                           |
| Pin                | Pinned vertices and its position                          |
| ClothTopology      | The intermediate matrix data of cloth solver (static)     |
| ClothSolverContext | The intermediate matrix data of cloth solver (dynamic)    |
| ObjectState        | The current position of the object                        |
| ObstaclePosition   | The position info of the obstacle                         |
| ObjectCollider     | The all in one mega data structure for collision pipeline |

## Key Files Map

- `silk/src/world.cpp:22` Public API implementation/entry point.
- `silk/src/solver/solver_pipeline.hpp` Main simulation loop.
- `silk/src/collision_pipeline.hpp` Main collision pipeline entry point. Integrate broadphase detection, narrowphase detection, and collision response computation. 
