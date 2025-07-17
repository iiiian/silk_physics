# how to impress

- Best FOSS blender bloth sim plugins

# Next step

- [ ] integrate tight inclusion benchmark/testing -> ccd works correct and efficiently

# Roadmap

## Tier 1

- [x] spatial hashing collision detection (native grouping)
- [x] eigen subspace + iterative solve
- [] blender integration with easy to use gui
- [] cuda solver
- [] install blas and lapack automatically

## Tier 2

- [] store upper triangle only for symmetric lhs
- [] real collision culling based in IPC
- [] auto stiffness tuning
- [] subspace conditioning
- [] initial curvature

# notes

## 0622

1. add obj (invalid solver)
2. delete obj (invalid solver)
3. update obj property
 - update init position (invalid solver)
 - pin position
 - colliding?
 - simulation parameter (invalid solver)
4. get current obj vertex position

- handle needs to be typed
- access can return err

- cloth
  - init position
  - mesh topo
  - vertex map to global vec
  - cloth para
- softbody
  - init position
  - mesh topo
  - vertex map to global vec
  - softbody para
- rigid body
  - init position
  - mesh topo
  - vertex map to global vec
  - rigid para
- hair
  - init position
  - mesh topo
  - vertex map to global vec
  - hair para
- collider
  - init position
  - mesh topo

## 0625

- demo test
- separate engine/demo
- collision pipeline
- rigidbody
- gpu solver
- softbody
- hair

## 0712

```
sudo dnf install alembic-devel imath-devel
```

## 0716

- ccd poly forward / backward
- double check ccd poly eps
