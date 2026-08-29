# Ordinary vehicle EKF

Two ordinary four-armor vehicle estimators are isolated here:

- `superpower_tongji/`: latest SuperPower/Tongji implementation copied from `xiugai` without algorithm rollback, including joint update support.
- `alliance_njust/`: NJUST/Alliance 11D rigid-body + 2D lightbar reprojection EKF.

`EKF/SuperPowerPredictor.{h,cpp}` is now a compatibility facade so the existing `xiugai` AllPredictor business path stays unchanged.

Switch at runtime in the YAML passed to the executable
(`configs/infantry.yaml` or `configs/infantry_video.yaml`):

```yaml
ordinary_vehicle_ekf:
  backend: superpower
```

or

```yaml
ordinary_vehicle_ekf:
  backend: alliance
```

Restart the executable after changing it; no rebuild is needed. Base and Outpost keep their existing paths.
