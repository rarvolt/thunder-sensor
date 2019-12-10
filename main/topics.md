tdesp/  - main device topic

## telemetry:
- `tdesp/temperature` - temperature in degC
- `tdesp/humidity` - % RH
- `tdesp/pressure` - pressure in pascals
- `tdesp/luminosity` - luminosity in lux
- `tdesp/thunder/present` - thunder detected
- `tdesp/thunder/distance` - distance to thunder front

## commands
#### `tdesp/cmd/telemetry_interval` 
Change telemetry interval in seconds.

Message:

`value` - interval in seconds

Empty value: return current interval on topic `tdesp/telemetry_interval`

#### `tdesp/cmd/t1`
#### `tdesp/cmd/t2`
#### `tdesp/cmd/rl1`
#### `tdesp/cmd/rl2`

