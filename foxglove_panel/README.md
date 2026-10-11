# Hexapod Foxglove panel

A Foxglove panel for the hexapod, through `foxglove_bridge`:

- **Body**: roll and pitch from `/imu/data`, with an attitude indicator.
- **Feet**: the six feet in their layout, filled when the foot's upward force on
  `/foot_force/legN` is above gait_controller's `contact_force_on`.
- **Power**: voltage, current and charge from `/power/left` and `/power/right`.
- **Mode and gait**: from `get_capabilities`, changed with `set_mode` and `set_gait`.
  Height is sent with `set_height` when the slider is let go (8–15 cm).
- **Features**: gait_controller's switchable features. Each shows the node's
  parameter and changes it with its `set_<feature>` service.

## Build and install

Needs Node.js 18 or later.

```sh
npm install
npm run package        # writes raven.hexapod-panel-<version>.foxe
```

Install the `.foxe` by dragging it onto the Foxglove window (or Settings →
Extensions), then add the **Hexapod** panel to a layout. `npm run local-install`
installs it into the Foxglove app on the same machine instead.
