# Foxglove Click-to-Waypoint Panel

A lightweight Foxglove panel extension that provides:

- Live map with satellite/OSM tiles
- Click to set a waypoint and visualize a 30×30m box around it
- Publishes a simple JSON command to `/behavior_governor/click_waypoint`
- Optionally advertises/publishes a ROS 2 `geographic_msgs/GeoPoseStamped` on `/operator/click_waypoint` if supported by your Foxglove runtime
- Subscribes to `/fix` (sensor_msgs/NavSatFix) to display current drone position

## Build

This is a standard Vite + TypeScript bundle that outputs a single `index.js` file Foxglove can load.

Install deps and build:

```bash
cd operator/foxglove_extensions/click_to_waypoint
npm install
npm run build
```

The output will be written to `dist/index.js`.

## Load in Foxglove

- Open Foxglove Studio (desktop or web) and go to Extensions → Load Local Extension
- Select the built file at `operator/foxglove_extensions/click_to_waypoint/dist/index.js`
- Add the panel via Panels → Custom → Click-to-Waypoint

## Configuration

- Tile source can be switched between OSM and a demo Google Sat layer. Provide your own tile URL if needed.
- Subscribed topic: `/fix` (sensor_msgs/NavSatFix)
- Published (simple JSON): `/behavior_governor/click_waypoint` with `{ type, lat, lon, alt }`
- Published (optional ROS 2): `/operator/click_waypoint` as `geographic_msgs/GeoPoseStamped`

You may need a ROS 2 bridge that converts the JSON message to your desired `behavior_governor` message or service. Alternatively, consume the ROS 2 topic directly if your runtime supports advertise/publish.

## Notes

- This panel intentionally avoids heavy framework dependencies and uses Leaflet via CDN for CSS. The JS module `leaflet` is bundled by Vite.
- The 30 m box is computed in lat/lon using a local flat-earth approximation suitable for small distances.
