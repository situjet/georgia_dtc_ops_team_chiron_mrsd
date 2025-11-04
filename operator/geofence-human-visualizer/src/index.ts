import { ExtensionContext } from "@foxglove/extension";

import { initGeofenceHumanPanel } from "./GeofenceHumanPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "geofence-human-visualizer",
    initPanel: initGeofenceHumanPanel,
  });
}

