import { ExtensionContext } from "@foxglove/extension";

import { initGimbalJoyPanel } from "./panels/GimbalJoyPanel";

export function activate(extensionContext: ExtensionContext): void {
  // Register the Gimbal Joy Controller panel
  extensionContext.registerPanel({ name: "gimbal-joy-controller", initPanel: initGimbalJoyPanel });
}
