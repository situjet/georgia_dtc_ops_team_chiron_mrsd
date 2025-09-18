import { ExtensionContext } from "@foxglove/extension";

import { initExamplePanel } from "./ExamplePanel";
import { initVirtualGimbalPanel } from "./VirtualGimbalPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "example-panel", initPanel: initExamplePanel });
  extensionContext.registerPanel({ name: "gimbal-virtual-controller", initPanel: initVirtualGimbalPanel });
}
