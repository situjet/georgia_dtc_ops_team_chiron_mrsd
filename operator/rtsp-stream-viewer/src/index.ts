import { ExtensionContext } from "@foxglove/extension";

import { initRTSPStreamPanel } from "./RTSPStreamPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "rtsp-stream-viewer", initPanel: initRTSPStreamPanel });
}
