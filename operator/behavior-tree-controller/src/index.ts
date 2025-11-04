import { ExtensionContext } from "@foxglove/extension";

import { initBehaviorTreeControllerPanel } from "./BehaviorTreeControllerPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ 
    name: "behavior-tree-controller", 
    initPanel: initBehaviorTreeControllerPanel 
  });
}

