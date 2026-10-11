import { ExtensionContext } from "@foxglove/extension";

import { initHexapodPanel } from "./HexapodPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Hexapod", initPanel: initHexapodPanel });
}
