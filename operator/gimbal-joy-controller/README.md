# gimbal-joy-controller

[Foxglove](https://foxglove.dev) allows developers to create [extensions](https://docs.foxglove.dev/docs/visualization/extensions/introduction), or custom code that is loaded and executed inside the Foxglove application. This can be used to add custom panels. Extensions are authored in TypeScript using the `@foxglove/extension` SDK.

## Develop

Extension development uses the `npm` package manager to install development dependencies and run build scripts.

To install extension dependencies, run `npm` from the root of the extension package.

```sh
npm install
```

To build and install the extension into your local Foxglove desktop app, run:

```sh
npm run local-install
```

Open the Foxglove desktop (or `ctrl-R` to refresh if it is already open). Your extension is installed and available within the app.

## Gimbal Joy Controller panel

This extension provides a panel named `gimbal-joy-controller` that reads inputs from a connected gamepad (e.g. Xbox controller) using the browser Gamepad API. It supports:

- Pan up/down/left/right (digital via D-pad and analog via stick axes)
- Zoom in/out
- Trigger button
- Mode switch button
- Recenter camera button

On first use, you can click "Setup bindings" to run a simple wizard: press the requested buttons in order to bind them. You can also configure button indices and axes under the panel settings sidebar at any time.

The panel publishes a simple object on the topic (default `/gimbal/teleop`) when publishing is supported by the current data source:

```
{
	panX: number, // -1..1
	panY: number, // -1..1
	zoom: -1 | 0 | 1,
	trigger: boolean,      // edge
	modeSwitch: boolean,   // edge
	recenter: boolean      // edge
}
```

You can remap or process this message in user scripts or downstream bridges.

## Package

Extensions are packaged into `.foxe` files. These files contain the metadata (package.json) and the build code for the extension.

Before packaging, make sure to set `name`, `publisher`, `version`, and `description` fields in _package.json_. When ready to distribute the extension, run:

```sh
npm run package
```

This command will package the extension into a `.foxe` file in the local directory.

## Publish

You can publish the extension to the public registry or privately for your organization.

See documentation here: https://docs.foxglove.dev/docs/visualization/extensions/publish/#packaging-your-extension
