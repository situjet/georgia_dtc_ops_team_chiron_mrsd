const foxglove = require("@foxglove/eslint-plugin");

module.exports = {
  root: true,
  extends: [
    foxglove.configs.base,
    foxglove.configs.react,
    foxglove.configs.typescript,
  ],
};

