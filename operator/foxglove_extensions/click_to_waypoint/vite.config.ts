import { defineConfig } from "vite";

export default defineConfig({
  build: {
    lib: {
      entry: "src/index.ts",
      formats: ["es"],
      fileName: () => "index.js",
    },
    rollupOptions: {
      external: [],
    },
    target: "es2020",
    sourcemap: true,
    outDir: "dist",
    emptyOutDir: true,
  },
});
