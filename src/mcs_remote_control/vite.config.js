import { defineConfig } from "vite";

export default defineConfig({
  root: "src",
  build: {
    outDir: "../dist",
    emptyOutDir: true,
  },
  server: {
    port: 3000,
    host: true,
    open: "/login.html",
  },
  preview: {
    port: 8080,
    host: true,
  },
});
