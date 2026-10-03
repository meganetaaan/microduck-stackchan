// MuJoCo stores compiled mesh_texcoord with V=0 at this screen's top.
// The PNG is also top-origin. Three.js must not flip it a second time.
export function configureScreenTexture(texture) {
  texture.flipY = false;
  texture.needsUpdate = true;
  return texture;
}
