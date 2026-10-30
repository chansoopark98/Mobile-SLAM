// reality.imagefound / imageupdated / imagelost of the slam_refact xr-slam chunk: the original diff
// over the detected image targets of one response (ARC:reality/app/xr/js/src/tracking-controller.ts:735-758;
// deployed SDK/xr-slam.js `S.forEach(...)` after `getDetection().getImages()`).
//   detectedImageMap: name -> last dispatched image (mutated); images: this response's images (built
//   like the original DetectedImageTarget objects); dispatch(name, detail) = framework.dispatchEvent.
export function dispatchImageEvents(detectedImageMap, images, dispatch) {
  const seenNames = new Set();
  images.forEach((img) => {
    seenNames.add(img.name);
    const found = detectedImageMap[img.name];
    if (!found) {
      detectedImageMap[img.name] = img;
      dispatch('imagefound', img);
      return;
    }
    if (JSON.stringify(found) !== JSON.stringify(img)) {
      detectedImageMap[img.name] = img;
      dispatch('imageupdated', img);
    }
  });
  Object.keys(detectedImageMap).forEach((k) => {
    if (!seenNames.has(k)) {
      const found = { ...detectedImageMap[k] };
      dispatch('imagelost', found);
      delete detectedImageMap[k];
    }
  });
}
