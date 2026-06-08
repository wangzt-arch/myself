import * as Cesium from "cesium";
import explosionGif from "./baozha.gif";

class CircleGifEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("CircleGifEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height ?? 0,
      radius: options.radius ?? 300000,
      speed: options.speed ?? 1.0,
      image: options.image || explosionGif,
    };

    this.entity = null;
    this.isDestroyed = false;
    this._init();
  }

  _init() {
    const CesiumRef = this._getCesium();
    const { longitude, latitude, height, radius } = this.options;
    const position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, height);

    this.entity = this.viewer.entities.add({
      name: "Circle Gif",
      position,
      ellipse: {
        semiMinorAxis: radius,
        semiMajorAxis: radius,
        height,
        material: new CesiumRef.ImageMaterialProperty({
          image: this.options.image,
          transparent: true,
        }),
      },
    });
  }

  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("CircleGifEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  destroy() {
    this.isDestroyed = true;

    if (this.entity && this.viewer) {
      this.viewer.entities.remove(this.entity);
      this.entity = null;
    }

    this.viewer = null;
  }
}

export default CircleGifEffect;
