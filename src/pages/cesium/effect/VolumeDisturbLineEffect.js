import * as Cesium from "cesium";

// 干扰线域
class VolumeDisturbLineEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("VolumeDisturbLineEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height ?? 0,
      radius: options.radius ?? 300000,
      speed: options.speed ?? 3.0,
      color: options.color,
      autoAnimate: options.autoAnimate !== false,
    };

    this.material = null;
    this.primitive = null;
    this.position = null;
    this.animationListener = null;
    this.isDestroyed = false;

    this._init();
  }

  _getShaderSource() {
    return `
      uniform vec4 color;
      uniform float speed;
      uniform vec2 px;
      uniform float pixel;

      float iTime = 0.0;
      vec2 iResolution = vec2(0.0);
      mat3 m = mat3(0.00, 0.80, 0.60, -0.80, 0.36, -0.48, -0.60, -0.48, 0.64);

      float hash(float n) { return fract(sin(n)*43758.5453123); }

      float noise(in vec3 x) {
        vec3 p = floor(x); vec3 f = fract(x);
        f = f*f*(3.0-2.0*f);
        float n = p.x + p.y*57.0 + 113.0*p.z;
        float res = mix(mix(mix(hash(n+0.0), hash(n+1.0),f.x),
          mix(hash(n+57.0), hash(n+58.0),f.x),f.y),
          mix(mix(hash(n+113.0), hash(n+114.0),f.x),
          mix(hash(n+170.0), hash(n+171.0),f.x),f.y),f.z);
        return res;
      }

      float fbm(vec3 p) {
        float f = 0.0;
        f += 0.5000*noise(p); p = m*p*2.02;
        f += 0.2500*noise(p); p = m*p*2.03;
        f += 0.1250*noise(p);
        return f/0.875;
      }

      vec3 gradient(float s) {
        return vec3(0.0, max(1.0-s*2.0, 0.0), max(s>0.5?1.0-(s-0.5)*5.0:1.0, 0.0));
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        iTime = czm_frameNumber/60.0 * speed;
        iResolution = czm_viewport.zw;

        vec2 uv = (2.0 * gl_FragCoord.xy - iResolution.xy) / min(iResolution.x,iResolution.y);
        vec2 center = vec2(px.x, iResolution.y - px.y);
        vec2 p2 = (2.0 * center.xy - iResolution.xy) / min(iResolution.x,iResolution.y);
        uv = uv - p2;
        uv = uv * 1.0 * min(iResolution.x,iResolution.y)/pixel;

        vec3 col = vec3(0.0);
        float s = (0.5 - length(uv)) * 2.0; s *= s;
        float a = 0.01; float b = 0.25; float d = 0.0;
        for (int j = 0; j < 3; j++) {
          d += 0.5/abs((fbm(5.0*vec3(uv,0.0)*b+0.075*iTime/b)*2.0-1.0)/a);
          b *= 2.0; a /= 2.0;
        }
        col += gradient(s)*max(d*s,0.0);
        col *= color.rgb;
        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0,1.0,length(col));
        vec4 fragColor = vec4(col, alpha);

        material.diffuse = fragColor.rgb * 2.0;
        material.alpha = fragColor.a * color.a;
        return material;
      }
    `;
  }

  _init() {
    this._registerMaterial();
    this._createPrimitive();
    if (this.options.autoAnimate) {
      this.startAnimation();
    }
  }

  _registerMaterial() {
    const CesiumRef = this._getCesium();
    const materialType = `VolumeDisturbLineMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#00ff88").withAlpha(0.9),
      speed: this.options.speed,
      px: new CesiumRef.Cartesian2(0.0, 0.0),
      pixel: 300.0,
    };

    if (CesiumRef.Material._materialCache) {
      CesiumRef.Material._materialCache.addMaterial(materialType, {
        fabric: {
          type: materialType,
          uniforms,
          source: this._getShaderSource(),
        },
        translucent: true,
      });
    }

    this.material = new CesiumRef.Material({
      fabric: {
        type: materialType,
        uniforms: { ...uniforms },
      },
      translucent: true,
    });
  }

  _createPrimitive() {
    const CesiumRef = this._getCesium();
    const { longitude, latitude, height, radius } = this.options;
    this.position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, height);

    this.primitive = new CesiumRef.Primitive({
      geometryInstances: new CesiumRef.GeometryInstance({
        geometry: new CesiumRef.EllipseGeometry({
          center: this.position,
          semiMinorAxis: radius,
          semiMajorAxis: radius,
          height,
          vertexFormat: CesiumRef.EllipsoidSurfaceAppearance.VERTEX_FORMAT,
        }),
      }),
      appearance: new CesiumRef.EllipsoidSurfaceAppearance({
        material: this.material,
        aboveGround: true,
      }),
      asynchronous: false,
      show: true,
    });

    this.viewer.scene.primitives.add(this.primitive);
  }

  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("VolumeDisturbLineEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  startAnimation() {
    if (this.animationListener || !this.material) return;
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed || !this.material) return;
      const CesiumRef = this._getCesium();
      let px;
      if (CesiumRef.SceneTransforms.wgs84ToWindowCoordinates) {
        px = CesiumRef.SceneTransforms.wgs84ToWindowCoordinates(this.viewer.scene, this.position);
      } else if (CesiumRef.SceneTransforms.worldToWindowCoordinates) {
        px = CesiumRef.SceneTransforms.worldToWindowCoordinates(this.viewer.scene, this.position);
      }
      if (px) {
        this.material.uniforms.px = px;
      }
    });
  }

  stopAnimation() {
    if (this.animationListener) {
      this.viewer.scene.preRender.removeEventListener(this.animationListener);
      this.animationListener = null;
    }
  }

  setSpeed(speed) {
    this.options.speed = speed;
    if (this.material) {
      this.material.uniforms.speed = speed;
    }
  }

  setColor(color) {
    if (this.material) {
      this.material.uniforms.color = color;
    }
  }

  show() {
    if (this.primitive) {
      this.primitive.show = true;
    }
  }

  hide() {
    if (this.primitive) {
      this.primitive.show = false;
    }
  }

  destroy() {
    this.isDestroyed = true;
    this.stopAnimation();

    if (this.primitive) {
      this.viewer.scene.primitives.remove(this.primitive);
      this.primitive = null;
    }

    if (this.material) {
      this.material.destroy();
      this.material = null;
    }

    this.viewer = null;
  }
}

export default VolumeDisturbLineEffect;
