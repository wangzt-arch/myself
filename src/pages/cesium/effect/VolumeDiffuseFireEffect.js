import * as Cesium from "cesium";

// 体散射火
class VolumeDiffuseFireEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("VolumeDiffuseFireEffect: viewer is required");
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

      float snoise(vec3 uv, float res) {
        const vec3 s = vec3(1e0, 1e2, 1e3);
        uv *= res;
        vec3 uv0 = floor(mod(uv, res))*s;
        vec3 uv1 = floor(mod(uv+vec3(1.), res))*s;
        vec3 f = fract(uv); f = f*f*(3.0-2.0*f);
        vec4 v = vec4(uv0.x+uv0.y+uv0.z, uv1.x+uv0.y+uv0.z,
          uv0.x+uv1.y+uv0.z, uv1.x+uv1.y+uv0.z);
        vec4 r = fract(sin(v*1e-1)*1e3);
        float r0 = mix(mix(r.x, r.y, f.x), mix(r.z, r.w, f.x), f.y);
        r = fract(sin((v + uv1.z - uv0.z)*1e-1)*1e3);
        float r1 = mix(mix(r.x, r.y, f.x), mix(r.z, r.w, f.x), f.y);
        return mix(r0, r1, f.z)*2.-1.;
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
        vec2 p = uv;
        float c = 3.0 - (3.*length(2.*p));
        vec3 coord = vec3(atan(p.x,p.y)/6.2832+.5, length(p)*.4, .5);
        for(int i = 1; i <= 7; i++) {
          float power = pow(2.0, float(i));
          c += (1.5 / power) * snoise(coord + vec3(0.,-iTime*.05, iTime*.01), power*16.);
        }
        vec3 col = vec3(c, pow(max(c,0.),2.)*0.4, pow(max(c,0.),3.)*0.15);
        col *= color.rgb;
        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0,1.0,length(col.rgb));
        vec4 fragColor = vec4(col.rgb, alpha);
        material.diffuse = fragColor.rgb * 1.0;
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
    const materialType = `VolumeDiffuseFireMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#ff6600").withAlpha(0.9),
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
    throw new Error("VolumeDiffuseFireEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
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
      if (CesiumRef.SceneTransforms && CesiumRef.SceneTransforms.worldToWindowCoordinates) {
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

export default VolumeDiffuseFireEffect;
