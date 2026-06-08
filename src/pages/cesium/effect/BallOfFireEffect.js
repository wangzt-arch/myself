import * as Cesium from "cesium";

/**
 * BallOfFireEffect - reusable Cesium GLSL fireball effect.
 *
 * The shader uses a compact procedural flame field inspired by the referenced
 * Shadertoy fire material, rendered on crossed slices with an irregular cone core.
 */
class BallOfFireEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("BallOfFireEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height,
      radius: options.radius ?? 120000,
      speed: options.speed ?? 0.8,
      heightOffset: options.heightOffset ?? 0.5,
      baseColor: options.baseColor,
      coreColor: options.coreColor,
      autoAnimate: options.autoAnimate !== false,
    };

    this.material = null;
    this.primitive = null;
    this.animationListener = null;
    this.startTime = null;
    this.isDestroyed = false;

    this._init();
  }

  _getShaderSource() {
    return `
      uniform vec4 color;
      uniform vec4 coreColor;
      uniform float speed;
      uniform float time;

      float noise(vec3 p) {
        vec3 i = floor(p);
        vec4 a = dot(i, vec3(1.0, 57.0, 21.0)) + vec4(0.0, 57.0, 21.0, 78.0);
        vec3 f = cos((p - i) * acos(-1.0)) * -0.5 + 0.5;
        a = mix(sin(cos(a) * a), sin(cos(1.0 + a) * (1.0 + a)), f.x);
        a.xy = mix(a.xz, a.yw, f.y);
        return mix(a.x, a.y, f.z);
      }

      float fbm(vec3 p) {
        float value = 0.0;
        float amp = 0.55;
        for (int i = 0; i < 5; i++) {
          value += noise(p) * amp;
          p *= 2.08;
          amp *= 0.52;
        }
        return value;
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        vec2 st = materialInput.st;
        vec2 uv = vec2(st.x * 2.0 - 1.0, st.y);
        float t = time * speed;

        float rise = fbm(vec3(uv.x * 1.7, uv.y * 3.1 - t * 1.7, t * 0.24));
        float orangeFlow = fbm(vec3(uv.x * 2.8 + rise * 0.5, uv.y * 5.4 - t * 2.85, 0.8));
        float lick = fbm(vec3(uv.x * 4.4 + rise * 0.35, uv.y * 7.4 - t * 2.45, 1.6));
        float veins = sin(uv.x * 9.0 + uv.y * 8.0 - t * 3.1) * 0.5 + 0.5;
        float sway = sin(uv.y * 7.0 + t * 2.2) * 0.1 + rise * 0.22;

        float width = mix(1.2, 0.08, pow(uv.y, 1.62));
        width *= 1.0 - smoothstep(0.78, 1.0, uv.y) * 0.38;
        width += (rise * 0.28 + lick * 0.16) * (1.0 - uv.y);
        float dist = abs(uv.x + sway);
        float body = smoothstep(width, width * 0.34, dist);
        float baseGlow = smoothstep(0.08, 0.0, uv.y) * smoothstep(1.0, 0.15, abs(uv.x));
        float hotCore = smoothstep(width * 0.28, 0.0, dist) * smoothstep(0.56, 0.04, uv.y);
        float tongues = smoothstep(0.18, 0.82, orangeFlow + lick * 0.36 + veins * 0.24) * smoothstep(width * 1.12, width * 0.46, dist);
        float fadeTop = smoothstep(1.0, 0.76, uv.y);
        float fadeBottom = smoothstep(0.0, 0.1, uv.y);
        float intensity = clamp((body * 0.72 + hotCore * 0.55 + tongues * 0.42 + baseGlow * 0.5) * fadeTop * fadeBottom, 0.0, 1.0);

        vec3 ember = vec3(0.18, 0.012, 0.0);
        vec3 redOrange = vec3(0.62, 0.12, 0.015);
        vec3 orange = mix(color.rgb, redOrange, 0.45) * 1.05;
        vec3 brightOrange = vec3(0.82, 0.2, 0.02) * 0.9;
        vec3 smokeBlue = vec3(0.08, 0.2, 0.45);

        float orangeWeight = smoothstep(0.02, 0.3, intensity + orangeFlow * 0.36 + tongues * 0.16);
        vec3 flameColor = mix(ember, orange, orangeWeight);
        flameColor = mix(flameColor, brightOrange, smoothstep(0.82, 1.08, hotCore + baseGlow * 0.08) * 0.12);
        flameColor = mix(flameColor, smokeBlue, smoothstep(0.62, 1.0, uv.y) * smoothstep(0.2, 0.0, intensity));

        float edgeSoftness = smoothstep(width * 1.34, width * 0.58, dist);
        float alpha = clamp(intensity * edgeSoftness * color.a * 0.72, 0.0, 0.62);

        czm_material material = czm_getDefaultMaterial(materialInput);
        material.diffuse = flameColor;
        material.emission = flameColor * (0.68 + intensity * 0.9 + hotCore * 0.18);
        material.alpha = alpha;
        material.specular = 0.0;
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
    const materialType = `BallOfFireMaterial_${this._generateId()}`;
    this._materialType = materialType;

    const uniforms = {
      color: this.options.baseColor || new CesiumRef.Color(1.0, 0.26, 0.02, 1.0),
      coreColor: this.options.coreColor || new CesiumRef.Color(1.0, 0.9, 0.42, 1.0),
      speed: this.options.speed,
      time: 0.0,
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
    const { longitude, latitude, height, radius, heightOffset } = this.options;
    const targetHeight = height !== undefined ? height : radius * heightOffset;
    const flameHeight = radius * 1.48;
    void flameHeight;
    this.position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, targetHeight + radius * 1.58 * 0.5);

    this.primitive = new CesiumRef.Primitive({
      geometryInstances: new CesiumRef.GeometryInstance({
        geometry: new CesiumRef.PlaneGeometry({
          vertexFormat: CesiumRef.MaterialAppearance.MaterialSupport.TEXTURED.vertexFormat,
        }),
      }),
      appearance: new CesiumRef.MaterialAppearance({
        material: this.material,
        translucent: true,
        faceForward: true,
        renderState: {
          depthTest: { enabled: true },
          depthMask: false,
          blending: CesiumRef.BlendingState.ALPHA_BLEND,
        },
      }),
      asynchronous: false,
    });

    this._syncPrimitive();
    this.viewer.scene.primitives.add(this.primitive);
  }

  _syncPrimitive() {
    if (!this.primitive || !this.position) return;

    const CesiumRef = this._getCesium();
    const { radius } = this.options;
    const flameHeight = radius * 1.48;
    const flameWidth = radius * 1.58;

    // Get camera position and compute direction to face camera
    const camera = this.viewer.camera;
    const cameraPos = camera.positionWC;

    // Compute vector from position to camera
    const toCamera = CesiumRef.Cartesian3.subtract(cameraPos, this.position, new CesiumRef.Cartesian3());
    CesiumRef.Cartesian3.normalize(toCamera, toCamera);

    // Build rotation matrix that makes plane face the camera
    // Plane normal (Z axis) should point to camera
    const zAxis = toCamera;

    // Choose up vector
    const up = CesiumRef.Cartesian3.normalize(this.position, new CesiumRef.Cartesian3());

    // Compute xAxis = up × zAxis
    const xAxis = CesiumRef.Cartesian3.cross(up, zAxis, new CesiumRef.Cartesian3());
    if (CesiumRef.Cartesian3.magnitude(xAxis) < 0.001) {
      CesiumRef.Cartesian3.clone(CesiumRef.Cartesian3.UNIT_X, xAxis);
    }
    CesiumRef.Cartesian3.normalize(xAxis, xAxis);

    // Compute yAxis = zAxis × xAxis
    const yAxis = CesiumRef.Cartesian3.cross(zAxis, xAxis, new CesiumRef.Cartesian3());
    CesiumRef.Cartesian3.normalize(yAxis, yAxis);

    // Build rotation matrix (plane faces camera)
    const rotationMatrix = CesiumRef.Matrix4.fromRotationTranslation(
      new CesiumRef.Matrix3(
        xAxis.x, yAxis.x, zAxis.x,
        xAxis.y, yAxis.y, zAxis.y,
        xAxis.z, yAxis.z, zAxis.z
      )
    );

    // Translation to position
    const translationMatrix = CesiumRef.Matrix4.fromTranslation(this.position);

    // Scale
    const scaleMatrix = CesiumRef.Matrix4.fromScale(new CesiumRef.Cartesian3(flameWidth, flameHeight, 1.0));

    // Combine: translation * rotation * scale
    let modelMatrix = CesiumRef.Matrix4.multiply(translationMatrix, rotationMatrix, new CesiumRef.Matrix4());
    modelMatrix = CesiumRef.Matrix4.multiply(modelMatrix, scaleMatrix, modelMatrix);
    this.primitive.modelMatrix = modelMatrix;
  }

  _createFlameConeGeometry(CesiumRef, diameter, height) {
    const radialSegments = 56;
    const heightSegments = 18;
    const positions = [];
    const normals = [];
    const textureCoordinates = [];
    const indices = [];
    const maxRadius = diameter * 0.5;

    for (let yIndex = 0; yIndex <= heightSegments; yIndex += 1) {
      const v = yIndex / heightSegments;
      const z = -height * 0.5 + v * height;
      const taper = Math.pow(1.0 - v, 0.82);
      const waist = 0.82 + Math.sin(v * Math.PI) * 0.24;
      const leanX = Math.sin(v * Math.PI * 1.35) * maxRadius * 0.12;
      const leanY = Math.sin(v * Math.PI * 2.15 + 0.6) * maxRadius * 0.08;

      for (let xIndex = 0; xIndex < radialSegments; xIndex += 1) {
        const u = xIndex / radialSegments;
        const angle = u * Math.PI * 2;
        const ripple = 1
          + Math.sin(angle * 3.0 + v * 5.2) * 0.11
          + Math.sin(angle * 7.0 - v * 8.4) * 0.06
          + Math.cos(angle * 11.0 + v * 3.6) * 0.035;
        const ringRadius = Math.max(maxRadius * 0.03, maxRadius * taper * waist * ripple);
        const x = Math.cos(angle) * ringRadius + leanX;
        const y = Math.sin(angle) * ringRadius + leanY;

        positions.push(x, y, z);
        normals.push(Math.cos(angle), Math.sin(angle), 0.36);
        textureCoordinates.push(u, v);
      }
    }

    for (let yIndex = 0; yIndex < heightSegments; yIndex += 1) {
      for (let xIndex = 0; xIndex < radialSegments; xIndex += 1) {
        const nextX = (xIndex + 1) % radialSegments;
        const current = yIndex * radialSegments + xIndex;
        const right = yIndex * radialSegments + nextX;
        const above = (yIndex + 1) * radialSegments + xIndex;
        const aboveRight = (yIndex + 1) * radialSegments + nextX;

        indices.push(current, right, above);
        indices.push(right, aboveRight, above);
      }
    }

    const bottomCenterIndex = positions.length / 3;
    positions.push(0, 0, -height * 0.5);
    normals.push(0, 0, -1);
    textureCoordinates.push(0.5, 0);
    for (let xIndex = 0; xIndex < radialSegments; xIndex += 1) {
      indices.push(bottomCenterIndex, (xIndex + 1) % radialSegments, xIndex);
    }

    const topCenterIndex = positions.length / 3;
    const topLeanX = Math.sin(Math.PI * 1.35) * maxRadius * 0.12;
    const topLeanY = Math.sin(Math.PI * 2.15 + 0.6) * maxRadius * 0.08;
    const topStart = heightSegments * radialSegments;
    positions.push(topLeanX, topLeanY, height * 0.5);
    normals.push(0, 0, 1);
    textureCoordinates.push(0.5, 1);
    for (let xIndex = 0; xIndex < radialSegments; xIndex += 1) {
      const nextX = (xIndex + 1) % radialSegments;
      indices.push(topStart + xIndex, topStart + nextX, topCenterIndex);
    }

    return new CesiumRef.Geometry({
      attributes: {
        position: new CesiumRef.GeometryAttribute({
          componentDatatype: CesiumRef.ComponentDatatype.DOUBLE,
          componentsPerAttribute: 3,
          values: new Float64Array(positions),
        }),
        normal: new CesiumRef.GeometryAttribute({
          componentDatatype: CesiumRef.ComponentDatatype.FLOAT,
          componentsPerAttribute: 3,
          values: new Float32Array(normals),
        }),
        st: new CesiumRef.GeometryAttribute({
          componentDatatype: CesiumRef.ComponentDatatype.FLOAT,
          componentsPerAttribute: 2,
          values: new Float32Array(textureCoordinates),
        }),
      },
      indices: new Uint16Array(indices),
      primitiveType: CesiumRef.PrimitiveType.TRIANGLES,
      boundingSphere: CesiumRef.BoundingSphere.fromVertices(positions),
    });
  }

  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("BallOfFireEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  startAnimation() {
    if (this.animationListener || !this.material) return;

    this.startTime = Date.now();
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed) return;
      this.material.uniforms.time = (Date.now() - this.startTime) / 1000;
      this._syncPrimitive();
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

  moveTo(longitude, latitude, height) {
    if (height !== undefined) {
      this.options.height = height;
    }

    if (this.primitive) {
      this.viewer.scene.primitives.remove(this.primitive);
      this.primitive = null;
    }

    this.options.longitude = longitude;
    this.options.latitude = latitude;
    this._createPrimitive();
  }

  setRadius(radius) {
    this.options.radius = radius;
    this.moveTo(this.options.longitude, this.options.latitude);
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

export default BallOfFireEffect;
