import * as Cesium from "cesium";
import BallOfFireEffect from "./effect/BallOfFireEffect";
import EmberSphereEffect from "./effect/EmberSphereEffect";
import ExplosionSphereEffect from "./effect/ExplosionSphereEffect";
import VolumeSmokeEffect from "./effect/VolumeSmokeEffect";

export const EFFECT_TYPES = [
  { key: "flame", label: "火球", color: "#ff7a18" },
  { key: "fireball", label: "火焰", color: "#ff9d1c" },
  { key: "volumeSmoke", label: "\u4f53\u79ef\u70df", color: "#b6b0a8" },
  { key: "explosion", label: "爆炸", color: "#ffce45" },
  { key: "energyWall", label: "能量墙", color: "#00f5ff" },
  { key: "alarmWall", label: "警戒墙", color: "#ff4d6d" },
  { key: "glow", label: "发光", color: "#8fff6a" },
];

const getEffectType = (type) => EFFECT_TYPES.find((item) => item.key === type) || EFFECT_TYPES[0];

const createPulse = (viewer, duration, from = 0, to = 1) => {
  return new Cesium.CallbackProperty(() => {
    const seconds = Cesium.JulianDate.secondsDifference(viewer.clock.currentTime, viewer.clock.startTime);
    const progress = ((seconds % duration) + duration) % duration / duration;
    return from + (to - from) * (0.5 + Math.sin(progress * Cesium.Math.TWO_PI) * 0.5);
  }, false);
};

const createOpenWallPositions = (center, halfWidth, halfDepth, height = 0) => {
  const transform = Cesium.Transforms.eastNorthUpToFixedFrame(center);
  const corners = [
    new Cesium.Cartesian3(-halfWidth, -halfDepth, height),
    new Cesium.Cartesian3(halfWidth, -halfDepth, height),
    new Cesium.Cartesian3(halfWidth, halfDepth, height),
    new Cesium.Cartesian3(-halfWidth * 0.35, halfDepth, height),
  ];

  return corners.map((corner) => {
    const point = Cesium.Matrix4.multiplyByPoint(transform, corner, new Cesium.Cartesian3());
    const cartographic = Cesium.Cartographic.fromCartesian(point);
    return Cesium.Cartesian3.fromRadians(cartographic.longitude, cartographic.latitude, height);
  });
};

const createAlarmWallTexture = () => {
  if (typeof document === "undefined") return "";

  const canvas = document.createElement("canvas");
  canvas.width = 256;
  canvas.height = 128;
  const ctx = canvas.getContext("2d");

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#6d0014";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.save();
  ctx.translate(-80, 0);
  for (let index = 0; index < 12; index += 1) {
    ctx.fillStyle = index % 2 === 0 ? "#ffd326" : "#151515";
    ctx.beginPath();
    ctx.moveTo(index * 42, 0);
    ctx.lineTo(index * 42 + 26, 0);
    ctx.lineTo(index * 42 - 22, canvas.height);
    ctx.lineTo(index * 42 - 48, canvas.height);
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();

  ctx.fillStyle = "#b30026";
  ctx.fillRect(0, 38, canvas.width, 52);

  [128].forEach((x) => {
    ctx.beginPath();
    ctx.moveTo(x, 24);
    ctx.lineTo(x + 36, 92);
    ctx.lineTo(x - 36, 92);
    ctx.closePath();
    ctx.fillStyle = "#ffdc2a";
    ctx.fill();
    ctx.lineWidth = 6;
    ctx.strokeStyle = "#121212";
    ctx.stroke();

    ctx.fillStyle = "#141414";
    ctx.fillRect(x - 4, 45, 8, 25);
    ctx.beginPath();
    ctx.arc(x, 80, 5, 0, Math.PI * 2);
    ctx.fill();
  });

  ctx.strokeStyle = "rgba(255, 255, 255, 0.34)";
  ctx.lineWidth = 2;
  ctx.strokeRect(1, 1, canvas.width - 2, canvas.height - 2);

  return canvas.toDataURL("image/png");
};

const createSingleWallPositions = (center, halfWidth, height = 0) => {
  const transform = Cesium.Transforms.eastNorthUpToFixedFrame(center);
  const corners = [
    new Cesium.Cartesian3(-halfWidth, 0, height),
    new Cesium.Cartesian3(halfWidth, 0, height),
  ];

  return corners.map((corner) => {
    const point = Cesium.Matrix4.multiplyByPoint(transform, corner, new Cesium.Cartesian3());
    const cartographic = Cesium.Cartographic.fromCartesian(point);
    return Cesium.Cartesian3.fromRadians(cartographic.longitude, cartographic.latitude, height);
  });
};

const addFlameEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const emberSphere = new EmberSphereEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: 95000,
    radius: 110000,
    speed: 0.55,
    baseColor: Cesium.Color.fromCssColorString(color),
    autoAnimate: true,
    Cesium,
  });

  return [emberSphere];
};

const addFireballEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const fireball = new BallOfFireEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: 5000,
    radius: 135000,
    speed: 0.9,
    baseColor: Cesium.Color.fromCssColorString(color),
    coreColor: Cesium.Color.fromCssColorString("#ffe76a"),
    autoAnimate: true,
    Cesium,
  });

  return [fireball];
};

const addExplosionEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const explosionSphere = new ExplosionSphereEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: 115000,
    radius: 175000,
    speed: 0.62,
    baseColor: Cesium.Color.fromCssColorString(color),
    shockColor: Cesium.Color.fromCssColorString("#ff6b35"),
    smokeColor: Cesium.Color.fromCssColorString("#3f1a08"),
    autoAnimate: true,
    Cesium,
  });

  return [explosionSphere];
};

const addVolumeSmokeEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const volumeSmoke = new VolumeSmokeEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height,
    radius: 170,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.82),
    autoAnimate: true,
    Cesium,
  });

  return [volumeSmoke];
};

const addWallEffect = (viewer, position, color, isAlarm = false) => {
  const wallColor = Cesium.Color.fromCssColorString(color);
  const pulse = isAlarm ? null : createPulse(viewer, 2.4, 0.2, 1);
  const halfWidth = isAlarm ? 320000 : 220000;
  const halfDepth = isAlarm ? 130000 : 170000;
  const wallHeight = isAlarm ? 150000 : 270000;
  const positions = isAlarm
    ? createSingleWallPositions(position, halfWidth)
    : createOpenWallPositions(position, halfWidth, halfDepth);
  const topPositions = isAlarm
    ? createSingleWallPositions(position, halfWidth, wallHeight)
    : createOpenWallPositions(position, halfWidth, halfDepth, wallHeight);
  const wallMaterial = isAlarm
    ? new Cesium.ImageMaterialProperty({
      image: createAlarmWallTexture(),
      transparent: false,
      repeat: new Cesium.Cartesian2(3, 1),
      color: Cesium.Color.WHITE,
    })
    : new Cesium.ColorMaterialProperty(new Cesium.CallbackProperty(() => (
      wallColor.withAlpha(0.16 + pulse.getValue() * 0.15)
    ), false));

  return [
    viewer.entities.add({
      name: isAlarm ? "WebGL Alarm Prism Wall" : "WebGL Energy Prism Wall",
      wall: {
        positions,
        minimumHeights: positions.map(() => 0),
        maximumHeights: positions.map(() => wallHeight),
        material: wallMaterial,
        outline: true,
        outlineColor: wallColor.withAlpha(0.8),
      },
    }),
    viewer.entities.add({
      name: isAlarm ? "WebGL Alarm Base" : "WebGL Energy Base",
      polyline: {
        positions,
        width: isAlarm ? 4 : 2,
        material: wallColor.withAlpha(0.85),
      },
    }),
    viewer.entities.add({
      name: isAlarm ? "WebGL Alarm Top" : "WebGL Energy Top",
      polyline: {
        positions: topPositions,
        width: isAlarm ? 4 : 3,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: isAlarm ? 0.12 : 0.22,
          color: wallColor.withAlpha(0.92),
        }),
      },
    }),
  ];
};

const addGlowEffect = (viewer, position, color) => {
  const glowColor = Cesium.Color.fromCssColorString(color);
  const pulse = createPulse(viewer, 1.6, 0.2, 1);

  return [
    viewer.entities.add({
      name: "WebGL Glow Point",
      position,
      point: {
        pixelSize: new Cesium.CallbackProperty(() => 16 + pulse.getValue() * 18, false),
        color: new Cesium.CallbackProperty(() => (
          glowColor.withAlpha(0.55 + pulse.getValue() * 0.35)
        ), false),
        outlineColor: Cesium.Color.WHITE.withAlpha(0.82),
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    }),
    viewer.entities.add({
      name: "WebGL Glow Disc",
      position,
      ellipse: {
        semiMajorAxis: new Cesium.CallbackProperty(() => 90000 + pulse.getValue() * 150000, false),
        semiMinorAxis: new Cesium.CallbackProperty(() => 90000 + pulse.getValue() * 150000, false),
        height: 900,
        material: new Cesium.ColorMaterialProperty(new Cesium.CallbackProperty(() => (
          glowColor.withAlpha(0.1 + pulse.getValue() * 0.18)
        ), false)),
        outline: true,
        outlineColor: glowColor.withAlpha(0.7),
      },
    }),
  ];
};

export const createWebGLEffect = (viewer, type, position, index) => {
  const effectType = getEffectType(type);
  const entityFactories = {
    flame: () => addFlameEffect(viewer, position, effectType.color),
    fireball: () => addFireballEffect(viewer, position, effectType.color),
    volumeSmoke: () => addVolumeSmokeEffect(viewer, position, effectType.color),
    explosion: () => addExplosionEffect(viewer, position, effectType.color),
    energyWall: () => addWallEffect(viewer, position, effectType.color, false),
    alarmWall: () => addWallEffect(viewer, position, effectType.color, true),
    glow: () => addGlowEffect(viewer, position, effectType.color),
  };
  const entities = (entityFactories[type] || entityFactories.flame)();
  const cartographic = Cesium.Cartographic.fromCartesian(position);

  return {
    id: `${type}-${Date.now()}-${index}`,
    type,
    label: effectType.label,
    lon: Cesium.Math.toDegrees(cartographic.longitude).toFixed(3),
    lat: Cesium.Math.toDegrees(cartographic.latitude).toFixed(3),
    entities,
  };
};

export const removeWebGLEffect = (viewer, effect) => {
  if (!viewer || !effect) return;
  effect.entities.forEach((entity) => {
    if (typeof entity.destroy === "function") {
      entity.destroy();
      return;
    }
    viewer.entities.remove(entity);
  });
};
