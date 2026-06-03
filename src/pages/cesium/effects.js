import * as Cesium from "cesium";

export const EFFECT_TYPES = [
  { key: "flame", label: "火焰", color: "#ff7a18" },
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

const createCirclePositions = (center, radius, count = 72, height = 0) => {
  const transform = Cesium.Transforms.eastNorthUpToFixedFrame(center);
  const positions = [];

  for (let i = 0; i <= count; i += 1) {
    const angle = (i / count) * Cesium.Math.TWO_PI;
    const tangentPoint = Cesium.Matrix4.multiplyByPoint(
      transform,
      new Cesium.Cartesian3(Math.cos(angle) * radius, Math.sin(angle) * radius, height),
      new Cesium.Cartesian3()
    );
    const cartographic = Cesium.Cartographic.fromCartesian(tangentPoint);
    positions.push(Cesium.Cartesian3.fromRadians(cartographic.longitude, cartographic.latitude, height));
  }

  return positions;
};

const addFlameEffect = (viewer, position, color) => {
  const glowColor = Cesium.Color.fromCssColorString(color);
  const hotColor = Cesium.Color.fromCssColorString("#ffd36a");
  const pulse = createPulse(viewer, 1.2, 0.35, 1);

  return [
    viewer.entities.add({
      name: "WebGL Flame Core",
      position,
      cylinder: {
        length: 90000,
        topRadius: 18000,
        bottomRadius: 52000,
        material: new Cesium.ColorMaterialProperty(new Cesium.CallbackProperty(() => (
          hotColor.withAlpha(0.22 + pulse.getValue() * 0.28)
        ), false)),
        outline: true,
        outlineColor: glowColor.withAlpha(0.72),
      },
    }),
    viewer.entities.add({
      name: "WebGL Flame Glow",
      position,
      ellipse: {
        semiMajorAxis: new Cesium.CallbackProperty(() => 70000 + pulse.getValue() * 26000, false),
        semiMinorAxis: new Cesium.CallbackProperty(() => 70000 + pulse.getValue() * 26000, false),
        height: 800,
        material: new Cesium.ColorMaterialProperty(new Cesium.CallbackProperty(() => (
          glowColor.withAlpha(0.16 + pulse.getValue() * 0.16)
        ), false)),
        outline: true,
        outlineColor: glowColor.withAlpha(0.55),
      },
    }),
  ];
};

const addExplosionEffect = (viewer, position, color) => {
  const coreColor = Cesium.Color.fromCssColorString(color);
  const ringColor = Cesium.Color.fromCssColorString("#ff6b35");
  const pulse = createPulse(viewer, 1.8, 0, 1);

  return [
    viewer.entities.add({
      name: "WebGL Explosion Sphere",
      position,
      ellipsoid: {
        radii: new Cesium.CallbackProperty(() => {
          const size = 52000 + pulse.getValue() * 110000;
          return new Cesium.Cartesian3(size, size, size * 0.65);
        }, false),
        material: new Cesium.ColorMaterialProperty(new Cesium.CallbackProperty(() => (
          coreColor.withAlpha(0.1 + (1 - pulse.getValue()) * 0.36)
        ), false)),
        outline: true,
        outlineColor: ringColor.withAlpha(0.7),
      },
    }),
    viewer.entities.add({
      name: "WebGL Explosion Wave",
      position,
      ellipse: {
        semiMajorAxis: new Cesium.CallbackProperty(() => 85000 + pulse.getValue() * 260000, false),
        semiMinorAxis: new Cesium.CallbackProperty(() => 85000 + pulse.getValue() * 260000, false),
        height: 1200,
        material: new Cesium.ColorMaterialProperty(new Cesium.CallbackProperty(() => (
          ringColor.withAlpha(0.24 * (1 - pulse.getValue()))
        ), false)),
        outline: true,
        outlineColor: ringColor.withAlpha(0.8),
      },
    }),
  ];
};

const addWallEffect = (viewer, position, color, isAlarm = false) => {
  const wallColor = Cesium.Color.fromCssColorString(color);
  const pulse = createPulse(viewer, isAlarm ? 1 : 2.4, 0.2, 1);
  const positions = createCirclePositions(position, isAlarm ? 220000 : 260000);

  return [
    viewer.entities.add({
      name: isAlarm ? "WebGL Alarm Wall" : "WebGL Energy Wall",
      wall: {
        positions,
        minimumHeights: positions.map(() => 0),
        maximumHeights: positions.map((_, index) => {
          const wave = Math.sin((index / positions.length) * Cesium.Math.TWO_PI * 3);
          return (isAlarm ? 220000 : 280000) + wave * 26000;
        }),
        material: new Cesium.ColorMaterialProperty(new Cesium.CallbackProperty(() => (
          wallColor.withAlpha(isAlarm ? 0.22 + pulse.getValue() * 0.22 : 0.14 + pulse.getValue() * 0.16)
        ), false)),
        outline: true,
        outlineColor: wallColor.withAlpha(0.8),
      },
    }),
    viewer.entities.add({
      name: isAlarm ? "WebGL Alarm Ring" : "WebGL Energy Ring",
      polyline: {
        positions,
        width: isAlarm ? 3 : 2,
        material: wallColor.withAlpha(0.85),
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
    viewer.entities.remove(entity);
  });
};
