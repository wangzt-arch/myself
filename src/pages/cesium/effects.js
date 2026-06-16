import * as Cesium from "cesium";
import BallOfFireEffect from "./effect/BallOfFireEffect";
import EmberSphereEffect from "./effect/EmberSphereEffect";
import ExplosionSphereEffect from "./effect/ExplosionSphereEffect";
import VolumeSmokeEffect from "./effect/VolumeSmokeEffect";
import CircleBlastEffect from "./effect/CircleBlastEffect";
import CircleWarnEffect from "./effect/CircleWarnEffect";
import CircleBlurEffect from "./effect/CircleBlurEffect";
import CircleDiffuseFireEffect from "./effect/CircleDiffuseFireEffect";
import CircleDisturbEffect from "./effect/CircleDisturbEffect";
import CircleElectricAreaEffect from "./effect/CircleElectricAreaEffect";
import CircleDiffusionEffect from "./effect/CircleDiffusionEffect";
import CircleGifEffect from "./effect/CircleGifEffect";
import CircleHelicalLineEffect from "./effect/CircleHelicalLineEffect";
import CircleFlashEffect from "./effect/CircleFlashEffect";
import CircleRotateColorLineEffect from "./effect/CircleRotateColorLineEffect";
import CircleRotateGarlandEffect from "./effect/CircleRotateGarlandEffect";
import CircleRotateHaloEffect from "./effect/CircleRotateHaloEffect";
import CircleTyphoonEffect from "./effect/CircleTyphoonEffect";
import CircleWrapFireEffect from "./effect/CircleWrapFireEffect";
import ElectricSphereEffect from "./effect/ElectricSphereEffect";
import VolumeAreaLightningEffect from "./effect/VolumeAreaLightningEffect";
import VolumeArrowAttackEffect from "./effect/VolumeArrowAttackEffect";
import VolumeDiffuseFireEffect from "./effect/VolumeDiffuseFireEffect";
import VolumeDisturbLineEffect from "./effect/VolumeDisturbLineEffect";
import AreaRainEffect from "./effect/AreaRainEffect";
import AreaSnowEffect from "./effect/AreaSnowEffect";
import FireworkEffect from "./effect/FireworkEffect";

export const EFFECT_TYPES = [
  { key: "flame", label: "火球", color: "#ff7a18" },
  { key: "fireball", label: "火焰", color: "#ff9d1c" },
  { key: "circleDiffuseFire", label: "圆形散射火", color: "#ff5a12" },
  { key: "volumeSmoke", label: "体积烟", color: "#b6b0a8" },
  { key: "circleBlast", label: "圆形爆炸", color: "#ff7a1a" },
  { key: "circleWarn", label: "圆形预警", color: "#ff2a1f" },
  { key: "circleBlur", label: "圆形模糊", color: "#ff6a42" },
  { key: "circleDisturb", label: "干扰圈", color: "#54d7ff" },
  { key: "circleElectricArea", label: "电域", color: "#40b7ff" },
  { key: "circleDiffusion", label: "圆形扩散", color: "#00f5ff" },
  { key: "circleGif", label: "静态爆炸", color: "#ff8a24" },
  { key: "circleHelicalLine", label: "螺旋线", color: "#8fff6a" },
  { key: "circleRotateColorLine", label: "旋转彩线", color: "#8b5cff" },
  { key: "circleRotateGarland", label: "旋转花环", color: "#ff9d1c" },
  { key: "circleRotateHalo", label: "旋转光圈", color: "#40b7ff" },
  { key: "circleTyphoon", label: "台风云团", color: "#8d8b8b" },
  { key: "circleWrapFire", label: "环绕火环", color: "#ff4d00" },
  { key: "electricSphere", label: "电磁弧", color: "#00e5ff" },
  { key: "volumeAreaLightning", label: "区域闪电", color: "#aabbff" },
  { key: "volumeArrowAttack", label: "万箭穿心", color: "#ff3333" },
  { key: "volumeDiffuseFire", label: "体散射火", color: "#ff6600" },
  { key: "volumeDisturbLine", label: "干扰线域", color: "#00ff88" },
  { key: "circleFlash", label: "圆形闪烁", color: "#ffd326" },
  { key: "explosion", label: "爆炸", color: "#ffce45" },
  { key: "energyWall", label: "能量墙", color: "#00f5ff" },
  { key: "alarmWall", label: "警戒墙", color: "#ff4d6d" },
  { key: "glow", label: "彩虹墙", color: "#8fff6a" },
  { key: "areaRain", label: "区域下雨", color: "#88bbff" },
  { key: "areaSnow", label: "区域下雪", color: "#ffffff" },
  { key: "firework", label: "烟花", color: "#ffaa44" },
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
    speed: 1.6,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.82),
    autoAnimate: true,
    Cesium,
  });

  return [volumeSmoke];
};

const addCircleBlastEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const circleBlast = new CircleBlastEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height,
    radius: 220,
    speed: 3.0,
    durationTime: 2.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [circleBlast];
};

const addCircleWarnEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const circleWarn = new CircleWarnEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 40,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.95),
    autoAnimate: true,
    Cesium,
  });

  return [circleWarn];
};

const addCircleBlurEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const circleBlur = new CircleBlurEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 30,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.85),
    autoAnimate: true,
    Cesium,
  });

  return [circleBlur];
};

const addCircleDiffuseFireEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const diffuseFire = new CircleDiffuseFireEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 35,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [diffuseFire];
};

const addCircleDisturbEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const disturb = new CircleDisturbEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 45,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.95),
    autoAnimate: true,
    Cesium,
  });

  return [disturb];
};

const addCircleElectricAreaEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const electricArea = new CircleElectricAreaEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 50,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [electricArea];
};

const addCircleDiffusionEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const diffusion = new CircleDiffusionEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 55,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.88),
    autoAnimate: true,
    Cesium,
  });

  return [diffusion];
};

const addCircleGifEffect = (viewer, position) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const circleGif = new CircleGifEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 60,
    radius: 300000,
    speed: 1.0,
    Cesium,
  });

  return [circleGif];
};

const addCircleHelicalLineEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const helicalLine = new CircleHelicalLineEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 65,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [helicalLine];
};

const addCircleFlashEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const circleFlash = new CircleFlashEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 70,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [circleFlash];
};

const addCircleRotateColorLineEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const rotateColorLine = new CircleRotateColorLineEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 75,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.95),
    autoAnimate: true,
    Cesium,
  });

  return [rotateColorLine];
};

const addCircleRotateGarlandEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const rotateGarland = new CircleRotateGarlandEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 80,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.92),
    autoAnimate: true,
    Cesium,
  });

  return [rotateGarland];
};

const addCircleRotateHaloEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const rotateHalo = new CircleRotateHaloEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 85,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [rotateHalo];
};

const addCircleTyphoonEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const typhoon = new CircleTyphoonEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 90,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [typhoon];
};

const addCircleWrapFireEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const wrapFire = new CircleWrapFireEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 75,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [wrapFire];
};

const addElectricSphereEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const electric = new ElectricSphereEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 80,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [electric];
};

const addVolumeAreaLightningEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const lightning = new VolumeAreaLightningEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 85,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [lightning];
};

const addVolumeArrowAttackEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const arrow = new VolumeArrowAttackEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 85,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [arrow];
};

const addVolumeDiffuseFireEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const fire = new VolumeDiffuseFireEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 85,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [fire];
};

const addVolumeDisturbLineEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const disturb = new VolumeDisturbLineEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 85,
    radius: 300000,
    speed: 3.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });

  return [disturb];
};

const addAreaRainEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const rain = new AreaRainEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 85,
    radius: 300000,
    speed: 2.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.7),
    autoAnimate: true,
    Cesium,
  });
  return [rain];
};

const addAreaSnowEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const snow = new AreaSnowEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 85,
    radius: 300000,
    speed: 1.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.8),
    autoAnimate: true,
    Cesium,
  });
  return [snow];
};

const addFireworkEffect = (viewer, position, color) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  const firework = new FireworkEffect(viewer, {
    longitude: Cesium.Math.toDegrees(cartographic.longitude),
    latitude: Cesium.Math.toDegrees(cartographic.latitude),
    height: cartographic.height + 85,
    radius: 300000,
    speed: 1.0,
    color: Cesium.Color.fromCssColorString(color).withAlpha(0.9),
    autoAnimate: true,
    Cesium,
  });
  return [firework];
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
  // color parameter kept for API consistency, rainbow gradient used internally
  void color;
  const halfWidth = 220000;
  const wallHeight = 270000;
  const positions = createSingleWallPositions(position, halfWidth);

  // Rainbow gradient: red -> orange -> yellow -> green -> cyan -> blue -> purple (top to bottom)
  const rainbowColors = [
    { r: 255, g: 0, b: 0 },      // red (top)
    { r: 255, g: 127, b: 0 },    // orange
    { r: 255, g: 255, b: 0 },    // yellow
    { r: 0, g: 255, b: 0 },      // green
    { r: 0, g: 255, b: 255 },    // cyan
    { r: 0, g: 0, b: 255 },      // blue
    { r: 128, g: 0, b: 128 },    // purple (bottom)
  ];

  // Create gradient texture for wall material
  const createGradientTexture = () => {
    if (typeof document === "undefined") return "";
    const canvas = document.createElement("canvas");
    canvas.width = 64;
    canvas.height = 512;
    const ctx = canvas.getContext("2d");

    const gradient = ctx.createLinearGradient(0, 0, 0, canvas.height);
    const step = 1.0 / (rainbowColors.length - 1);
    for (let i = 0; i < rainbowColors.length; i++) {
      const c = rainbowColors[i];
      gradient.addColorStop(i * step, `rgba(${c.r}, ${c.g}, ${c.b}, 0.7)`);
    }

    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    return canvas.toDataURL("image/png");
  };

  return [
    viewer.entities.add({
      name: "WebGL Gradient Wall",
      wall: {
        positions,
        minimumHeights: positions.map(() => 0),
        maximumHeights: positions.map(() => wallHeight),
        material: new Cesium.ImageMaterialProperty({
          image: createGradientTexture(),
          transparent: true,
          repeat: new Cesium.Cartesian2(1, 1),
        }),
        outline: false,
      },
    }),
  ];
};

export const createWebGLEffect = (viewer, type, position, index) => {
  const effectType = getEffectType(type);
  const entityFactories = {
    flame: () => addFlameEffect(viewer, position, effectType.color),
    fireball: () => addFireballEffect(viewer, position, effectType.color),
    circleDiffuseFire: () => addCircleDiffuseFireEffect(viewer, position, effectType.color),
    volumeSmoke: () => addVolumeSmokeEffect(viewer, position, effectType.color),
    circleBlast: () => addCircleBlastEffect(viewer, position, effectType.color),
    circleWarn: () => addCircleWarnEffect(viewer, position, effectType.color),
    circleBlur: () => addCircleBlurEffect(viewer, position, effectType.color),
    circleDisturb: () => addCircleDisturbEffect(viewer, position, effectType.color),
    circleElectricArea: () => addCircleElectricAreaEffect(viewer, position, effectType.color),
    circleDiffusion: () => addCircleDiffusionEffect(viewer, position, effectType.color),
    circleGif: () => addCircleGifEffect(viewer, position),
    circleHelicalLine: () => addCircleHelicalLineEffect(viewer, position, effectType.color),
    circleRotateColorLine: () => addCircleRotateColorLineEffect(viewer, position, effectType.color),
    circleRotateGarland: () => addCircleRotateGarlandEffect(viewer, position, effectType.color),
    circleRotateHalo: () => addCircleRotateHaloEffect(viewer, position, effectType.color),
    circleTyphoon: () => addCircleTyphoonEffect(viewer, position, effectType.color),
    circleWrapFire: () => addCircleWrapFireEffect(viewer, position, effectType.color),
    electricSphere: () => addElectricSphereEffect(viewer, position, effectType.color),
    volumeAreaLightning: () => addVolumeAreaLightningEffect(viewer, position, effectType.color),
    volumeArrowAttack: () => addVolumeArrowAttackEffect(viewer, position, effectType.color),
    volumeDiffuseFire: () => addVolumeDiffuseFireEffect(viewer, position, effectType.color),
    volumeDisturbLine: () => addVolumeDisturbLineEffect(viewer, position, effectType.color),
    circleFlash: () => addCircleFlashEffect(viewer, position, effectType.color),
    explosion: () => addExplosionEffect(viewer, position, effectType.color),
    energyWall: () => addWallEffect(viewer, position, effectType.color, false),
    alarmWall: () => addWallEffect(viewer, position, effectType.color, true),
    glow: () => addGlowEffect(viewer, position, effectType.color),
    areaRain: () => addAreaRainEffect(viewer, position, effectType.color),
    areaSnow: () => addAreaSnowEffect(viewer, position, effectType.color),
    firework: () => addFireworkEffect(viewer, position, effectType.color),
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
  if (typeof effect.destroy === "function") {
    effect.destroy();
    return;
  }
  if (effect.entities && Array.isArray(effect.entities)) {
    effect.entities.forEach((entity) => {
      if (typeof entity.destroy === "function") {
        entity.destroy();
        return;
      }
      if (viewer.entities.contains(entity)) {
        viewer.entities.remove(entity);
        return;
      }
      if (entity.primitive && viewer.scene.primitives.contains(entity.primitive)) {
        viewer.scene.primitives.remove(entity.primitive);
        return;
      }
      if (viewer.scene.primitives.contains(entity)) {
        viewer.scene.primitives.remove(entity);
      }
    });
  }
};
