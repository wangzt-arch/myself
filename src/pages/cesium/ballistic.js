/**
 * 弹道打击模块
 *
 * 功能：在 Cesium 地球上选择起点和终点，生成不同类型的弹道曲线，
 *       发射导弹模型沿轨迹飞行，到达终点触发体散射火爆炸特效，支持循环播放。
 *
 * 状态机流转：
 *   IDLE → SELECTING_START → SELECTING_END → READY ⇄ FIRING
 *
 * 弹道类型：
 *   - 抛物线/椭圆弧/高抛/低抛：简单纵向弧线
 *   - 螺旋/波浪/双弧弹跳/螺旋突进/S型机动/锯齿弹道：带横向偏移的复杂轨迹
 *
 * 核心流程：
 *   1. 用户点击"选择目标" → 进入选点模式
 *   2. 点击地球选起点 → 即时显示起点标记
 *   3. 点击地球选终点 → 即时显示终点标记 + 轨迹线
 *   4. 点击"发射" → 导弹模型沿曲线飞行，朝向跟随轨迹切线方向
 *   5. 到达终点 → 触发体散射火特效 → 2.5秒后循环发射
 */

import * as Cesium from "cesium";
import { createWebGLEffect, removeWebGLEffect } from "./effects";
import missile1Model from "./models/missile_1.glb";
import missile2Model from "./models/missile_2.glb";

// ==================== 弹道状态枚举 ====================

/** 弹道打击状态枚举 */
export const BALLISTIC_STATUS = {
  IDLE: "idle",                     // 空闲，未开始选点
  SELECTING_START: "selecting_start", // 正在等待用户选择起点
  SELECTING_END: "selecting_end",     // 正在等待用户选择终点
  READY: "ready",                     // 已选完两点，可发射
  FIRING: "firing",                   // 导弹飞行中
};

// ==================== 弹道类型配置 ====================

/** 弹道类型列表，用于UI渲染和类型映射 */
export const BALLISTIC_TYPES = [
  { key: "parabola", label: "抛物线", description: "经典抛物线弹道" },
  { key: "ellipse", label: "椭圆弧", description: "平滑椭圆轨迹" },
  { key: "highArc", label: "高抛弹道", description: "高弧度远程打击" },
  { key: "lowArc", label: "低抛弹道", description: "低弧度快速打击" },
  { key: "qianXuesen", label: "助推滑翔弹道", description: "助推滑翔弹跳轨迹" },
  { key: "spiral", label: "螺旋弹道", description: "螺旋上升轨迹" },
  { key: "wave", label: "波浪弹道", description: "正弦波浪轨迹" },
  { key: "doubleArc", label: "双弧弹跳", description: "双弧线弹跳打击" },
  { key: "corkscrew", label: "螺旋突进", description: "密集螺旋突进" },
  { key: "sCurve", label: "S型机动", description: "S型规避弹道" },
  { key: "zigzag", label: "锯齿弹道", description: "锯齿状机动" },
];

// ==================== 弹道曲线计算函数 ====================
//
// 所有计算函数的签名统一为：
//   (start: Cartesian3, end: Cartesian3, maxHeight?: number, segments?: number) => Cartesian3[]
//
// 核心思路：
//   1. 将起终点转换为 Cartographic 弧度坐标
//   2. 用 EllipsoidGeodesic 计算地表距离
//   3. 根据距离动态计算最大高度（clamp到合理范围）
//   4. 沿路径按 segments 数量插值，叠加高度曲线
//   5. 复杂弹道额外叠加横向偏移（基于 startHeading 计算法线方向）
// ==========================================================

/**
 * 抛物线弹道
 * 高度公式：h = 4 * H_max * t * (1-t)，经典二次抛物线
 */
const calculateParabolaCurve = (start, end, maxHeight = 500000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  // 动态最大高度：距离的15%，最低10万米，最高50万米
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.15, 100000), maxHeight);

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + 4 * dynamicMaxHeight * t * (1 - t);
    points.push(Cesium.Cartesian3.fromRadians(lon, lat, height));
  }
  return points;
};

/**
 * 椭圆弧弹道
 * 高度公式：h = H_max * sin(t * π)，半周期正弦曲线
 */
const calculateEllipseCurve = (start, end, maxHeight = 500000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  // 动态最大高度：距离的20%，最低15万米
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.2, 150000), maxHeight);

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const angle = t * Math.PI;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + dynamicMaxHeight * Math.sin(angle);
    points.push(Cesium.Cartesian3.fromRadians(lon, lat, height));
  }
  return points;
};

/**
 * 高抛弹道
 * 高度公式：h = H_max * sin(t * π)，H_max 为距离的30%，最低30万米
 * 适合远程高弧度打击
 */
const calculateHighArcCurve = (start, end, maxHeight = 800000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.3, 300000), maxHeight);

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + dynamicMaxHeight * Math.sin(t * Math.PI);
    points.push(Cesium.Cartesian3.fromRadians(lon, lat, height));
  }
  return points;
};

/**
 * 低抛弹道
 * 高度公式：h = 4 * H_max * t * (1-t)，H_max 为距离的8%，最低5万米
 * 适合近距离快速打击
 */
const calculateLowArcCurve = (start, end, maxHeight = 300000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.08, 50000), maxHeight);

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + 4 * dynamicMaxHeight * t * (1 - t);
    points.push(Cesium.Cartesian3.fromRadians(lon, lat, height));
  }
  return points;
};

/**
 * 钱学森弹道（助推-滑翔弹道）
 *
 * 特点：导弹先以弹道弧上升，进入大气层边缘后借助气动升力
 * 多次滑翔弹跳，振幅逐渐衰减，最终命中目标。
 * 与传统弹道导弹相比，射程更远、轨迹更难预测。
 *
 * 实现方式：
 *   1. 前15%：助推上升段，快速爬升到峰值高度
 *   2. 15%~100%：滑翔弹跳段，用衰减正弦波模拟大气层边缘弹跳
 *      每次弹跳高度为前一次的 70%，共约4次弹跳
 */
const calculateQianXuesenCurve = (start, end, maxHeight = 600000, segments = 300) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  // 助推段峰值高度
  const boostHeight = Math.min(Math.max(distance * 0.25, 200000), maxHeight);
  // 滑翔段首次弹跳高度
  const glideHeight = boostHeight * 0.6;
  // 衰减系数：每次弹跳高度为前一次的 70%
  const decayRate = 0.7;
  // 弹跳次数
  const bounceCount = 4;
  // 助推段占比
  const boostRatio = 0.15;

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    let height;

    if (t < boostRatio) {
      // 助推上升段：快速爬升到峰值（半抛物线）
      const t1 = t / boostRatio;
      height = baseHeight + boostHeight * Math.sin(t1 * Math.PI / 2);
    } else {
      // 滑翔弹跳段：衰减正弦波
      const t2 = (t - boostRatio) / (1 - boostRatio);
      // 叠加多次衰减弹跳
      let glideOffset = 0;
      for (let b = 0; b < bounceCount; b++) {
        // 每次弹跳的频率和振幅
        const freq = (b + 1) * 1.2;
        const amp = Math.pow(decayRate, b);
        // 使用 sin 包络使弹跳两端归零
        const envelope = Math.sin(t2 * Math.PI);
        glideOffset += Math.sin(t2 * Math.PI * freq) * amp * envelope;
      }
      // 归一化并应用滑翔高度
      height = baseHeight + glideHeight * Math.abs(glideOffset) / bounceCount;
      // 确保高度不低于助推段末尾高度的平滑过渡
      const boostEndHeight = boostHeight * Math.sin(Math.PI / 2);
      const transitionT = Math.max(0, 1 - (t - boostRatio) / 0.1);
      const minGlideHeight = boostEndHeight * 0.4 * (1 - t2);
      height = Math.max(height, baseHeight + minGlideHeight + glideHeight * 0.15 * Math.sin(t2 * Math.PI));
    }

    points.push(Cesium.Cartesian3.fromRadians(lon, lat, height));
  }
  return points;
};

/**
 * 螺旋弹道
 * 纵向：sin(t * π) 弧线
 * 横向：4圈螺旋偏移，偏移半径按 sin(t * π) 衰减（两端归零）
 */
const calculateSpiralCurve = (start, end, maxHeight = 600000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.25, 200000), maxHeight);
  const earthRadius = 6371000;

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + dynamicMaxHeight * Math.sin(t * Math.PI);
    // 4圈螺旋偏移
    const offsetAngle = t * Math.PI * 4;
    const offsetRadius = Math.sin(t * Math.PI) * distance * 0.02;
    const offsetLon = Math.cos(offsetAngle) * offsetRadius / earthRadius;
    const offsetLat = Math.sin(offsetAngle) * offsetRadius / earthRadius;
    points.push(Cesium.Cartesian3.fromRadians(lon + offsetLon, lat + offsetLat, height));
  }
  return points;
};

/**
 * 波浪弹道
 * 纵向：sin(t * π) 弧线
 * 横向：6个正弦波峰沿路径法线方向波动，振幅按 sin(t * π) 衰减
 * 法线方向由 geodesic.startHeading 确定
 */
const calculateWaveCurve = (start, end, maxHeight = 500000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.12, 80000), maxHeight);
  const waveCount = 6;
  const waveAmplitude = distance * 0.03;
  const earthRadius = 6371000;

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + dynamicMaxHeight * Math.sin(t * Math.PI);
    // 横向波浪偏移，两端衰减
    const waveOffset = Math.sin(t * Math.PI * waveCount) * waveAmplitude * Math.sin(t * Math.PI);
    const perpLon = -Math.sin(geodesic.startHeading) * waveOffset / earthRadius;
    const perpLat = Math.cos(geodesic.startHeading) * waveOffset / earthRadius;
    points.push(Cesium.Cartesian3.fromRadians(lon + perpLon, lat + perpLat, height));
  }
  return points;
};

/**
 * 双弧弹跳弹道
 * 两段正弦弧叠加，形成"起→落→起→落"的弹跳效果
 * 弹跳1: 0%~55% 区间的主弧（高）
 * 弹跳2: 45%~100% 区间的副弧（较低），与弹跳1重叠平滑过渡
 * 中间低谷保底不低于弧高的25%，避免贴地
 */
const calculateDoubleArcCurve = (start, end, maxHeight = 400000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  const arcHeight = Math.min(Math.max(distance * 0.18, 150000), maxHeight);

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    // 弹跳1：前55%的主弧
    const t1 = Math.min(t / 0.55, 1);
    const bounce1 = Math.sin(t1 * Math.PI);
    // 弹跳2：后55%的副弧（高度为弹跳1的55%），与弹跳1在45%~55%重叠
    const t2 = Math.max(0, Math.min((t - 0.45) / 0.55, 1));
    const bounce2 = Math.sin(t2 * Math.PI) * 0.55;
    // 叠加两段弧，中间低谷保底25%
    const combined = bounce1 + bounce2;
    const minFloor = 0.25 * Math.sin(t * Math.PI);
    const height = baseHeight + arcHeight * Math.max(combined, minFloor);
    points.push(Cesium.Cartesian3.fromRadians(lon, lat, height));
  }
  return points;
};

/**
 * 螺旋突进弹道
 * 纵向：sin(t * π) 弧线
 * 横向：12圈密集螺旋，半径按 sin(t * π) 衰减
 * 与普通螺旋弹道相比：圈数更多(12vs4)、半径更小、插值更密(300段)
 */
const calculateCorkscrewCurve = (start, end, maxHeight = 500000, segments = 300) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.2, 150000), maxHeight);
  const earthRadius = 6371000;
  const spiralCount = 12;
  const spiralRadius = distance * 0.015;

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + dynamicMaxHeight * Math.sin(t * Math.PI);
    const angle = t * Math.PI * 2 * spiralCount;
    const radius = spiralRadius * Math.sin(t * Math.PI);
    const offsetLon = Math.cos(angle) * radius / earthRadius;
    const offsetLat = Math.sin(angle) * radius / earthRadius;
    points.push(Cesium.Cartesian3.fromRadians(lon + offsetLon, lat + offsetLat, height));
  }
  return points;
};

/**
 * S型机动弹道
 * 纵向：4 * H_max * t * (1-t) 抛物线弧
 * 横向：一个完整正弦波（2π），振幅中间最大两端为零
 * 模拟导弹规避机动
 */
const calculateSCurve = (start, end, maxHeight = 400000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.15, 100000), maxHeight);
  const sAmplitude = distance * 0.08;
  const earthRadius = 6371000;

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + dynamicMaxHeight * t * (1 - t) * 4;
    // S型横向偏移，中间大两端小
    const sOffset = Math.sin(t * Math.PI * 2) * sAmplitude * (1 - Math.abs(t - 0.5) * 2);
    const perpLon = -Math.sin(geodesic.startHeading) * sOffset / earthRadius;
    const perpLat = Math.cos(geodesic.startHeading) * sOffset / earthRadius;
    points.push(Cesium.Cartesian3.fromRadians(lon + perpLon, lat + perpLat, height));
  }
  return points;
};

/**
 * 锯齿弹道
 * 纵向：sin(t * π) 弧线
 * 横向：8段锯齿状折线，每段为三角波，振幅按 sin(t * π) 衰减
 * 模拟导弹规避机动的锯齿形路径
 */
const calculateZigzagCurve = (start, end, maxHeight = 350000, segments = 200) => {
  const points = [];
  const startCarto = Cesium.Cartographic.fromCartesian(start);
  const endCarto = Cesium.Cartographic.fromCartesian(end);
  const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
  const distance = geodesic.surfaceDistance;
  const dynamicMaxHeight = Math.min(Math.max(distance * 0.12, 80000), maxHeight);
  const zigzagCount = 8;
  const zigzagAmplitude = distance * 0.04;
  const earthRadius = 6371000;

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const lon = Cesium.Math.lerp(startCarto.longitude, endCarto.longitude, t);
    const lat = Cesium.Math.lerp(startCarto.latitude, endCarto.latitude, t);
    const baseHeight = Cesium.Math.lerp(startCarto.height, endCarto.height, t);
    const height = baseHeight + dynamicMaxHeight * Math.sin(t * Math.PI);
    // 三角波生成锯齿偏移
    const phase = (t * zigzagCount) % 1;
    const zigzagOffset = (phase < 0.5 ? phase * 2 - 0.5 : 1.5 - phase * 2) * zigzagAmplitude * Math.sin(t * Math.PI);
    const perpLon = -Math.sin(geodesic.startHeading) * zigzagOffset / earthRadius;
    const perpLat = Math.cos(geodesic.startHeading) * zigzagOffset / earthRadius;
    points.push(Cesium.Cartesian3.fromRadians(lon + perpLon, lat + perpLat, height));
  }
  return points;
};

// ==================== 曲线计算器映射 ====================

/** 弹道类型 key → 计算函数的映射表 */
const CURVE_CALCULATORS = {
  parabola: calculateParabolaCurve,
  ellipse: calculateEllipseCurve,
  highArc: calculateHighArcCurve,
  lowArc: calculateLowArcCurve,
  qianXuesen: calculateQianXuesenCurve,
  spiral: calculateSpiralCurve,
  wave: calculateWaveCurve,
  doubleArc: calculateDoubleArcCurve,
  corkscrew: calculateCorkscrewCurve,
  sCurve: calculateSCurve,
  zigzag: calculateZigzagCurve,
};

// ==================== 辅助函数 ====================

/**
 * 根据弹道类型选择导弹模型
 * 复杂弹道使用 missile_2，简单弹道使用 missile_1
 */
const getMissileModelForType = (curveType) => {
  const complexTypes = ["spiral", "corkscrew", "wave", "zigzag", "sCurve", "doubleArc", "qianXuesen"];
  return complexTypes.includes(curveType) ? missile2Model : missile1Model;
};

/**
 * 在地球上创建带标签的点标记
 * @param {Cesium.Viewer} viewer - Cesium viewer 实例
 * @param {Cesium.Cartesian3} position - 标记位置（笛卡尔坐标）
 * @param {string} labelText - 标签文字（如"起点"/"终点"）
 * @returns {Cesium.Entity} 创建的实体
 */
const createPointMarker = (viewer, position, labelText) => {
  return viewer.entities.add({
    name: `Ballistic ${labelText}`,
    position,
    point: {
      pixelSize: 14,
      color: Cesium.Color.fromCssColorString("#00f5ff"),
      outlineColor: Cesium.Color.WHITE,
      outlineWidth: 2,
      heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
    },
    label: {
      text: labelText,
      font: "bold 14px sans-serif",
      fillColor: Cesium.Color.WHITE,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
      pixelOffset: new Cesium.Cartesian2(0, -18),
      heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
    },
  });
};

// ==================== 曲线平滑工具 ====================

/**
 * 对曲线点做移动平均平滑，消除转折点的硬折角
 * 使用加权滑动窗口（两端点权重更高），保留首末位置不变
 *
 * @param {Cesium.Cartesian3[]} points - 原始曲线点
 * @param {number} [windowSize=5] - 平滑窗口大小，越大越平滑
 * @returns {Cesium.Cartesian3[]} 平滑后的曲线点
 */
const smoothCurvePoints = (points, windowSize = 5) => {
  if (points.length < windowSize) return points;
  const halfWin = Math.floor(windowSize / 2);
  const smoothed = [];

  for (let i = 0; i < points.length; i++) {
    // 首末点保持不变
    if (i < halfWin || i >= points.length - halfWin) {
      smoothed.push(points[i]);
      continue;
    }
    // 加权移动平均：中心权重最大，向两端递减
    let sumX = 0, sumY = 0, sumZ = 0, sumW = 0;
    for (let j = -halfWin; j <= halfWin; j++) {
      const weight = halfWin + 1 - Math.abs(j);
      const p = points[i + j];
      sumX += p.x * weight;
      sumY += p.y * weight;
      sumZ += p.z * weight;
      sumW += weight;
    }
    smoothed.push(new Cesium.Cartesian3(sumX / sumW, sumY / sumW, sumZ / sumW));
  }
  return smoothed;
};

// ==================== 弹道控制器 ====================

/**
 * 创建弹道打击控制器
 *
 * 控制器是一个包含状态、实体管理和交互逻辑的对象，
 * 通过闭包持有 viewer 引用，对外暴露操作方法。
 *
 * @param {Cesium.Viewer} viewer - Cesium viewer 实例
 * @param {Function} onStatusChange - 状态变化回调，用于同步 React 组件状态
 * @returns {Object} 弹道控制器对象
 */
export const createBallisticController = (viewer, onStatusChange) => {
  const controller = {
    viewer,
    status: BALLISTIC_STATUS.IDLE,  // 当前状态
    startPoint: null,               // 起点 Cartesian3
    endPoint: null,                 // 终点 Cartesian3
    curvePoints: [],                // 弹道曲线插值点数组
    curveType: "parabola",          // 当前弹道类型
    maxHeight: 500000,              // 最大飞行高度（米）
    fireStartTime: 0,               // 发射开始时间戳
    duration: 3,                    // 飞行持续时间（秒）
    loop: true,                     // 是否循环播放
    entities: [],                   // 当前管理的 Cesium 实体列表
    clickHandler: null,             // 地球点击事件处理器
    _effect: null,                  // 当前爆炸特效实例
    _effectTimer: null,             // 爆炸特效清除定时器
    _loopTimer: null,               // 循环发射定时器
    onStatusChange,                 // 状态变化回调

    /**
     * 更新状态并通知外部
     * @param {string} status - 新状态值
     */
    _setStatus(status) {
      controller.status = status;
      if (controller.onStatusChange) controller.onStatusChange(status);
    },

    /**
     * 清除所有管理的 Cesium 实体（标记、轨迹线、导弹等）
     */
    _clearEntities() {
      controller.entities.forEach((e) => viewer.entities.remove(e));
      controller.entities = [];
    },

    /**
     * 清除爆炸特效和相关定时器
     */
    _clearEffect() {
      if (controller._effect) {
        removeWebGLEffect(viewer, controller._effect);
        controller._effect = null;
      }
      if (controller._effectTimer) {
        clearTimeout(controller._effectTimer);
        controller._effectTimer = null;
      }
      if (controller._loopTimer) {
        clearTimeout(controller._loopTimer);
        controller._loopTimer = null;
      }
    },

    /**
     * 根据当前弹道类型计算曲线插值点
     * @returns {Cesium.Cartesian3[]} 曲线点数组
     */
    _calculateCurve() {
      const calculator = CURVE_CALCULATORS[controller.curveType] || CURVE_CALCULATORS.parabola;
      const rawPoints = calculator(controller.startPoint, controller.endPoint, controller.maxHeight);
      return smoothCurvePoints(rawPoints, 7);
    },

    /**
     * 创建弹道轨迹线（青色发光效果）
     * @returns {Cesium.Entity} 轨迹线实体
     */
    _createTrajectoryLine() {
      return viewer.entities.add({
        name: "Ballistic Trajectory",
        polyline: {
          positions: controller.curvePoints,
          width: 3,
          material: new Cesium.PolylineGlowMaterialProperty({
            glowPower: 0.3,
            color: Cesium.Color.fromCssColorString("#00f5ff").withAlpha(0.9),
          }),
        },
      });
    },

    /**
     * 切换弹道类型
     * READY 状态：重新计算并刷新轨迹线
     * FIRING 状态：重新计算曲线，清除旧实体后重建标记+轨迹线+导弹，重新开始飞行
     * @param {string} type - 弹道类型 key
     */
    setCurveType(type) {
      controller.curveType = type;
      if (controller.status === BALLISTIC_STATUS.READY || controller.status === BALLISTIC_STATUS.FIRING) {
        controller._clearEffect();
        controller._clearEntities();

        // 重建起点终点标记和轨迹线
        const marker1 = createPointMarker(viewer, controller.startPoint, "起点");
        const marker2 = createPointMarker(viewer, controller.endPoint, "终点");
        controller.entities.push(marker1, marker2);

        controller.curvePoints = controller._calculateCurve();
        const trajectory = controller._createTrajectoryLine();
        controller.entities.push(trajectory);

        // FIRING 状态下重新发射导弹
        if (controller.status === BALLISTIC_STATUS.FIRING) {
          controller.fireStartTime = Date.now();
          controller._launchMissile();
        }
      }
      // 触发状态同步，确保 UI 高亮正确
      if (controller.onStatusChange) controller.onStatusChange(controller.status);
    },

    /**
     * 开始选点模式
     * 重置状态 → 进入选择起点阶段 → 绑定地球点击事件
     */
    startSelecting() {
      controller.reset();
      controller._setStatus(BALLISTIC_STATUS.SELECTING_START);
      controller._bindClickHandler();
    },

    /**
     * 设置起点
     * 立即在地球上显示起点标记，状态切换为选择终点
     * @param {Cesium.Cartesian3} position - 起点位置
     */
    setStartPoint(position) {
      controller.startPoint = position;
      const marker = createPointMarker(viewer, position, "起点");
      controller.entities.push(marker);
      controller._setStatus(BALLISTIC_STATUS.SELECTING_END);
    },

    /**
     * 设置终点
     * 立即显示终点标记和轨迹线，状态切换为准备就绪，解除点击监听
     * @param {Cesium.Cartesian3} position - 终点位置
     */
    setEndPoint(position) {
      controller.endPoint = position;
      const marker = createPointMarker(viewer, position, "终点");
      controller.entities.push(marker);

      controller.curvePoints = controller._calculateCurve();
      const trajectory = controller._createTrajectoryLine();
      controller.entities.push(trajectory);

      controller._setStatus(BALLISTIC_STATUS.READY);
      controller._unbindClickHandler();
    },

    /**
     * 发射导弹（入口方法）
     * 从 READY 状态进入 FIRING 状态，然后调用 _launchMissile 创建导弹实体
     */
    fire() {
      if (controller.status !== BALLISTIC_STATUS.READY) return;
      controller._setStatus(BALLISTIC_STATUS.FIRING);
      controller.fireStartTime = Date.now();

      // 根据起终点距离计算飞行时长
      const startCarto = Cesium.Cartographic.fromCartesian(controller.startPoint);
      const endCarto = Cesium.Cartographic.fromCartesian(controller.endPoint);
      const geodesic = new Cesium.EllipsoidGeodesic(startCarto, endCarto);
      const distance = geodesic.surfaceDistance;
      controller.duration = Math.max(2.5, Math.min(6, distance / 800000));

      controller._launchMissile();
    },

    /**
     * 创建导弹实体并启动飞行检测循环
     * 被 fire() 和 setCurveType() 复用
     *
     * 核心流程：
     *   1. 用 SampledPositionProperty 采样曲线点，VelocityOrientationProperty 自动计算朝向
     *   2. CallbackProperty 驱动虚拟时间推进位置和朝向
     *   3. requestAnimationFrame 循环检测飞行完成
     *   4. 到达终点 → 触发体散射火特效 → 2秒后清除特效
     *   5. 循环模式下 2.5 秒后重置发射时间重新飞行
     */
    _launchMissile() {
      // 根据弹道类型选择导弹模型
      const missileModelUri = getMissileModelForType(controller.curveType);

      // 虚拟起始时间（不修改 viewer 时钟）
      const virtualStart = Cesium.JulianDate.now();

      // 用 SampledPositionProperty 采样所有曲线点
      const sampledPosition = new Cesium.SampledPositionProperty(Cesium.ReferenceFrame.FIXED);
      for (let i = 0; i < controller.curvePoints.length; i++) {
        const t = i / (controller.curvePoints.length - 1);
        const sampleTime = Cesium.JulianDate.addSeconds(virtualStart, t * controller.duration, new Cesium.JulianDate());
        sampledPosition.addSample(sampleTime, controller.curvePoints[i]);
      }
      sampledPosition.setInterpolationOptions({
        interpolationDegree: 1,
        interpolationAlgorithm: Cesium.LinearApproximation,
      });

      // VelocityOrientationProperty 自动根据位置变化率计算朝向
      const orientationProperty = new Cesium.VelocityOrientationProperty(sampledPosition);

      // 创建导弹实体
      const missile = viewer.entities.add({
        name: "Ballistic Missile",
        // 位置：根据飞行时间查询采样属性
        position: new Cesium.CallbackProperty(() => {
          const elapsed = (Date.now() - controller.fireStartTime) / 1000;
          const virtualTime = Cesium.JulianDate.addSeconds(virtualStart, elapsed, new Cesium.JulianDate());
          return sampledPosition.getValue(virtualTime);
        }, false),
        // 朝向：VelocityOrientationProperty 自动计算
        orientation: new Cesium.CallbackProperty(() => {
          const elapsed = (Date.now() - controller.fireStartTime) / 1000;
          const virtualTime = Cesium.JulianDate.addSeconds(virtualStart, elapsed, new Cesium.JulianDate());
          return orientationProperty.getValue(virtualTime) || Cesium.Quaternion.IDENTITY;
        }, false),
        model: {
          uri: missileModelUri,
          scale: 5000,
          minimumPixelSize: 24,
          maximumScale: 500000,
          color: Cesium.Color.WHITE,
          colorBlendMode: Cesium.ColorBlendMode.NONE,
        },
      });
      controller.entities.push(missile);

      /**
       * 飞行完成检测循环
       * 使用 requestAnimationFrame 逐帧检测是否到达终点
       * 到达后：触发爆炸特效 → 延迟清除特效 → 循环模式下延迟重新发射
       */
      const checkCompletion = () => {
        if (controller.status !== BALLISTIC_STATUS.FIRING) return;

        const elapsed = (Date.now() - controller.fireStartTime) / 1000;
        if (elapsed >= controller.duration) {
          // 到达终点，触发体散射火爆炸特效
          controller._clearEffect();
          controller._effect = createWebGLEffect(viewer, "volumeDiffuseFire", controller.endPoint, Date.now());

          // 2秒后清除爆炸特效
          controller._effectTimer = setTimeout(() => {
            controller._clearEffect();
          }, 2000);

          // 循环模式：2.5秒后重新发射
          if (controller.loop) {
            controller._loopTimer = setTimeout(() => {
              if (controller.status === BALLISTIC_STATUS.FIRING) {
                controller.fireStartTime = Date.now();
                requestAnimationFrame(checkCompletion);
              }
            }, 2500);
          }
        } else {
          // 未到达终点，继续检测
          requestAnimationFrame(checkCompletion);
        }
      };

      requestAnimationFrame(checkCompletion);
    },

    /**
     * 停止打击（回到 READY 状态，保留选点和轨迹线）
     */
    stop() {
      controller._setStatus(BALLISTIC_STATUS.READY);
      controller._clearEffect();
    },

    /**
     * 完全重置（清除所有实体、特效、事件监听，回到 IDLE 状态）
     */
    reset() {
      controller._clearEffect();
      controller._clearEntities();
      controller._unbindClickHandler();
      controller.startPoint = null;
      controller.endPoint = null;
      controller.curvePoints = [];
      controller._setStatus(BALLISTIC_STATUS.IDLE);
    },

    /**
     * 绑定地球左键点击事件
     * 根据当前状态决定是设置起点还是终点
     */
    _bindClickHandler() {
      controller._unbindClickHandler();
      const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
      handler.setInputAction((click) => {
        // 从屏幕坐标射线拾取地球表面位置
        const ray = viewer.camera.getPickRay(click.position);
        const position = ray ? viewer.scene.globe.pick(ray, viewer.scene) : null;
        if (!position) {
          return;
        }
        if (controller.status === BALLISTIC_STATUS.SELECTING_START) {
          controller.setStartPoint(position);
        } else if (controller.status === BALLISTIC_STATUS.SELECTING_END) {
          controller.setEndPoint(position);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      controller.clickHandler = handler;
    },

    /**
     * 解除地球点击事件
     */
    _unbindClickHandler() {
      if (controller.clickHandler) {
        controller.clickHandler.destroy();
        controller.clickHandler = null;
      }
    },

    /**
     * 销毁控制器，释放所有资源
     */
    destroy() {
      controller._clearEffect();
      controller._clearEntities();
      controller._unbindClickHandler();
    },

    /**
     * 获取当前状态的快照（用于 React 组件渲染）
     * @returns {Object} 包含 status、curveType、起终点经纬度的对象
     */
    getSnapshot() {
      const toLonLat = (pos) => {
        if (!pos) return null;
        const c = Cesium.Cartographic.fromCartesian(pos);
        return {
          lon: Cesium.Math.toDegrees(c.longitude).toFixed(4),
          lat: Cesium.Math.toDegrees(c.latitude).toFixed(4),
        };
      };
      return {
        status: controller.status,
        curveType: controller.curveType,
        startLonLat: toLonLat(controller.startPoint),
        endLonLat: toLonLat(controller.endPoint),
      };
    },
  };

  return controller;
};

// ==================== 工具函数 ====================

/**
 * 获取弹道状态的中文显示文本
 * @param {string} status - 状态值
 * @returns {string} 中文状态文本
 */
export const getBallisticStatusText = (status) => {
  const texts = {
    [BALLISTIC_STATUS.IDLE]: "空闲",
    [BALLISTIC_STATUS.SELECTING_START]: "请选择起点",
    [BALLISTIC_STATUS.SELECTING_END]: "请选择终点",
    [BALLISTIC_STATUS.READY]: "准备就绪",
    [BALLISTIC_STATUS.FIRING]: "正在打击",
  };
  return texts[status] || "未知";
};